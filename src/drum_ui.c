/*
 * drum_ui.c - control surface + pattern/voice glue for the drum station
 *
 * -----------------------------------------------------------------------------
 */

#include <string.h>

#include "drum_ui.h"
#include "analog_conditioning.h"
#include "hardware_controls.h"
#include "UI_conditioning.h"
#include "params_lfo.h"
#include "params_lfo_period.h"
#include "math_util.h"
#include "led_cont.h"

extern o_analog	analog[NUM_ANALOG_ELEMENTS];

o_drum_chan	drum_chan[NUM_CHANNELS];
uint8_t		drum_selected_chan = 0;
uint8_t		drum_global_edit_mode = 0;

enum DrumPatternEngine	drum_pattern_engine = PATTERN_ENGINE_EUCLID;

GridsState	grids_state;
uint8_t		grids_x       = 128;
uint8_t		grids_y       = 128;
uint8_t		pattern_chaos = 0;	/* off by default: the map/pattern alone is already musical */

/* Clock divide/multiply for Grids' one shared stepper -- unlike the
 * euclidean per-channel clock_rate, this is a single value shared by
 * every Grids-driven channel, same as x/y/chaos, since they all read
 * one grids_state.step position rather than having their own. */
float		grids_clock_divmult_id = LFO_UNITY_DIVMULT_ID;
float		grids_clock_rate       = 1.0f;
static float	grids_step_phase       = 0.0f;

/* Counts down in LED-update ticks; non-zero means "this channel just hit"
 * and its button LED is flashed. Written from OSC_TIM, read/decremented
 * by led_cont.c. */
volatile uint8_t drum_trig_flash[NUM_CHANNELS];

/* Counts down in PWM_OUTS_TIM ticks; non-zero drives that channel's
 * ENV OUT jack high as a gate pulse. Written from OSC_TIM, read/
 * decremented by update_envout_pwm(). */
volatile uint8_t drum_gate_ticks[NUM_CHANNELS];

/* Channel currently held down (button_pressed() != RELEASED), or -1 if
 * none. While held, turning the DEPTH or LFOSPEED encoder routes that
 * channel's CV jack to the corresponding modulation target instead of
 * (only) editing the knob -- see assign_cv_target_if_held(). */
static int8_t drum_held_chan = -1;

/* display_drum_cv_mode() needs this: assign_cv_target_if_held() routes
 * the HELD channel's CV mode, which can differ from drum_selected_chan
 * (you can hold one channel while turning a knob that's still editing
 * whichever channel is selected -- see apply_to_selected_or_all()). */
int8_t drum_ui_held_chan(void) { return drum_held_chan; }

/* Set by assign_cv_target_if_held() the moment a hold actually routes a
 * CV target, cleared at the start of each new hold -- read_channel_buttons()
 * checks this on release to suppress the channel-select/global-edit-toggle
 * action for a hold that was really a CV-routing gesture, not a tap. */
static uint8_t held_chan_cv_assigned = 0;

/* Holding a channel for this long *without* touching a knob (see
 * held_chan_cv_assigned above) clears that channel's CV mode back to
 * DRUM_CV_TARGET_TRIGGER -- the only way back now that routing a target is a
 * plain "set", not a toggle (see assign_cv_target_if_held()). Tracked
 * in ms (HAL_GetTick(), same clock read_automation() already uses)
 * rather than a tick count, since read_drum_ui() runs off the variable-
 * period main loop, not a fixed timer. */
#define DRUM_CV_CLEAR_HOLD_MS	4000u
static uint32_t held_chan_press_start_ms = 0;
static uint8_t  held_chan_cv_cleared = 0;

/* A FINE+press mute toggle queued in performance mode, one per channel.
 * Set here (main loop, read_performance_controls()), consumed and
 * cleared in update_drum_triggers() (OSC_TIM) at the next bar boundary
 * -- same producer/consumer split as pattern_resync_pending below,
 * since bar_start is only computed on the OSC_TIM side. */
static volatile uint8_t mute_toggle_pending[NUM_CHANNELS];

uint8_t drum_ui_mute_pending(uint8_t chan)
{
	return (chan < NUM_CHANNELS) ? mute_toggle_pending[chan] : 0;
}

uint8_t drum_ui_performance_mode(void)
{
	return switch_pressed(VOCTSW) != RELEASED;
}

/* Each channel has a fixed role (category), but it's no longer a 1:1
 * cast onto DrumVoiceCategory: channels E and F both map to
 * DRUM_CAT_OTHER (crash voices moved into DRUM_CAT_OPEN_HAT instead --
 * they read as more hat-like than a distinct instrument, so there's no
 * dedicated crash category any more), hence an explicit table rather
 * than `(DrumVoiceCategory)c`. rotm_LFOSHAPE cycles a channel through
 * only its own category's registry entries (see read_voice_encoders()),
 * so a channel is always "a kick" (etc.) no matter how far you turn it. */
static const DrumVoiceCategory kChannelCategory[NUM_CHANNELS] = {
	DRUM_CAT_KICK,
	DRUM_CAT_SNARE,
	DRUM_CAT_CLOSED_HAT,
	DRUM_CAT_OPEN_HAT,
	DRUM_CAT_OTHER,
	DRUM_CAT_OTHER,
};

/* Factory-default voice pick within each channel's category. */
static const DrumVoiceOps *const kMvpKit[NUM_CHANNELS] = {
	&drum_voice_plaits_kick,		/* DRUM_CAT_KICK */
	&drum_voice_plaits_snare,		/* DRUM_CAT_SNARE */
	&drum_voice_plaits_hihat,		/* DRUM_CAT_CLOSED_HAT */
	&drum_voice_plaits_hihat,		/* DRUM_CAT_OPEN_HAT -- same generator as closed, see the registry's own comment on this pairing */
	&drum_voice_mpump_rimshot,		/* DRUM_CAT_OTHER (channel E) */
	&drum_voice_mpump_cowbell,		/* DRUM_CAT_OTHER (channel F) */
};

/* Factory-default decay, per channel -- every other default is uniform
 * (see init_drum_ui() below) but a closed hat reading as "closed" and
 * an open hat reading as "open" is mostly about decay length, so this
 * one gets its own per-channel table rather than one shared constant. */
static const float kDefaultDecay[NUM_CHANNELS] = {
	0.5f,	/* Kick */
	0.5f,	/* Snare */
	0.15f,	/* Closed HH -- short/tight by default */
	0.5f,	/* Open HH -- unchanged */
	0.5f,	/* Other (E) */
	0.5f,	/* Other (F) */
};

/* Channel -> Grids part, a plain table rather than a per-voice lookup:
 * Grids authors only three parts, and both hi-hat channels read the
 * one hihat part (their densities stay independent, so open/closed
 * still thin out separately). Channels E and F (both DRUM_CAT_OTHER)
 * have no Grids data. */
static const int8_t kChanGridsPart[NUM_CHANNELS] = {
	0,	/* Kick  -> Grids kick */
	1,	/* Snare -> Grids snare */
	2,	/* Closed HH -> Grids hihat */
	2,	/* Open HH   -> Grids hihat, shared */
	-1,	/* Other (E) -> always euclidean */
	-1,	/* Other (F) -> always euclidean */
};

int8_t drum_chan_grids_part(uint8_t c)
{
	return (c < NUM_CHANNELS) ? kChanGridsPart[c] : -1;
}

/* True when this channel's pattern is currently coming from Grids. */
static uint8_t chan_is_grids_driven(uint8_t c)
{
	return (drum_pattern_engine == PATTERN_ENGINE_GRIDS) && (drum_chan_grids_part(c) >= 0);
}

#define DRUM_CV_TRIG_THRESHOLD	0.2f
#define DRUM_ACCENT_GAIN		1.0f	/* accented hit (Grids level > GRIDS_ACCENT_LEVEL, or Euclid's own downbeat): full loudness */
#define DRUM_UNACCENT_GAIN		0.6f	/* everything else real: pulled back rather than boosted, so accents can't newly clip */
#define DRUM_GHOST_GAIN			0.12f	/* extra hit ghost_amount inserts on an otherwise-silent step -- quieter than any real hit */
#define DRUM_DENSITY_DETENTS	32u		/* slider -> Grids density resolution, see read_channel_sliders() */
#define DRUM_GRIDS_XY_STEP		6		/* encoder clicks are coarse: ~42 turns spans the whole map */
#define DRUM_CHAOS_STEP			6		/* same coarseness as DRUM_GRIDS_XY_STEP, one shared 0..255 range */
#define DRUM_PARAM_STEP			0.02f
#define DRUM_PITCH_STEP			1.0f	/* one semitone per encoder click */
/* LED update runs at 60 Hz, so 4 ticks is a ~66 ms visible blip. */
#define DRUM_FLASH_TICKS		4

/* The read/write pair every DrumParamId funnels through, normalized to
 * 0..1 regardless of the target's own native range/type -- what lets
 * CV routing (read_cv_param_mod()) and automation (read_automation())
 * share one implementation instead of each needing its own per-
 * parameter special-casing. Chaos/density/speed each branch on
 * chan_is_grids_driven(c) the same way their own manual controls
 * already do (see read_voice_encoders()), so routing one of these on a
 * Grids-driven channel reaches whichever storage (shared or
 * per-channel) that channel's own knob would have reached too.
 * Rotation/steps always address this channel's own euclid state
 * regardless of engine -- harmless (if inaudible) to route on a
 * Grids-driven channel, same as turning their own knobs already is. */
static float drum_param_get01(uint8_t c, DrumParamId id)
{
	const o_drum_chan *d = &drum_chan[c];

	switch (id) {
		case DRUM_PARAM_FILTER:        return d->filter;
		case DRUM_PARAM_FILTER_RANDOM: return d->filter_random;
		case DRUM_PARAM_DECAY:         return d->decay;
		case DRUM_PARAM_DECAY_RANDOM:  return d->decay_random;
		case DRUM_PARAM_OTHER:         return d->other;
		case DRUM_PARAM_OTHER_RANDOM:  return d->other_random;
		case DRUM_PARAM_PITCH:         return (d->pitch + 24.0f) / 48.0f;
		case DRUM_PARAM_HUMANIZE:      return d->humanize;
		case DRUM_PARAM_GHOST:         return d->ghost_amount;
		case DRUM_PARAM_CHAOS:
			return (float)(chan_is_grids_driven(c) ? pattern_chaos : d->chaos_amount) / 255.0f;
		case DRUM_PARAM_SPEED: {
			float divmult_id = chan_is_grids_driven(c) ? grids_clock_divmult_id : d->clock_divmult_id;
			return (divmult_id - LFO_MIN_DIVMULT_ID) / (LFO_MAX_DIVMULT_ID - LFO_MIN_DIVMULT_ID);
		}
		case DRUM_PARAM_DENSITY:
			if (chan_is_grids_driven(c))
				return (float)d->density / 255.0f;
			return (d->euclid.n > 0) ? (float)d->euclid.k / (float)d->euclid.n : 0.0f;
		case DRUM_PARAM_ROTATION:
			return (d->euclid.n > 0) ? (float)d->euclid.rotation / (float)d->euclid.n : 0.0f;
		case DRUM_PARAM_STEPS:
			return (float)(d->euclid.n - 1) / (float)(EUCLID_MAX_STEPS - 1);
		default:
			return 0.0f;
	}
}

/* Sets the target directly from a 0..1 value -- an absolute
 * substitute, not a modulation added on top of the manual value, same
 * as CV_MODE_DENSITY's original (pre-generalization) behavior: while a
 * routed jack is patched, it fully owns this parameter every tick (see
 * read_cv_param_mod()), and the manual control for it does nothing
 * until the cable comes out. Chosen over an additive scheme because
 * most of these parameters (density, rotation, chaos, speed, steps...)
 * have no separate "live" value distinct from their own stored field
 * the way filter/decay/other's voice-coefficient push does -- writing
 * a modulated result back into that same field would either drift the
 * manual value permanently or need a second shadow copy of every
 * parameter, so one consistent rule (CV wins outright while patched)
 * beats a per-parameter special case. */
static void drum_param_set01(uint8_t c, DrumParamId id, float v01)
{
	o_drum_chan *d = &drum_chan[c];
	v01 = _CLAMP_F(v01, 0.0f, 1.0f);

	switch (id) {
		case DRUM_PARAM_FILTER:
			d->filter = v01;
			if (d->ops) d->ops->set_filter(d->state, d->filter);
			break;
		case DRUM_PARAM_FILTER_RANDOM:
			d->filter_random = v01;
			break;
		case DRUM_PARAM_DECAY:
			d->decay = v01;
			if (d->ops) d->ops->set_decay(d->state, d->decay);
			break;
		case DRUM_PARAM_DECAY_RANDOM:
			d->decay_random = v01;
			break;
		case DRUM_PARAM_OTHER:
			d->other = v01;
			if (d->ops) d->ops->set_other(d->state, d->other);
			break;
		case DRUM_PARAM_OTHER_RANDOM:
			d->other_random = v01;
			break;
		case DRUM_PARAM_PITCH:
			d->pitch = -24.0f + v01 * 48.0f;
			break;
		case DRUM_PARAM_HUMANIZE:
			d->humanize = v01;
			break;
		case DRUM_PARAM_GHOST:
			d->ghost_amount = v01;
			break;
		case DRUM_PARAM_CHAOS:
			if (chan_is_grids_driven(c))
				pattern_chaos = (uint8_t)(v01 * 255.0f);
			else
				d->chaos_amount = (uint8_t)(v01 * 255.0f);
			break;
		case DRUM_PARAM_SPEED: {
			float divmult_id = LFO_MIN_DIVMULT_ID + v01 * (LFO_MAX_DIVMULT_ID - LFO_MIN_DIVMULT_ID);
			if (chan_is_grids_driven(c)) {
				grids_clock_divmult_id = divmult_id;
				grids_clock_rate = calc_divmult_amount(divmult_id);
			} else {
				d->clock_divmult_id = divmult_id;
				d->clock_rate = calc_divmult_amount(divmult_id);
			}
			break;
		}
		case DRUM_PARAM_DENSITY:
			if (chan_is_grids_driven(c)) {
				d->density = (uint8_t)(v01 * 255.0f);
			} else {
				__disable_irq();
				euclid_set_k(&d->euclid, (int)(v01 * (float)d->euclid.n + 0.5f));
				__enable_irq();
			}
			break;
		case DRUM_PARAM_ROTATION:
			__disable_irq();
			euclid_set_rotation(&d->euclid, (int)(v01 * (float)d->euclid.n));
			__enable_irq();
			break;
		case DRUM_PARAM_STEPS:
			__disable_irq();
			euclid_set_n(&d->euclid, 1 + (int)(v01 * (float)(EUCLID_MAX_STEPS - 1) + 0.5f));
			__enable_irq();
			break;
		default:
			break;
	}
}

/* Steps `current` forward/back by `step` positions within just the
 * registry entries tagged `cat`, wrapping within that subset (not the
 * whole registry) -- this is what keeps a channel's voice cycling
 * "in-category" regardless of how far LFO shape gets turned. If
 * `current` isn't itself in `cat` (e.g. a channel loaded from a preset
 * saved before categories existed), treats it as if positioned just
 * before the first match. Registry is small (tens of entries) and this
 * only runs on an encoder tick, so a linear scan is fine. */
static const DrumVoiceOps *cycle_voice_in_category(DrumVoiceCategory cat, const DrumVoiceOps *current, int step)
{
	int count = 0;
	int cur_pos = -1;

	for (uint8_t i = 0; i < kNumDrumVoices; i++) {
		if (kDrumVoiceRegistry[i].category != cat)
			continue;
		if (kDrumVoiceRegistry[i].ops == current)
			cur_pos = count;
		count++;
	}
	if (count == 0)
		return NULL;

	int target_pos = ((cur_pos < 0 ? 0 : cur_pos) + step) % count;
	if (target_pos < 0)
		target_pos += count;

	int seen = 0;
	for (uint8_t i = 0; i < kNumDrumVoices; i++) {
		if (kDrumVoiceRegistry[i].category != cat)
			continue;
		if (seen == target_pos)
			return kDrumVoiceRegistry[i].ops;
		seen++;
	}
	return NULL; /* unreachable: target_pos < count, which is how many matches exist */
}

static void push_params(uint8_t chan)
{
	o_drum_chan *d = &drum_chan[chan];
	if (!d->ops) return;
	d->ops->set_filter(d->state, d->filter);
	d->ops->set_decay (d->state, d->decay);
	d->ops->set_other (d->state, d->other);
}

void init_drum_ui(void)
{
	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		o_drum_chan *d = &drum_chan[c];

		memset(d, 0, sizeof(*d));
		euclid_init(&d->euclid);

		/* Default n/rotation so the module makes a beat as soon as a
		 * clock arrives; k is overwritten on the very next
		 * read_channel_sliders() call to match wherever that
		 * channel's slider physically sits, so its initial value
		 * here is inconsequential. */
		euclid_set_n(&d->euclid, 16);
		euclid_set_k(&d->euclid, (c == 0) ? 4 : 2);
		euclid_set_rotation(&d->euclid, (c == 0) ? 0 : 4);

		/* Level is no longer slider-live (sliders now drive k), so it
		 * needs an audible default -- there's no other way to hear a
		 * channel until its button is held and browse is turned. */
		/* Same story as k: overwritten from the slider on the next
		 * read_channel_sliders() while Grids mode is active. */
		d->density = 128;
		d->accent_gain = 1.0f;
		d->humanize    = 0.0f;
		d->fire_delay  = 0;

		d->level  = 0.8f;
		d->pitch  = 0.0f;
		d->filter = 1.0f;
		d->decay  = kDefaultDecay[c];
		d->other  = 0.5f;

		d->clock_divmult_id = LFO_UNITY_DIVMULT_ID;
		d->clock_rate       = calc_divmult_amount(d->clock_divmult_id);
		d->step_phase       = 0.0f;

		/* Refuse to bind a voice whose state doesn't fit rather than
		 * silently corrupting the neighbouring channel's state. */
		if (kMvpKit[c] && kMvpKit[c]->state_size <= DRUM_VOICE_STATE_BYTES) {
			d->ops = kMvpKit[c];
			d->ops->init(d->state);
			push_params(c);
		}

		drum_trig_flash[c] = 0;
	}

	grids_init(&grids_state);
}

/* ── Control surface ─────────────────────────────────────────────────── */

/* Each channel's own slider sets its own pattern density (active-step
 * count k) directly -- no channel selection needed to sculpt a beat.
 * Actually moving a slider enough to change k also jumps edit focus to
 * that channel, so Depth/Latitude/Longitude/browse immediately act on
 * whichever voice you're touching. */
/* Rounds slider01 (0..1) to one of num_levels+1 discrete steps
 * (0..num_levels), but keeps returning `current` unless slider01 has
 * moved past its boundary by a bit of margin. Plain round-to-nearest
 * flickers between two neighbouring levels forever when the (already
 * IIR-filtered, but not perfectly still) ADC value happens to sit
 * right on a boundary -- and for k, every flicker was also re-grabbing
 * edit focus via drum_selected_chan, which is what "a channel gets
 * stuck as active" actually was: not one bad read, but a continuous
 * back-and-forth at rest. */
static int slider_to_level_hysteretic(float slider01, int current, int num_levels)
{
	if (num_levels <= 0)
		return 0;

	float step = 1.0f / (float)num_levels;
	float margin = step * 0.25f;
	float lower = ((float)current - 0.5f) * step - margin;
	float upper = ((float)current + 0.5f) * step + margin;

	if (slider01 >= lower && slider01 <= upper)
		return current;

	int target = (int)(slider01 * (float)num_levels + 0.5f);
	if (target < 0) target = 0;
	if (target > num_levels) target = num_levels;
	return target;
}

/* After a preset load, a slider sitting anywhere other than the
 * loaded k/density's own position would otherwise instantly overwrite
 * it on the very next call below -- this runs every main-loop tick and
 * has no way to tell "the user just moved this" apart from "we just
 * loaded a different value than wherever this physical slider happens
 * to sit", so the loaded pattern density snapped back to the slider
 * position within a few milliseconds of loading. This is almost
 * certainly why loading a preset looked like it did nothing.
 *
 * Fix: a "soft takeover" per channel, the standard answer to this on
 * any hardware controller with absolute (non-motorized) faders --
 * after a load, a channel's slider is ignored until the user actually
 * moves it back to (approximately) the loaded value, then normal
 * absolute control resumes. Set by drum_preset.c after a load. */
static uint8_t slider_pickup_pending[NUM_CHANNELS];

/* Same idea, for performance mode's slider->level takeover instead of
 * a preset load -- see its doc comment further down, next to
 * read_performance_controls(). Declared here, alongside
 * slider_pickup_pending, purely so drum_ui_slider_pickup_pending()
 * below can report either one through the one shared LED. */
static uint8_t level_pickup_pending[NUM_CHANNELS];

void drum_ui_request_slider_pickup(void)
{
	for (uint8_t c = 0; c < NUM_CHANNELS; c++)
		slider_pickup_pending[c] = 1;
}

/* For the slider LED: true while channel c's slider is being ignored
 * (see above), i.e. while its LED does *not* reflect the channel's
 * real k/density/level -- the physical slider hasn't been moved to
 * catch up with a just-loaded preset or a performance-mode switch yet.
 * The two pending flags are mutually exclusive in practice (edit vs.
 * performance mode), so reporting either one through the one LED is
 * unambiguous. */
uint8_t drum_ui_slider_pickup_pending(uint8_t chan)
{
	return (chan < NUM_CHANNELS) ? (slider_pickup_pending[chan] || level_pickup_pending[chan]) : 0;
}

static void read_channel_sliders(void)
{
	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		EuclidChannelState *e = &drum_chan[c].euclid;

		/* If this channel's CV is routed to density and the jack is
		 * patched, read_cv_param_mod() overwrites whatever gets set
		 * below again later this same tick (it runs after this in
		 * read_drum_ui()) -- so this always reads the physical slider
		 * unconditionally; CV, when routed and patched, simply wins
		 * every tick, same as any other routed target. */
		float slider01 = _CLAMP_F(analog[A_SLIDER + c].lpf_val / 4095.0f, 0.0f, 1.0f);

		/* Grids has no k -- the same slider becomes that part's
		 * density threshold instead. The two Other channels have no Grids data,
		 * so they keep driving k in either mode. */
		if (chan_is_grids_driven(c)) {
			/* Quantized to DRUM_DENSITY_DETENTS steps rather than the
			 * full 0..255, both for a coarser/more usable feel and so
			 * the hysteresis margin below means something (a margin on
			 * a 256-level range would be sub-single-bit). */
			uint8_t current_detent = (uint8_t)(((uint16_t)drum_chan[c].density * DRUM_DENSITY_DETENTS + 127u) / 255u);

			if (slider_pickup_pending[c]) {
				int raw_detent = (int)(slider01 * (float)DRUM_DENSITY_DETENTS + 0.5f);
				if (raw_detent == (int)current_detent)
					slider_pickup_pending[c] = 0;
				else
					continue;
			}

			int target_detent = slider_to_level_hysteretic(slider01, current_detent, DRUM_DENSITY_DETENTS);
			uint8_t target_density = (uint8_t)(((uint32_t)target_detent * 255u) / DRUM_DENSITY_DETENTS);
			if (target_density != drum_chan[c].density) {
				drum_chan[c].density = target_density;
				drum_selected_chan = c;
			}
			continue;
		}

		/* Squared rather than linear: low k values (near the bottom of
		 * the slider's travel) then span more of the slider's physical
		 * range, giving finer control over sparse patterns, at the
		 * cost of coarser control approaching a fully-active one --
		 * the more usable tradeoff for a density control, where "how
		 * sparse" matters more than "how dense". */
		float k_slider01 = slider01 * slider01;

		if (slider_pickup_pending[c]) {
			int raw_k = (int)(k_slider01 * (float)e->n + 0.5f);
			if (raw_k == e->k)
				slider_pickup_pending[c] = 0;
			else
				continue;
		}

		int target_k = slider_to_level_hysteretic(k_slider01, e->k, e->n);

		if (target_k != e->k) {
			__disable_irq();
			euclid_set_k(e, target_k);
			__enable_irq();
			drum_selected_chan = c;
		}
	}
}

/* Select/global-edit-toggle now lands on release rather than press, and
 * only for a genuine short tap that never routed a CV target (see
 * assign_cv_target_if_held()) -- holding a channel to route CV, or just
 * holding it a while for no reason, must not also reselect it or flip
 * global edit mode out from under whatever the hold was actually for.
 * "Short" reuses the existing button_pressed() press-length ladder
 * (RELEASED < PRESSED < SHORT_PRESSED < ...): the decision is made on
 * the release edge by looking at whichever level the press had reached
 * right before it, same idiom as read_drum_preset_ui()'s load/save
 * decision -- a press released before it ever reaches SHORT_PRESSED
 * (500ms) counts as short here. */
static void read_channel_buttons(void)
{
	static enum PressTypes prev_press_level[NUM_CHANNELS];
	int8_t held = -1;

	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		enum PressTypes level = button_pressed(c);

		if (level != RELEASED && prev_press_level[c] == RELEASED) {
			held_chan_cv_assigned = 0;
			held_chan_cv_cleared = 0;
			held_chan_press_start_ms = HAL_GetTick() / TICKS_PER_MS;
		}

		if (level == RELEASED && prev_press_level[c] != RELEASED
		    && !held_chan_cv_assigned && prev_press_level[c] < SHORT_PRESSED) {
			/* Re-pressing the channel that's already selected toggles
			 * global edit mode instead of just reselecting it (a no-op
			 * otherwise); pressing a genuinely different channel always
			 * drops back to normal per-channel editing. */
			if (c == drum_selected_chan)
				drum_global_edit_mode = !drum_global_edit_mode;
			else {
				drum_selected_chan = c;
				drum_global_edit_mode = 0;
			}
		}

		if (level != RELEASED)
			held = (int8_t)c;
		prev_press_level[c] = level;
	}
	drum_held_chan = held;

	/* DRUM_CV_CLEAR_HOLD_MS held with no knob touched (held_chan_cv_assigned
	 * still clear) clears this channel's CV mode back to trigger -- see
	 * that constant's doc comment. Checked live, not on release, so it
	 * reads as an immediate "held long enough" moment rather than
	 * something that only shows up once you let go. */
	if (held >= 0 && !held_chan_cv_assigned && !held_chan_cv_cleared) {
		uint32_t now_ms = HAL_GetTick() / TICKS_PER_MS;
		if (now_ms - held_chan_press_start_ms >= DRUM_CV_CLEAR_HOLD_MS) {
			drum_chan[held].cv_mode = DRUM_CV_TARGET_TRIGGER;
			held_chan_cv_cleared = 1;
			start_ongoing_display_drum_cv_mode();
		}
	}
}

/* Performance mode (VOCTSW -- see drum_ui_performance_mode()'s doc
 * comment): sliders drive level directly (no detents/hysteresis
 * needed, unlike k/density -- level is already a plain continuous
 * knob) and channel buttons mute instead of selecting. A plain press
 * toggles immediately; FINE+press instead queues the toggle for the
 * next bar boundary (mute_toggle_pending, applied in
 * update_drum_triggers()), so a change can be cued up without landing
 * off-beat. Entirely separate from read_channel_buttons()/
 * read_channel_sliders() above -- read_drum_ui() calls one set or the
 * other, never both. */
/* level_pickup_pending is declared next to slider_pickup_pending above.
 * Armed on the edit-to-performance transition (see read_drum_ui()),
 * cleared per channel once its slider is moved back within
 * DRUM_LEVEL_PICKUP_MARGIN of the level it already had -- entering
 * performance mode otherwise has each slider instantly snap that
 * channel's level to wherever it physically happens to be sitting,
 * almost never where it was already set to. */
#define DRUM_LEVEL_PICKUP_MARGIN	0.03f

static void read_performance_controls(void)
{
	static uint8_t prev_pressed[NUM_CHANNELS];
	uint8_t fine_held = switch_pressed(FINE_BUTTON);

	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		float slider01 = _CLAMP_F(analog[A_SLIDER + c].lpf_val / 4095.0f, 0.0f, 1.0f);

		if (level_pickup_pending[c]) {
			float diff = slider01 - drum_chan[c].level;
			if (diff < 0.0f)
				diff = -diff;
			if (diff <= DRUM_LEVEL_PICKUP_MARGIN)
				level_pickup_pending[c] = 0;
		}
		if (!level_pickup_pending[c])
			drum_chan[c].level = slider01;

		uint8_t now = (button_pressed(c) != RELEASED);
		if (now && !prev_pressed[c]) {
			if (fine_held)
				mute_toggle_pending[c] = 1;
			else
				drum_chan[c].muted = !drum_chan[c].muted;
		}
		prev_pressed[c] = now;
	}
}

/* Manually touching a knob always wins over a looping automation
 * playback on that same channel -- same "the real control wins"
 * precedent as the slider-pickup fix and CV-density mode -- but only
 * when it's the knob automation is actually driving (automation_target,
 * independent of cv_mode -- see that field's doc comment): touching an
 * unrelated parameter shouldn't stop a loop that has nothing to do
 * with it. Doesn't apply while RECORD is armed (FINE held): that's the
 * manual knob turning itself being captured, not something to cancel. */
static void cancel_automation_if_target(uint8_t c, DrumParamId id)
{
	o_drum_chan *dc = &drum_chan[c];
	if (dc->automation_state == AUTOMATION_PLAY && dc->automation_target == id)
		dc->automation_state = AUTOMATION_OFF;
}

/* Applies `delta` to *one* channel's filter, re-pushing to its voice.
 * Broken out so the DEPTH handler below can hit either just the
 * selected channel or (in global edit mode) all six identically. */
static void apply_filter_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_target(c, DRUM_PARAM_FILTER);
	dc->filter = _CLAMP_F(dc->filter + delta, 0.0f, 1.0f);
	if (dc->ops) dc->ops->set_filter(dc->state, dc->filter);
}

static void apply_decay_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_target(c, DRUM_PARAM_DECAY);
	dc->decay = _CLAMP_F(dc->decay + delta, 0.0f, 1.0f);
	if (dc->ops) dc->ops->set_decay(dc->state, dc->decay);
}

static void apply_other_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_target(c, DRUM_PARAM_OTHER);
	dc->other = _CLAMP_F(dc->other + delta, 0.0f, 1.0f);
	if (dc->ops) dc->ops->set_other(dc->state, dc->other);
}

static void apply_filter_random_delta(uint8_t c, float delta)
{
	cancel_automation_if_target(c, DRUM_PARAM_FILTER_RANDOM);
	drum_chan[c].filter_random = _CLAMP_F(drum_chan[c].filter_random + delta, 0.0f, 1.0f);
}

static void apply_decay_random_delta(uint8_t c, float delta)
{
	cancel_automation_if_target(c, DRUM_PARAM_DECAY_RANDOM);
	drum_chan[c].decay_random = _CLAMP_F(drum_chan[c].decay_random + delta, 0.0f, 1.0f);
}

static void apply_other_random_delta(uint8_t c, float delta)
{
	cancel_automation_if_target(c, DRUM_PARAM_OTHER_RANDOM);
	drum_chan[c].other_random = _CLAMP_F(drum_chan[c].other_random + delta, 0.0f, 1.0f);
}

static void apply_pitch_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_target(c, DRUM_PARAM_PITCH);
	dc->pitch = _CLAMP_F(dc->pitch + delta, -24.0f, 24.0f);
}

static void apply_clock_rate_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_target(c, DRUM_PARAM_SPEED);
	dc->clock_divmult_id = _CLAMP_F(dc->clock_divmult_id + delta, LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID);
	dc->clock_rate = calc_divmult_amount(dc->clock_divmult_id);
}

static void apply_humanize_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_target(c, DRUM_PARAM_HUMANIZE);
	dc->humanize = _CLAMP_F(dc->humanize + delta, 0.0f, 1.0f);
}

static void apply_ghost_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_target(c, DRUM_PARAM_GHOST);
	dc->ghost_amount = _CLAMP_F(dc->ghost_amount + delta, 0.0f, 1.0f);
}

static void apply_chaos_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_target(c, DRUM_PARAM_CHAOS);
	dc->chaos_amount = (uint8_t)_CLAMP_I32((int32_t)dc->chaos_amount + (int32_t)delta, 0, 255);
}

/* Active step count / density, the same value read_channel_sliders()
 * drives -- see this function's own doc comment there for why Grids-
 * driven channels use a quantized density threshold while everyone
 * else (Euclid-mode selected channel, or channels E/F regardless of
 * engine -- see chan_is_grids_driven()) uses k. `delta` is encoder
 * clicks, not a 0..1 fraction like the other apply_*_delta functions
 * here -- one click moves one Grids detent or one Euclid k step.
 *
 * Arms this channel's own slider_pickup_pending the same way a preset
 * load does (see that flag's doc comment above) -- without it,
 * read_channel_sliders() runs again on the very next tick, sees the
 * physical slider sitting away from the value the encoder just set,
 * and immediately snaps it right back to wherever the slider happens
 * to be resting. */
static void apply_density_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_target(c, DRUM_PARAM_DENSITY);
	if (chan_is_grids_driven(c)) {
		int32_t step = (int32_t)(delta * (255.0f / DRUM_DENSITY_DETENTS));
		dc->density = (uint8_t)_CLAMP_I32((int32_t)dc->density + step, 0, 255);
	} else {
		__disable_irq();
		euclid_set_k(&dc->euclid, dc->euclid.k + (int)delta);
		__enable_irq();
	}
	slider_pickup_pending[c] = 1;
}

/* Runs `apply` on every channel if global edit mode is active, else on
 * just the selected one -- the one piece of logic every knob handler
 * below shares. */
static void apply_to_selected_or_all(void (*apply)(uint8_t, float), float delta)
{
	if (drum_global_edit_mode) {
		for (uint8_t c = 0; c < NUM_CHANNELS; c++)
			apply(c, delta);
	} else {
		apply(drum_selected_chan, delta);
	}
}

/* While a channel button is held, turning *any* per-channel knob
 * routes that held channel's CV jack to that parameter *instead of*
 * editing the knob's value -- the target is chosen by touching the
 * control you want it to affect, and the hold is a dedicated "pick a
 * target" gesture, not also a live edit (held channel and selected
 * channel can be two different channels, and it would be surprising
 * for turning a knob to route one channel's CV to also change another
 * channel's sound). The only way back to DRUM_CV_TARGET_TRIGGER is
 * holding the channel for DRUM_CV_CLEAR_HOLD_MS without touching a
 * knob at all -- see read_channel_buttons(). */
static void assign_cv_target_if_held(DrumParamId id)
{
	drum_chan[drum_held_chan].cv_mode = 1 + (uint8_t)id;
	held_chan_cv_assigned = 1;
	start_ongoing_display_drum_cv_mode();
}

/* Same idea as assign_cv_target_if_held() above, but for
 * automation_target instead of cv_mode -- automation's own,
 * independent target (see that field's doc comment), routed with FINE
 * also held so the same "hold channel + turn a knob" gesture can pick
 * either one without a second physical control: FINE held routes
 * automation, released routes CV -- see route_cv_or_edit(). No
 * "hold-to-clear" here since automation_target has no special
 * "nothing routed" state the way cv_mode's trigger default is --
 * it's just always some DrumParamId, defaulting to filter. */
static void assign_automation_target_if_held(DrumParamId id)
{
	drum_chan[drum_held_chan].automation_target = (uint8_t)id;
	held_chan_cv_assigned = 1;
	start_ongoing_display_drum_automation_target();
}

/* The one piece of logic every per-channel knob handler below shares:
 * while a channel is held, route its CV target (or, with FINE also
 * held, its automation target instead) to `id` and report "I handled
 * this, don't also edit the value" (1); otherwise report "go ahead and
 * edit normally" (0). See assign_cv_target_if_held()/
 * assign_automation_target_if_held() above. */
static uint8_t route_cv_or_edit(DrumParamId id)
{
	if (drum_held_chan < 0)
		return 0;
	if (switch_pressed(FINE_BUTTON))
		assign_automation_target_if_held(id);
	else
		assign_cv_target_if_held(id);
	return 1;
}

static void read_voice_encoders(void)
{
	o_drum_chan *d = &drum_chan[drum_selected_chan];
	int16_t enc;

	enc = pop_encoder_q(pec_DEPTH);
	if (enc && !route_cv_or_edit(DRUM_PARAM_FILTER)) {
		apply_to_selected_or_all(apply_filter_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_FILTER);
	}

	/* Push+turn on DEPTH (sec_DISPERSION, dead in the old wavetable UI)
	 * -- how much random bipolar offset gets added to filter on each
	 * hit (see apply_random_offsets() in drum_render_channel()). */
	enc = pop_encoder_q(sec_DISPERSION);
	if (enc && !route_cv_or_edit(DRUM_PARAM_FILTER_RANDOM)) {
		apply_to_selected_or_all(apply_filter_random_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_FILTER_RANDOM);
	}

	enc = pop_encoder_q(pec_LATITUDE);
	if (enc && !route_cv_or_edit(DRUM_PARAM_DECAY)) {
		apply_to_selected_or_all(apply_decay_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_DECAY);
	}

	/* Push+turn on LATITUDE (sec_DISPPATT) -- same idea, for decay.
	 * Used to be ghost-note amount; that moved to OCT push+turn below
	 * to make room for all three params to get this symmetrically. */
	enc = pop_encoder_q(sec_DISPPATT);
	if (enc && !route_cv_or_edit(DRUM_PARAM_DECAY_RANDOM)) {
		apply_to_selected_or_all(apply_decay_random_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_DECAY_RANDOM);
	}

	enc = pop_encoder_q(pec_LONGITUDE);
	if (enc && !route_cv_or_edit(DRUM_PARAM_OTHER)) {
		apply_to_selected_or_all(apply_other_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_OTHER);
	}

	/* Push+turn on LONGITUDE (sec_WTSEL_SPREAD, dead in the old
	 * wavetable UI) -- same idea, for other. */
	enc = pop_encoder_q(sec_WTSEL_SPREAD);
	if (enc && !route_cv_or_edit(DRUM_PARAM_OTHER_RANDOM)) {
		apply_to_selected_or_all(apply_other_random_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_OTHER_RANDOM);
	}

	enc = pop_encoder_q(pec_TRANSPOSE);
	if (enc && !route_cv_or_edit(DRUM_PARAM_PITCH))
		apply_to_selected_or_all(apply_pitch_delta, (float)enc * DRUM_PITCH_STEP);

	/* Push+turn on the same encoder (sec_OSC_SPREAD, "spread" in the
	 * old wavetable UI this panel used to drive) -- humanize amount:
	 * see schedule_pattern_hit() in update_drum_triggers() for what it
	 * actually does to a hit. Global-edit-mode-aware like every other
	 * knob here. */
	enc = pop_encoder_q(sec_OSC_SPREAD);
	if (enc && !route_cv_or_edit(DRUM_PARAM_HUMANIZE)) {
		apply_to_selected_or_all(apply_humanize_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_HUMANIZE);
	}

	/* LFOSPEED: active step count / density -- the same value the
	 * channel slider drives (see read_channel_sliders()' doc comment),
	 * just reachable from an encoder too. apply_density_delta() picks
	 * Grids density vs Euclid k per channel on its own (see
	 * chan_is_grids_driven()), so this is safe to broadcast in global
	 * edit mode even with a mix of Grids- and Euclid-driven channels
	 * selected. */
	enc = pop_encoder_q(pec_LFOSPEED);
	if (enc && !route_cv_or_edit(DRUM_PARAM_DENSITY))
		apply_to_selected_or_all(apply_density_delta, (float)enc);

	/* Push+turn on LFOSPEED (sec_LFOGAIN, dead until now) -- total step
	 * count, always for the selected channel only (not global-edit-mode
	 * aware -- a step count only means something relative to that one
	 * channel's own pattern). No effect on a Grids-driven channel,
	 * which has no step count of its own (see chan_is_grids_driven()). */
	enc = pop_encoder_q(sec_LFOGAIN);
	if (enc && !route_cv_or_edit(DRUM_PARAM_STEPS) && !chan_is_grids_driven(drum_selected_chan)) {
		EuclidChannelState *e = &d->euclid;
		cancel_automation_if_target(drum_selected_chan, DRUM_PARAM_STEPS);
		__disable_irq();
		euclid_set_n(e, e->n + enc);
		__enable_irq();
	}

	/* LFOSHAPE: pattern rotation, always for the selected channel only
	 * -- same convention and same Grids-driven exclusion as step count
	 * above (this used to be the WBROWSE plain turn's job; see
	 * read_pattern_encoder()). */
	enc = pop_encoder_q(pec_LFOSHAPE);
	if (enc && !route_cv_or_edit(DRUM_PARAM_ROTATION) && !chan_is_grids_driven(drum_selected_chan)) {
		EuclidChannelState *e = &d->euclid;
		/* euclid_rotate() advances a step's *source* index with
		 * +rotation, which moves onsets to earlier step indices
		 * (anticipates) as rotation increases -- backwards from
		 * the expected "turn = later/lag" feel, hence the sign
		 * flip here rather than in the (tested) pattern engine. */
		cancel_automation_if_target(drum_selected_chan, DRUM_PARAM_ROTATION);
		__disable_irq();
		euclid_set_rotation(e, e->rotation - enc);
		__enable_irq();
	}

	/* Push+turn on LFOSHAPE (sec_LFOPHASE) -- clock divide/multiply,
	 * the job pec_LFOSPEED's plain turn used to do before LFOSPEED took
	 * over density above. FINE used to fine-scale this; FINE is now
	 * dedicated entirely to automation record/play (see
	 * read_automation()), so this always uses its one coarse step now.
	 * In Grids mode this is one shared rate for the whole stepper --
	 * unconditional on selected channel, same as x/y -- since all four
	 * Grids-driven channels read the one grids_state.step; in Euclid
	 * mode it stays per-channel (global-edit-mode-aware). Channels E/F
	 * have no Grids data (see kChanGridsPart) and always run their own
	 * independent Euclidean pattern -- including their own clock_rate --
	 * even while the kit is in Grids mode, so selecting one of them
	 * keeps editing its own per-channel rate instead of the shared one. */
	enc = pop_encoder_q(sec_LFOPHASE);
	if (enc && !route_cv_or_edit(DRUM_PARAM_SPEED)) {
		if (drum_pattern_engine == PATTERN_ENGINE_GRIDS && chan_is_grids_driven(drum_selected_chan)) {
			grids_clock_divmult_id = _CLAMP_F(grids_clock_divmult_id + (float)enc, LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID);
			grids_clock_rate = calc_divmult_amount(grids_clock_divmult_id);
		} else {
			apply_to_selected_or_all(apply_clock_rate_delta, (float)enc);
		}
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_SPEED);
	}

	/* Plain turn on OCT (fully dead otherwise) -- ghost-note amount,
	 * same shape as humanize above. Paired with chaos on the same
	 * physical encoder since both are "pattern variation" controls. */
	enc = pop_encoder_q(pec_OCT);
	if (enc && !route_cv_or_edit(DRUM_PARAM_GHOST)) {
		apply_to_selected_or_all(apply_ghost_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_GHOST);
	}

	/* Push+turn on OCT (sec_SCALE, dead in the old wavetable UI) --
	 * chaos amount. Grids mode edits the one shared pattern_chaos (its
	 * parts have no other per-channel identity); Euclid mode edits the
	 * selected channel's own chaos_amount instead, same selected-vs-
	 * global convention as every other per-channel knob here, since
	 * each Euclid channel already has its own independent pattern. */
	enc = pop_encoder_q(sec_SCALE);
	if (enc && !route_cv_or_edit(DRUM_PARAM_CHAOS)) {
		if (drum_pattern_engine == PATTERN_ENGINE_GRIDS) {
			pattern_chaos = (uint8_t)_CLAMP_I32((int32_t)pattern_chaos + enc * DRUM_CHAOS_STEP, 0, 255);
		} else {
			apply_to_selected_or_all(apply_chaos_delta, (float)(enc * DRUM_CHAOS_STEP));
		}
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_CHAOS);
	}

}

/* butm_LFOMODE_BUTTON: cycles the selected channel's voice within its
 * own category (see kChannelCategory) -- the job pec_LFOSHAPE's plain
 * turn used to do before LFOSHAPE took over rotation above. In the old
 * wavetable UI this button toggled each channel's LFO/LPG mode, hence
 * its name; fully free on the drum station until now. Edge-triggered,
 * one step forward per press. Deliberately ignores global edit mode --
 * "same position in each channel's own category" isn't a coherent
 * action across channels with different categories, unlike a plain
 * knob delta. Re-init rather than carry over DSP state across a voice
 * swap -- the old voice's envelope/oscillator phase means nothing to
 * the new one -- then re-push filter/decay/other so the knobs don't
 * silently reset to that voice's own defaults. */
static void read_voice_select_button(void)
{
	static uint8_t prev_pressed = 0;
	uint8_t now = (button_pressed(butm_LFOMODE_BUTTON) != RELEASED);

	if (now && !prev_pressed) {
		o_drum_chan *d = &drum_chan[drum_selected_chan];
		DrumVoiceCategory cat = kChannelCategory[drum_selected_chan];
		const DrumVoiceOps *new_ops = cycle_voice_in_category(cat, d->ops, 1);

		if (new_ops && new_ops->state_size <= DRUM_VOICE_STATE_BYTES) {
			d->ops = new_ops;
			d->ops->init(d->state);
			push_params(drum_selected_chan);
		}
		start_ongoing_display_drum_voice();
	}
	prev_pressed = now;
}

/* Grids' shared X/Y map position -- the one job left on this encoder
 * now that rotation and step count moved to LFOSHAPE/LFOSPEED+push
 * (see read_voice_encoders()), and the held-channel-nudges-level
 * override that used to live here is gone entirely (holding a channel
 * now routes CV targets instead -- see assign_cv_target_if_held()).
 * X/Y have no per-channel identity (all Grids-driven channels share
 * one map position), so this is unconditional on the selected channel,
 * unlike everything that moved off it. Does nothing outside Grids
 * mode -- currently unclaimed there. */
static void read_pattern_encoder(void)
{
	if (drum_pattern_engine != PATTERN_ENGINE_GRIDS)
		return;

	int16_t enc = pop_encoder_q(pec_WBROWSE);
	int16_t enc2 = pop_encoder_q(sec_WTSEL);

	if (enc) {
		grids_x = (uint8_t)_CLAMP_I32((int32_t)grids_x + enc * DRUM_GRIDS_XY_STEP, 0, 255);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_GRIDS_X);
	}
	if (enc2) {
		grids_y = (uint8_t)_CLAMP_I32((int32_t)grids_y + enc2 * DRUM_GRIDS_XY_STEP, 0, 255);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_GRIDS_Y);
	}
}

/* butm_LFOVCA_BUTTON is dead on the drum station (the keys-mode combo it
 * used to belong to has no call site left), so it toggles the kit-wide
 * pattern algorithm. Edge-triggered, not level: holding it must not
 * flip back and forth. */
/* Set here (main loop), consumed and cleared in update_drum_triggers()
 * (OSC_TIM) -- same producer/consumer split as trigger_pending/
 * choke_pending, since the reset it triggers touches current_step/
 * step_phase/grids_state.step, which only OSC_TIM is otherwise allowed
 * to write. */
static volatile uint8_t pattern_resync_pending = 0;

static void read_pattern_engine_button(void)
{
	static uint8_t prev_pressed = 0;
	uint8_t now = (button_pressed(butm_LFOVCA_BUTTON) != RELEASED);

	if (now && !prev_pressed) {
		drum_pattern_engine = (drum_pattern_engine == PATTERN_ENGINE_EUCLID)
		                    ? PATTERN_ENGINE_GRIDS : PATTERN_ENGINE_EUCLID;
		start_ongoing_display_drum_engine();
		/* Whichever engine is now active resumes its channels from
		 * wherever they last were, which can be anywhere -- while
		 * they were inactive, the master clock (and the two Other channels,
		 * always euclidean regardless of mode) kept moving without
		 * them. Force everything back to a synced downbeat rather
		 * than let the just-switched-to channels start out of phase
		 * with the ones that never stopped. */
		pattern_resync_pending = 1;
	}
	prev_pressed = now;
}

/* Drives whichever parameter this channel's CV is routed to (anything
 * but DRUM_CV_TARGET_TRIGGER -- see o_drum_chan.cv_mode's doc comment)
 * straight from the jack, every tick, via drum_param_set01() -- this
 * has to run continuously rather than only on a change, since the CV
 * itself can be moving every tick. Only while actually patched: an
 * unplugged jack leaves the target exactly where manual control (or a
 * previous CV reading) last set it, rather than pulling it to some
 * default. Runs after every manual control above in read_drum_ui(), so
 * a routed+patched target always wins the tick over its own knob --
 * same "the cable's job, not the knob's, while it's plugged in"
 * behavior CV_MODE_DENSITY originally established, just generalized to
 * every parameter instead of one. */
static void read_cv_param_mod(void)
{
	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		o_drum_chan *d = &drum_chan[c];
		if (d->cv_mode == DRUM_CV_TARGET_TRIGGER || !analog_jack_plugged(A_VOCT + c))
			continue;

		float cv01 = _CLAMP_F(analog[A_VOCT + c].lpf_val / 4095.0f, 0.0f, 1.0f);
		drum_param_set01(c, (DrumParamId)(d->cv_mode - 1), cv01);
	}
}

/* Bar-tick and the master clock's fractional progress through the
 * current tick, both written once per OSC_TIM tick in
 * update_drum_triggers() below and read (loosely -- a stale-by-one-tick
 * read is imperceptible here) by read_automation() on the main loop to
 * index/interpolate the automation lanes. */
static volatile uint16_t g_bar_tick  = 0;
static volatile float    g_clk_frac  = 0.0f;

/* FINE dedicated entirely to automation record/play for this channel's
 * own automation_target (independent of cv_mode -- see that field's
 * doc comment), one lane per channel, one bar long
 * (DRUM_BAR_TICKS points, linearly interpolated between them, stored
 * normalized 0..1 via drum_param_get01()/drum_param_set01() regardless
 * of the target's own native range). Holding FINE arms/continues
 * RECORD on the selected channel (or every channel, in global edit
 * mode) -- sampling the live target value once per bar_tick, wrapping
 * and overwriting continuously, so a hold of any length just keeps the
 * most recent lap. Releasing commits and starts PLAY, looping forever
 * until a manual edit of that same target cancels it (see
 * cancel_automation_if_target()) or FINE is held again for a fresh
 * take. Playback always runs for every channel regardless of
 * selection, so switching which channel is selected mid-loop doesn't
 * stall one that's already playing. Retargeting a channel (holding it
 * and touching a different control) takes effect on the *next* RECORD
 * -- an in-progress PLAY keeps looping whatever it already captured
 * under the old target until re-recorded, rather than reinterpreting
 * old lane data against a new parameter. */
/* Guards against exactly the bug that caused a boot crash: a
 * momentary power-on glitch on FINE's GPIO read as a "press" for a
 * tick or two arms RECORD, and the instant it clears (still within
 * the same boot sequence, no user anywhere near the panel) the old
 * code committed straight to PLAY -- which then pushed a bogus
 * (all-zero, since the lanes are freshly memset) value into that
 * channel's voice on every single tick forever. Requiring a real,
 * sustained hold before a take counts as valid closes that off, and
 * is also just better behavior on its own terms: a "recording" a
 * few milliseconds long was never a usable loop anyway. */
#define AUTOMATION_MIN_RECORD_MS	200u

static void read_automation(void)
{
	static uint32_t record_start_ms[NUM_CHANNELS];
	static DrumParamId play_target[NUM_CHANNELS];

	uint8_t fine_held = switch_pressed(FINE_BUTTON);
	uint32_t now_ms = HAL_GetTick() / TICKS_PER_MS;
	uint16_t tick = g_bar_tick;
	uint16_t next = (tick + 1) % DRUM_BAR_TICKS;
	float frac = g_clk_frac;

	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		o_drum_chan *d = &drum_chan[c];
		uint8_t recording = fine_held && (drum_global_edit_mode || c == drum_selected_chan);

		if (recording) {
			if (d->automation_state != AUTOMATION_RECORD)
				record_start_ms[c] = now_ms;
			d->automation_state = AUTOMATION_RECORD;
			d->automation_lane[tick] = drum_param_get01(c, (DrumParamId)d->automation_target);
			continue;
		}

		if (d->automation_state == AUTOMATION_RECORD) {
			/* Commits this take's target now, once, rather than
			 * re-reading automation_target every PLAY tick below -- see
			 * this function's own doc comment on retargeting. */
			play_target[c] = (DrumParamId)d->automation_target;
			d->automation_state = ((now_ms - record_start_ms[c]) >= AUTOMATION_MIN_RECORD_MS)
			                     ? AUTOMATION_PLAY : AUTOMATION_OFF;
		}

		if (d->automation_state != AUTOMATION_PLAY)
			continue;

		float v = d->automation_lane[tick] + (d->automation_lane[next] - d->automation_lane[tick]) * frac;
		drum_param_set01(c, play_target[c], v);
	}
}

void read_drum_ui(void)
{
	/* Performance mode takes the sliders and buttons over entirely and
	 * locks out everything else -- no density editing, voice browsing,
	 * pattern-engine toggle, CV mode, or automation reachable until
	 * VOCTSW flips back. Whichever pattern was already programmed in
	 * edit mode just keeps playing underneath (update_drum_triggers()
	 * isn't touched by this at all); this is a live mixing overlay, not
	 * a pause. */
	static uint8_t was_performance_mode = 0;
	uint8_t now_performance_mode = drum_ui_performance_mode();

	if (now_performance_mode) {
		/* Just switched in from edit mode -- arm soft pickup on every
		 * slider (see level_pickup_pending's doc comment) rather than
		 * letting each one instantly snap the channel's level to
		 * wherever it physically happens to be sitting. */
		if (!was_performance_mode) {
			for (uint8_t c = 0; c < NUM_CHANNELS; c++)
				level_pickup_pending[c] = 1;
		}
		was_performance_mode = 1;
		read_performance_controls();
		return;
	}
	if (was_performance_mode) {
		/* Just switched back from performance mode -- arm the same
		 * soft pickup in the other direction, on k/density this
		 * time, so the pattern doesn't snap to wherever the slider
		 * happens to be sitting after moving levels around. */
		for (uint8_t c = 0; c < NUM_CHANNELS; c++)
			slider_pickup_pending[c] = 1;
	}
	was_performance_mode = 0;

	read_channel_sliders();
	read_channel_buttons();
	read_voice_encoders();
	read_pattern_encoder();
	read_pattern_engine_button();
	read_voice_select_button();
	read_cv_param_mod();
	read_automation();
}

/* ── Clock / trigger ─────────────────────────────────────────────────── */

static void fire(uint8_t chan)
{
	drum_chan[chan].trigger_pending = 1;
	drum_trig_flash[chan] = DRUM_FLASH_TICKS;

	/* Performance-mode mute (see drum_ui_performance_mode()) silences
	 * this channel's audio via the level ramp in oscillator.c, but that
	 * doesn't touch the ENV OUT gate jack at all -- without this check
	 * a muted channel would still pulse CV out on every hit, which
	 * defeats the point of muting it for anything patched from that
	 * jack downstream. */
	if (!drum_chan[chan].muted)
		drum_gate_ticks[chan] = DRUM_GATE_TICKS;

	/* Hi-hat choke group: the closed-hat channel firing always cuts the
	 * open-hat channel dead, same as a real cymbal being one physical
	 * object. Channel roles are fixed by category (see kMvpKit), so
	 * this is just the two category indices, not a special case per
	 * voice. */
	if (chan == DRUM_CAT_CLOSED_HAT)
		drum_chan[DRUM_CAT_OPEN_HAT].choke_pending = 1;
}

#define DRUM_HUMANIZE_MAX_DELAY_TICKS	20u		/* ~11ms at the 1.8kHz OSC_TIM rate; late-only, see schedule_pattern_hit() */
#define DRUM_HUMANIZE_VELOCITY_RANGE	0.6f	/* +/- this fraction of gain at humanize=1.0 */

static uint32_t humanize_rng = 0x9E3779B9u;	/* arbitrary nonzero xorshift seed */

static float humanize_rand01(void)
{
	humanize_rng ^= humanize_rng << 13;
	humanize_rng ^= humanize_rng >> 17;
	humanize_rng ^= humanize_rng << 5;
	return (float)(humanize_rng >> 8) / (float)0x00FFFFFFu;
}

/* How much of a channel's ghost_pattern turns over each time it's
 * mutated -- see mutate_ghost_pattern() below. */
#define DRUM_GHOST_MUTATE_CHANCE	0.15f

/* Evolves a channel's existing ghost_pattern for its next bar (Euclid)
 * or lap (Grids) rather than replacing it outright: each of the low
 * `num_bits` bits has only a DRUM_GHOST_MUTATE_CHANCE chance of being
 * touched at all, and only a touched bit gets re-rolled (set with
 * probability `ghost_amount`, clear otherwise) -- the rest carry over
 * unchanged. A full reroll every bar is indistinguishable from
 * randomizing every hit fresh each time; this instead reads as one
 * recognizable pattern that slowly drifts, a few steps at a time,
 * bar over bar. */
static uint32_t mutate_ghost_pattern(uint32_t pattern, float ghost_amount, uint8_t num_bits)
{
	for (uint8_t i = 0; i < num_bits; i++) {
		if (humanize_rand01() >= DRUM_GHOST_MUTATE_CHANCE)
			continue;
		if (humanize_rand01() < ghost_amount)
			pattern |= (1u << i);
		else
			pattern &= ~(1u << i);
	}
	return pattern;
}

/* 0..1: how densely packed this channel's own pattern currently is --
 * k/n for Euclid, density/255 for Grids (grids_part >= 0 selects which).
 * Both ghost and chaos scale their probability by this, so a channel
 * with a sparse pattern sprouts proportionally fewer extra/flipped hits,
 * and a fully silent one (k==0 or density==0) sprouts none at all --
 * ghost/chaos are meant to vary a pattern that's there, not invent one
 * out of nothing. */
static float channel_density_frac(const o_drum_chan *d, int8_t grids_part)
{
	if (grids_part >= 0)
		return (float)d->density / 255.0f;
	return (d->euclid.n > 0) ? (float)d->euclid.k / (float)d->euclid.n : 0.0f;
}

/* A pattern-triggered hit (never a CV-triggered one -- that already has
 * real-world timing) goes through here instead of calling fire()
 * directly. `base_gain` is whatever the pattern engine already decided
 * (Grids' accent or a flat 1.0 for euclidean) -- humanize scales it by
 * a small random amount for velocity variation, and, since there's no
 * way to fire *before* the grid position, adds a small random delay
 * before the hit actually lands so it doesn't read as machine-quantized. */
static void schedule_pattern_hit(uint8_t c, float base_gain)
{
	o_drum_chan *d = &drum_chan[c];

	if (d->humanize <= 0.0f) {
		d->accent_gain = base_gain;
		fire(c);
		return;
	}

	/* Squared rather than linear: most of the knob's travel stays
	 * subtle (a small nudge low/mid), and the effect only really opens
	 * up approaching full -- makes the top end read as distinctly
	 * "loose"/"drunk" rather than a smooth, easy-to-miss ramp. */
	float amount = d->humanize * d->humanize;

	float jitter = (humanize_rand01() * 2.0f - 1.0f) * amount * DRUM_HUMANIZE_VELOCITY_RANGE;
	d->accent_gain = _CLAMP_F(base_gain * (1.0f + jitter), 0.15f, 1.3f);

	uint8_t delay = (uint8_t)(humanize_rand01() * amount * DRUM_HUMANIZE_MAX_DELAY_TICKS);
	if (delay > 0)
		d->fire_delay = delay;
	else
		fire(c);
}

/* Below this, a channel's step_phase reads as "just fired" -- close
 * enough to its last natural hit that forcing an immediate extra one
 * right on top of it reads as a double-trigger rather than a clean
 * resync. See reset_all_patterns() below. */
#define DRUM_RESET_SOFT_MARGIN	0.15f

/* LFO CV in as a hard pattern reset: on a rising edge past half scale,
 * every channel jumps straight to step 0 (Grids' shared step included)
 * and the shared bar counter restarts with it, so the whole kit's
 * patterns realign to the first beat together. This used to be a
 * "Global VCA" ducking input (see the removed read_lfo_cv() call in
 * params_lfo.c's update_lfo_params()) -- a continuous duck level and an
 * edge-triggered reset can't both live on the same jack, and the reset
 * is more useful for a drum station. Fires immediately, this same
 * tick, for any channel not already close to its own next hit (see
 * DRUM_RESET_SOFT_MARGIN below) -- it doesn't wait for the next clock
 * tick to land on step 0. */
static void reset_all_patterns(volatile uint16_t *bar_tick)
{
	*bar_tick = 0;

	/* Both engines advance-then-evaluate (grids_advance()/euclid_advance()
	 * both increment their step counter before it's read), so landing
	 * directly on step 0 here would mean the very next advance actually
	 * evaluates step 1 -- permanently skipping step 0 and playing every
	 * step one slot early. Parking one step *before* 0 instead, exactly
	 * like the per-channel do_resync path below already does for
	 * Euclid, means the very next advance (this same tick, via the
	 * drain check below) wraps onto 0 correctly. Without this, Grids
	 * channels stayed one step early forever (nothing else ever
	 * resyncs them); Euclid channels self-corrected at their own next
	 * bar boundary, but played one step early for however long that
	 * took to arrive. */
	grids_state.step = GRIDS_NUM_STEPS - 1;

	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		o_drum_chan *d = &drum_chan[c];

		d->euclid.current_step = d->euclid.n - 1;

		/* current_step is realigned above either way, so this
		 * channel's own next natural advance already wraps onto step
		 * 0 correctly -- if a hit only just landed (step_phase still
		 * near 0), a soft warp skips forcing a second one right now
		 * and just lets that next natural advance do the resync
		 * instead of cramming an extra hit in immediately. Anywhere
		 * else in the cycle still gets the hard snap, since that's the
		 * whole point of a reset -- realign now, not at the end of
		 * whatever's currently in flight. */
		if (d->step_phase >= DRUM_RESET_SOFT_MARGIN)
			d->step_phase = 1.0f;

		/* Also resync the divided-clock countdown here, the same way
		 * the do_resync path recomputes it after firing -- otherwise a
		 * channel slower than 1x keeps counting down from wherever it
		 * happened to be, and fires a second, phantom resync of its
		 * own some bars later on top of this one. */
		int bars_per_cycle = (d->clock_rate > 0.0f)
			? (int)(1.0f / d->clock_rate + 0.5f) : 1;
		if (bars_per_cycle < 1)
			bars_per_cycle = 1;
		d->bars_until_resync = (uint8_t)(bars_per_cycle - 1);
	}
}

static void read_reset_trigger(volatile uint16_t *bar_tick)
{
	static uint8_t prev_reset_high = 0;
	uint8_t reset_high = analog_jack_plugged(LFO_CV) && (analog[LFO_CV].bracketed_val > 2048);

	if (reset_high && !prev_reset_high)
		reset_all_patterns(bar_tick);
	prev_reset_high = reset_high;
}

void update_drum_triggers(void)
{
	static float	prev_clk_pos = 0.0f;
	static uint8_t	prev_cv_high[NUM_CHANNELS];

	/* The global clock's cycle position is advanced by update_lfos() off
	 * the recovered external/internal clock; a wrap back to ~0 is one
	 * master-clock tick, and DRUM_BAR_TICKS of those make up one shared
	 * bar. Each channel advances its own pattern by (n / DRUM_BAR_TICKS)
	 * steps per master tick -- fractional, accumulated in step_phase --
	 * so every channel's n-step pattern takes exactly one bar at
	 * clock_rate 1 regardless of n, and clock_rate then scales that
	 * per-channel on top (LFO speed), independent of every other
	 * channel.
	 *
	 * That per-channel step_phase is still float-accumulated, so it can
	 * drift out of sync with the other channels over time (e.g. editing
	 * n and changing it back doesn't necessarily land step_phase back
	 * where it started). To guarantee "same n -> same step, always" for
	 * channels running at the same rate, each channel is hard-resynced
	 * to step 0 on a shared bar boundary -- but only the boundary where
	 * its OWN loop actually completes (every round(1/clock_rate) bars),
	 * not every single one: a channel slowed to half speed takes two
	 * bars to play out its pattern, and snapping it back to step 0 at
	 * the end of the first bar would silently cancel the slowdown.
	 *
	 * This function runs every OSC_TIM tick (~1.8kHz), not just on a
	 * clk_step -- that matters below. */
	float clk_pos = lfos.cycle_pos[GLO_CLK];
	uint8_t clk_tick = (clk_pos < prev_clk_pos);
	prev_clk_pos = clk_pos;
	g_clk_frac = clk_pos;	/* read by read_automation() on the main loop for lane interpolation */

	read_reset_trigger(&g_bar_tick);

	if (pattern_resync_pending) {
		pattern_resync_pending = 0;
		reset_all_patterns(&g_bar_tick);
	}

	/* If a clock cable is plugged in but has gone quiet, read_ext_clk()
	 * clears lfos.use_ext_clock after its own timeout (2x the last
	 * period with no edge) -- but cycle_pos[GLO_CLK] keeps free-running
	 * on the last known tempo regardless, so without this check the
	 * pattern would keep playing forever even with the clock stopped
	 * or unplugged mid-performance. Nothing patched at all is
	 * unaffected: that's just this module's ordinary internal-tempo
	 * free-run, not a "clock stopped" condition. */
	uint8_t clock_present = !jack_plugged(CLK_SENSE) || lfos.use_ext_clock;
	uint8_t clk_step = clk_tick && clock_present;

	uint8_t bar_start = clk_step && (g_bar_tick == 0);

	if (bar_start) {
		/* A FINE+press mute toggle cued in performance mode lands here,
		 * on the beat, rather than the instant it was pressed -- see
		 * read_performance_controls(). */
		for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
			if (mute_toggle_pending[c]) {
				mute_toggle_pending[c] = 0;
				drum_chan[c].muted = !drum_chan[c].muted;
			}
		}
	}

	/* Grids shares one 32-step position across all its parts, so it
	 * advances once here rather than per-channel -- none of the
	 * euclidean bar-sync machinery below applies to it. It does share
	 * the same fractional step_phase/drain-one-per-tick technique as
	 * the euclidean channels though (grids_clock_rate, shared like x/y
	 * rather than per-channel), for the same reason: bursting several
	 * due steps in the same instant would collapse into one hit. */
	uint8_t grids_advanced = 0;
	if (drum_pattern_engine == PATTERN_ENGINE_GRIDS) {
		if (clk_step) {
			/* Guards against a bad grids_clock_rate (NaN, zero,
			 * negative) permanently freezing the shared stepper --
			 * every `grids_step_phase >= 1.0f` compare below would
			 * silently be false forever otherwise, with no crash to
			 * signal it. Self-heals the source too, not just this
			 * tick, so a bad value doesn't have to be caught here on
			 * every single tick from then on. `!(rate > 0.0f)` catches
			 * NaN as well as <= 0, since every comparison against NaN
			 * is false. */
			if (!(grids_clock_rate > 0.0f) || grids_clock_rate > 64.0f)
				grids_clock_rate = 1.0f;
			grids_step_phase += grids_clock_rate;
			if (grids_step_phase > 64.0f)
				grids_step_phase = 64.0f;
		}
		if (grids_step_phase >= 1.0f) {
			grids_step_phase -= 1.0f;
			grids_advance(&grids_state);
			grids_advanced = 1;
		}
	}

	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		o_drum_chan *d = &drum_chan[c];

		/* Runs every tick regardless of engine/branch below, so a
		 * humanize-delayed hit from schedule_pattern_hit() still fires
		 * on schedule even once this loop iteration continues past it. */
		if (d->fire_delay > 0) {
			d->fire_delay--;
			if (d->fire_delay == 0)
				fire(c);
		}

		uint8_t cv_high = 0;

		/* A plugged CV jack takes over as this channel's trigger source
		 * only in DRUM_CV_TARGET_TRIGGER -- any other target is read
		 * elsewhere instead (read_cv_param_mod(), main loop) and leaves
		 * the pattern engine driving triggers normally here. Either way
		 * the pattern keeps advancing underneath, so switching target
		 * or unplugging drops back in sync rather than at a stale step. */
		uint8_t cv_override = (d->cv_mode == DRUM_CV_TARGET_TRIGGER) && analog_jack_plugged(A_VOCT + c);
		if (cv_override) {
			float cv = analog[A_VOCT + c].lpf_val / 4095.0f;
			cv_high = (cv > DRUM_CV_TRIG_THRESHOLD);
		}

		uint8_t pattern_hit = 0;
		uint8_t ghost_hit = 0;
		int8_t grids_part = (drum_pattern_engine == PATTERN_ENGINE_GRIDS) ? drum_chan_grids_part(c) : -1;

		if (grids_part >= 0) {
			uint8_t out_level = 0;

			/* grids_advanced, not clk_step -- at a multiplied
			 * grids_clock_rate the shared stepper can advance more
			 * than once per master-clock tick (spread across
			 * subsequent OSC_TIM ticks, see above), and each of those
			 * advances needs its own trigger evaluation or the
			 * in-between steps would be silently skipped. */
			if (grids_advanced) {
				float density_frac = channel_density_frac(d, grids_part);

				/* New lap: mutate which steps are ghost hits before
				 * evaluating step 0 below, so this lap's ghost layer
				 * (and step 0 itself) is settled up front. Scaled by
				 * density_frac -- see its doc comment. */
				if (grids_state.step == 0)
					d->ghost_pattern = mutate_ghost_pattern(d->ghost_pattern, d->ghost_amount * density_frac, GRIDS_NUM_STEPS);

				/* pattern_chaos itself scaled by density_frac too, same
				 * reasoning as ghost above -- a near-empty pattern
				 * shouldn't get proportionally the same chaos jitter as
				 * a dense one. */
				pattern_hit = grids_step_active(&grids_state, (uint8_t)grids_part, grids_state.step,
				                                grids_x, grids_y, d->density,
				                                (uint8_t)((float)pattern_chaos * density_frac), &out_level);
				/* Ghost: an extra quiet hit on a step Grids itself
				 * didn't fire -- chaos is already baked into
				 * grids_step_active() above, so it doesn't need a
				 * separate roll here the way Euclid does below. */
				if (!pattern_hit && ((d->ghost_pattern >> grids_state.step) & 1u))
					ghost_hit = 1;
			}

			if (cv_override) {
				/* CV bypasses Grids entirely -- no level to derive an
				 * accent from, so these always land at normal gain,
				 * and humanize doesn't apply (real-world timing already). */
				if (cv_high && !prev_cv_high[c]) {
					d->accent_gain = DRUM_ACCENT_GAIN;
					fire(c);
				}
			} else if (pattern_hit) {
				float base_gain = (out_level > GRIDS_ACCENT_LEVEL) ? DRUM_ACCENT_GAIN : DRUM_UNACCENT_GAIN;
				schedule_pattern_hit(c, base_gain);
			} else if (ghost_hit) {
				schedule_pattern_hit(c, DRUM_GHOST_GAIN);
			}

			prev_cv_high[c] = cv_high;
			continue;
		}

		if (clk_step) {
			uint8_t do_resync = 0;

			if (bar_start) {
				if (d->bars_until_resync == 0) {
					do_resync = 1;
					int bars_per_cycle = (d->clock_rate > 0.0f)
						? (int)(1.0f / d->clock_rate + 0.5f) : 1;
					if (bars_per_cycle < 1)
						bars_per_cycle = 1;
					d->bars_until_resync = (uint8_t)(bars_per_cycle - 1);
				} else {
					d->bars_until_resync--;
				}
			}

			if (do_resync) {
				/* Force the *next* advance below to land exactly on
				 * step 0: parking current_step one before it and
				 * step_phase at exactly 1.0 guarantees precisely one
				 * euclid_advance() call, through the same single
				 * mechanism every other step uses (no separate
				 * explicit-fire path to double-count against). */
				d->euclid.current_step = d->euclid.n - 1;
				d->step_phase = 1.0f;

				/* This channel's own pattern is restarting -- mutate
				 * which steps are ghost hits for the lap about to
				 * begin, same as Grids does on its own lap wrap. Tied
				 * to do_resync (this channel's own loop boundary), not
				 * the shared bar_start, so a slowed-down channel's
				 * ghost layer holds for its whole (possibly
				 * multi-bar) loop rather than reshuffling mid-pattern.
				 * Scaled by this channel's own density (k/n) -- see
				 * channel_density_frac()'s doc comment. */
				d->ghost_pattern = mutate_ghost_pattern(
					d->ghost_pattern, d->ghost_amount * channel_density_frac(d, -1), (uint8_t)d->euclid.n);
			} else {
				d->step_phase += (d->euclid.n / (float)DRUM_BAR_TICKS) * d->clock_rate;
				/* A sustained clock_rate faster than this channel can
				 * drain via the one-step-per-OSC_TIM-tick loop below
				 * would otherwise grow step_phase without bound. */
				if (d->step_phase > 64.0f)
					d->step_phase = 64.0f;
			}
		}

		/* Drain at most one due step per call (every OSC_TIM tick, not
		 * just on a clk_step) rather than bursting through all of them
		 * in one go: a fast clock_rate can queue up several steps on a
		 * single clk_step, and firing them all in the same instant
		 * would collapse into a single audible hit (trigger_pending is
		 * a flag, not a counter) instead of the intended rapid-fire
		 * notes. Spreading the drain across the ~1.8kHz ticks between
		 * master-clock pulses gives them real, if tight, spacing. */
		uint8_t chaos_flip = 0;

		if (d->step_phase >= 1.0f) {
			d->step_phase -= 1.0f;
			pattern_hit = euclid_advance(&d->euclid);

			/* Chaos: symmetric flip of this step's fire decision --
			 * borrowed from Grids, where a chaos-perturbed level can
			 * land either side of its threshold. An active step can
			 * go silent, a silent one can fire (at DRUM_UNACCENT_GAIN,
			 * not the ghost gain -- this is meant to read as "the
			 * pattern itself varied", not a soft ornament on top of
			 * it, which is ghost's job below). Per-channel, unlike
			 * Grids' one shared pattern_chaos -- see chaos_amount's
			 * doc comment in drum_ui.h. Scaled by this channel's own
			 * density (k/n), same reasoning as ghost -- a near-empty
			 * pattern (k near 0) shouldn't flip nearly as often as a
			 * dense one, and a fully silent one (k==0) never flips. */
			if (d->chaos_amount > 0 &&
			    humanize_rand01() < (float)d->chaos_amount / 255.0f * channel_density_frac(d, -1)) {
				pattern_hit = !pattern_hit;
				chaos_flip = 1;
			}

			/* Ghost: an extra quiet hit on a step that still isn't
			 * firing after the chaos flip above -- from the stable,
			 * once-per-lap ghost_pattern rolled in do_resync above,
			 * not a fresh roll every single step. */
			if (!pattern_hit && ((d->ghost_pattern >> d->euclid.current_step) & 1u))
				ghost_hit = 1;
		}

		if (cv_override) {
			if (cv_high && !prev_cv_high[c])
				fire(c);
		} else if (pattern_hit) {
			/* Euclid's own accent: the first step of the pattern
			 * (current_step wraps to 0 right after euclid_advance())
			 * reads as the downbeat and fires loud, same binary split
			 * Grids already has -- everything else, including a
			 * chaos-flipped-on step, fires normal. */
			float base_gain = (!chaos_flip && d->euclid.current_step == 0) ? DRUM_ACCENT_GAIN : DRUM_UNACCENT_GAIN;
			schedule_pattern_hit(c, base_gain);
		} else if (ghost_hit) {
			schedule_pattern_hit(c, DRUM_GHOST_GAIN);
		}

		prev_cv_high[c] = cv_high;
	}

	if (clk_step)
		g_bar_tick = (g_bar_tick + 1) % DRUM_BAR_TICKS;
}

/* ── Audio ─────────────────────────────────────────────────────────────── */

/* Own RNG, separate from humanize_rng: everything below runs in the
 * audio ISR, not OSC_TIM, so sharing mutable RNG state across two
 * different interrupt contexts without locking would race. */
static uint32_t render_rng = 0xB5297A4Du;

static float render_rand01(void)
{
	render_rng ^= render_rng << 13;
	render_rng ^= render_rng >> 17;
	render_rng ^= render_rng << 5;
	return (float)(render_rng >> 8) / (float)0x00FFFFFFu;
}

/* Rerolls a fresh bipolar offset (-amount..+amount) for each of filter/
 * decay/other that has a nonzero *_random amount, on top of that
 * param's own base value -- one new roll per hit, not held between
 * hits. Called right before ops->trigger() below. */
static void apply_random_offsets(uint8_t c)
{
	o_drum_chan *d = &drum_chan[c];
	if (!d->ops)
		return;

	if (d->filter_random > 0.0f) {
		float offset = (render_rand01() * 2.0f - 1.0f) * d->filter_random;
		d->ops->set_filter(d->state, _CLAMP_F(d->filter + offset, 0.0f, 1.0f));
	}
	if (d->decay_random > 0.0f) {
		float offset = (render_rand01() * 2.0f - 1.0f) * d->decay_random;
		d->ops->set_decay(d->state, _CLAMP_F(d->decay + offset, 0.0f, 1.0f));
	}
	if (d->other_random > 0.0f) {
		float offset = (render_rand01() * 2.0f - 1.0f) * d->other_random;
		d->ops->set_other(d->state, _CLAMP_F(d->other + offset, 0.0f, 1.0f));
	}
}

void drum_render_channel(uint8_t chan, float *out, int n)
{
	o_drum_chan *d = &drum_chan[chan];

	if (!d->ops) {
		memset(out, 0, (size_t)n * sizeof(float));
		return;
	}

	if (d->choke_pending) {
		d->choke_pending = 0;
		/* A choke wins over a same-block trigger for this channel --
		 * matches a real hi-hat choke group, where re-striking closed
		 * always cuts an open hit even if it "just" started. */
		d->trigger_pending = 0;
		d->ops->init(d->state);
		push_params(chan);
	}

	if (d->trigger_pending) {
		d->trigger_pending = 0;
		apply_random_offsets(chan);
		d->ops->trigger(d->state, d->pitch);
	}

	d->ops->render(d->state, out, n);
}
