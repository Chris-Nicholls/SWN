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
 * none. While held, browse-encoder turns adjust that channel's level
 * instead of rotating the selected channel's pattern. */
static int8_t drum_held_chan = -1;

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
#define DRUM_CV_FILTER_MOD_RANGE	0.5f	/* +/- this much filter, bipolar around the manual knob position, at full-scale CV */
/* LED update runs at 60 Hz, so 4 ticks is a ~66 ms visible blip. */
#define DRUM_FLASH_TICKS		4

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

		/* CV_MODE_DENSITY feeds this exact same density/k input from
		 * that channel's own CV jack instead of its physical slider --
		 * everything below (hysteresis, pickup, the squared k curve)
		 * is unchanged either way. Falls back to the physical slider
		 * if nothing's actually patched, so picking this mode with an
		 * empty jack doesn't just freeze the channel. */
		float slider01;
		if (drum_chan[c].cv_mode == CV_MODE_DENSITY && analog_jack_plugged(A_VOCT + c))
			slider01 = _CLAMP_F(analog[A_VOCT + c].lpf_val / 4095.0f, 0.0f, 1.0f);
		else
			slider01 = _CLAMP_F(analog[A_SLIDER + c].lpf_val / 4095.0f, 0.0f, 1.0f);

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

static void read_channel_buttons(void)
{
	static uint8_t prev_pressed[NUM_CHANNELS];
	int8_t held = -1;

	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		uint8_t now = (button_pressed(c) != RELEASED);
		if (now && !prev_pressed[c]) {
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
		if (now)
			held = (int8_t)c;
		prev_pressed[c] = now;
	}
	drum_held_chan = held;
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
 * precedent as the slider-pickup fix and CV-density mode. Doesn't apply
 * while RECORD is armed (FINE held): that's the manual knob turning
 * itself being captured, not something to cancel. */
static void cancel_automation_if_playing(uint8_t c)
{
	o_drum_chan *dc = &drum_chan[c];
	if (dc->automation_state == AUTOMATION_PLAY)
		dc->automation_state = AUTOMATION_OFF;
}

/* Applies `delta` to *one* channel's filter, re-pushing to its voice.
 * Broken out so the DEPTH handler below can hit either just the
 * selected channel or (in global edit mode) all six identically. */
static void apply_filter_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_playing(c);
	dc->filter = _CLAMP_F(dc->filter + delta, 0.0f, 1.0f);
	if (dc->ops) dc->ops->set_filter(dc->state, dc->filter);
}

static void apply_decay_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_playing(c);
	dc->decay = _CLAMP_F(dc->decay + delta, 0.0f, 1.0f);
	if (dc->ops) dc->ops->set_decay(dc->state, dc->decay);
}

static void apply_other_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	cancel_automation_if_playing(c);
	dc->other = _CLAMP_F(dc->other + delta, 0.0f, 1.0f);
	if (dc->ops) dc->ops->set_other(dc->state, dc->other);
}

static void apply_filter_random_delta(uint8_t c, float delta)
{
	drum_chan[c].filter_random = _CLAMP_F(drum_chan[c].filter_random + delta, 0.0f, 1.0f);
}

static void apply_decay_random_delta(uint8_t c, float delta)
{
	drum_chan[c].decay_random = _CLAMP_F(drum_chan[c].decay_random + delta, 0.0f, 1.0f);
}

static void apply_other_random_delta(uint8_t c, float delta)
{
	drum_chan[c].other_random = _CLAMP_F(drum_chan[c].other_random + delta, 0.0f, 1.0f);
}

static void apply_pitch_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	dc->pitch = _CLAMP_F(dc->pitch + delta, -24.0f, 24.0f);
}

static void apply_clock_rate_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	dc->clock_divmult_id = _CLAMP_F(dc->clock_divmult_id + delta, LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID);
	dc->clock_rate = calc_divmult_amount(dc->clock_divmult_id);
}

static void apply_humanize_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	dc->humanize = _CLAMP_F(dc->humanize + delta, 0.0f, 1.0f);
}

static void apply_ghost_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	dc->ghost_amount = _CLAMP_F(dc->ghost_amount + delta, 0.0f, 1.0f);
}

static void apply_chaos_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	dc->chaos_amount = (uint8_t)_CLAMP_I32((int32_t)dc->chaos_amount + (int32_t)delta, 0, 255);
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

static void read_voice_encoders(void)
{
	o_drum_chan *d = &drum_chan[drum_selected_chan];
	int16_t enc;

	enc = pop_encoder_q(pec_DEPTH);
	if (enc) {
		apply_to_selected_or_all(apply_filter_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_FILTER);
	}

	/* Push+turn on DEPTH (sec_DISPERSION, dead in the old wavetable UI)
	 * -- how much random bipolar offset gets added to filter on each
	 * hit (see apply_random_offsets() in drum_render_channel()). */
	enc = pop_encoder_q(sec_DISPERSION);
	if (enc) {
		apply_to_selected_or_all(apply_filter_random_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_FILTER_RANDOM);
	}

	enc = pop_encoder_q(pec_LATITUDE);
	if (enc) {
		apply_to_selected_or_all(apply_decay_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_DECAY);
	}

	/* Push+turn on LATITUDE (sec_DISPPATT) -- same idea, for decay.
	 * Used to be ghost-note amount; that moved to OCT push+turn below
	 * to make room for all three params to get this symmetrically. */
	enc = pop_encoder_q(sec_DISPPATT);
	if (enc) {
		apply_to_selected_or_all(apply_decay_random_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_DECAY_RANDOM);
	}

	enc = pop_encoder_q(pec_LONGITUDE);
	if (enc) {
		apply_to_selected_or_all(apply_other_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_OTHER);
	}

	/* Push+turn on LONGITUDE (sec_WTSEL_SPREAD, dead in the old
	 * wavetable UI) -- same idea, for other. */
	enc = pop_encoder_q(sec_WTSEL_SPREAD);
	if (enc) {
		apply_to_selected_or_all(apply_other_random_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_OTHER_RANDOM);
	}

	enc = pop_encoder_q(pec_TRANSPOSE);
	if (enc)
		apply_to_selected_or_all(apply_pitch_delta, (float)enc * DRUM_PITCH_STEP);

	/* Push+turn on the same encoder (sec_OSC_SPREAD, "spread" in the
	 * old wavetable UI this panel used to drive) -- humanize amount:
	 * see schedule_pattern_hit() in update_drum_triggers() for what it
	 * actually does to a hit. Global-edit-mode-aware like every other
	 * knob here. */
	enc = pop_encoder_q(sec_OSC_SPREAD);
	if (enc) {
		apply_to_selected_or_all(apply_humanize_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_HUMANIZE);
	}

	/* Clock divide/multiply. FINE used to fine-scale this; FINE is now
	 * dedicated entirely to automation record/play (see
	 * read_automation()), so this always uses its one coarse step now.
	 * In Grids mode this is one shared rate for the whole stepper --
	 * unconditional on selected channel, same as x/y -- since all four
	 * Grids-driven channels read the one grids_state.step; in Euclid
	 * mode it stays per-channel (global-edit-mode-aware). */
	enc = pop_encoder_q(pec_LFOSPEED);
	if (enc) {
		if (drum_pattern_engine == PATTERN_ENGINE_GRIDS) {
			grids_clock_divmult_id = _CLAMP_F(grids_clock_divmult_id + (float)enc, LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID);
			grids_clock_rate = calc_divmult_amount(grids_clock_divmult_id);
		} else {
			apply_to_selected_or_all(apply_clock_rate_delta, (float)enc);
		}
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_SPEED);
	}

	/* Plain turn on OCT (fully dead otherwise) -- chaos amount. Grids
	 * mode edits the one shared pattern_chaos (its parts have no other
	 * per-channel identity); Euclid mode edits the selected channel's
	 * own chaos_amount instead, same selected-vs-global convention as
	 * every other per-channel knob here, since each Euclid channel
	 * already has its own independent pattern. */
	enc = pop_encoder_q(pec_OCT);
	if (enc) {
		if (drum_pattern_engine == PATTERN_ENGINE_GRIDS) {
			pattern_chaos = (uint8_t)_CLAMP_I32((int32_t)pattern_chaos + enc * DRUM_CHAOS_STEP, 0, 255);
		} else {
			apply_to_selected_or_all(apply_chaos_delta, (float)(enc * DRUM_CHAOS_STEP));
		}
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_CHAOS);
	}

	/* Push+turn on OCT (sec_SCALE, dead in the old wavetable UI) --
	 * ghost-note amount, same shape as humanize above. Paired with
	 * chaos on the same physical encoder since both are "pattern
	 * variation" controls. */
	enc = pop_encoder_q(sec_SCALE);
	if (enc) {
		apply_to_selected_or_all(apply_ghost_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_GHOST);
	}

	/* Voice selection deliberately ignores global edit mode -- "same
	 * position in each channel's own category" isn't a coherent action
	 * across channels with different categories, unlike a plain knob
	 * delta. Cycle the selected channel through its own category's voices only
	 * (see kChannelCategory), so a channel is always "a kick" (etc.) no
	 * matter how far this gets turned. Re-init rather than carry over
	 * DSP state across a voice swap -- the old voice's envelope/
	 * oscillator phase means nothing to the new one -- then re-push
	 * filter/decay/other so the knobs don't silently reset to that
	 * voice's own defaults. */
	enc = pop_encoder_q(pec_LFOSHAPE);
	if (enc) {
		DrumVoiceCategory cat = kChannelCategory[drum_selected_chan];
		const DrumVoiceOps *new_ops = cycle_voice_in_category(cat, d->ops, enc);
		if (new_ops && new_ops->state_size <= DRUM_VOICE_STATE_BYTES) {
			d->ops = new_ops;
			d->ops->init(d->state);
			push_params(drum_selected_chan);
		}
	}
}

static void read_pattern_encoder(void)
{
	EuclidChannelState *e = &drum_chan[drum_selected_chan].euclid;

	/* Plain turn rotates the selected channel's pattern, unless a
	 * channel button is currently held -- in that case the same turn
	 * instead nudges that held channel's level (k is now the sliders'
	 * job, so this encoder no longer needs to touch it). */
	int16_t enc = pop_encoder_q(pec_WBROWSE);
	int16_t enc2 = pop_encoder_q(sec_WTSEL);

	/* Grids mode repoints this encoder at the shared map position:
	 * rotation and n don't exist there. Holding a channel for level
	 * still wins over both, in either mode. */
	if (drum_pattern_engine == PATTERN_ENGINE_GRIDS) {
		if (enc) {
			if (drum_held_chan >= 0) {
				o_drum_chan *held = &drum_chan[drum_held_chan];
				held->level = _CLAMP_F(held->level + (float)enc * DRUM_PARAM_STEP, 0.0f, 1.0f);
			} else {
				grids_x = (uint8_t)_CLAMP_I32((int32_t)grids_x + enc * DRUM_GRIDS_XY_STEP, 0, 255);
				start_ongoing_display_drum_param(DRUM_PARAM_DISP_GRIDS_X);
			}
		}
		if (enc2) {
			grids_y = (uint8_t)_CLAMP_I32((int32_t)grids_y + enc2 * DRUM_GRIDS_XY_STEP, 0, 255);
			start_ongoing_display_drum_param(DRUM_PARAM_DISP_GRIDS_Y);
		}
		return;
	}

	if (enc) {
		if (drum_held_chan >= 0) {
			o_drum_chan *held = &drum_chan[drum_held_chan];
			held->level = _CLAMP_F(held->level + (float)enc * DRUM_PARAM_STEP, 0.0f, 1.0f);
		} else {
			__disable_irq();
			/* euclid_rotate() advances a step's *source* index with
			 * +rotation, which moves onsets to earlier step indices
			 * (anticipates) as rotation increases -- backwards from
			 * the expected "turn = later/lag" feel, hence the sign
			 * flip here rather than in the (tested) pattern engine. */
			euclid_set_rotation(e, e->rotation - enc);
			__enable_irq();
		}
	}

	/* Push+turn: total step count, always for the selected channel. */
	if (enc2) {
		__disable_irq();
		euclid_set_n(e, e->n + enc2);
		__enable_irq();
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

/* butm_LFOMODE_BUTTON is otherwise fully dead on the drum station --
 * cycles the selected channel's CV-jack mode (or every channel's, in
 * global edit mode, same selected-vs-global convention as the knob
 * handlers above). Edge-triggered for the same reason the pattern-
 * engine button above is. */
static void read_cv_mode_button(void)
{
	static uint8_t prev_pressed = 0;
	uint8_t now = (button_pressed(butm_LFOMODE_BUTTON) != RELEASED);

	if (now && !prev_pressed) {
		if (drum_global_edit_mode) {
			for (uint8_t c = 0; c < NUM_CHANNELS; c++)
				drum_chan[c].cv_mode = (drum_chan[c].cv_mode + 1) % NUM_CV_MODES;
		} else {
			o_drum_chan *d = &drum_chan[drum_selected_chan];
			d->cv_mode = (d->cv_mode + 1) % NUM_CV_MODES;
		}
		start_ongoing_display_drum_cv_mode();
	}
	prev_pressed = now;
}

/* CV_MODE_FILTER: adds the patched CV (bipolar around the manual knob
 * position, so it can sweep the filter both ways rather than only ever
 * opening it further) on top of `filter` and re-pushes every tick --
 * unlike the encoder-driven filter/decay/other above, this has to run
 * continuously rather than only on a change, since the CV itself can be
 * moving every tick. Cheap: set_filter() is a coefficient recompute,
 * not audio-rate work. Leaves `filter` itself untouched so the manual
 * knob position underneath is never clobbered -- switching back to
 * CV_MODE_TRIGGER/DENSITY (or unpatching) just drops the modulation and
 * resumes exactly at the knob's own value. */
static void read_cv_filter_mod(void)
{
	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		o_drum_chan *d = &drum_chan[c];
		if (d->cv_mode != CV_MODE_FILTER || !d->ops)
			continue;

		float mod = 0.0f;
		if (analog_jack_plugged(A_VOCT + c)) {
			float cv01 = _CLAMP_F(analog[A_VOCT + c].lpf_val / 4095.0f, 0.0f, 1.0f);
			mod = (cv01 - 0.5f) * 2.0f * DRUM_CV_FILTER_MOD_RANGE;
		}

		d->ops->set_filter(d->state, _CLAMP_F(d->filter + mod, 0.0f, 1.0f));
	}
}

/* Bar-tick and the master clock's fractional progress through the
 * current tick, both written once per OSC_TIM tick in
 * update_drum_triggers() below and read (loosely -- a stale-by-one-tick
 * read is imperceptible here) by read_automation() on the main loop to
 * index/interpolate the automation lanes. */
static volatile uint16_t g_bar_tick  = 0;
static volatile float    g_clk_frac  = 0.0f;

/* FINE dedicated entirely to automation record/play for filter/decay/
 * other, one lane per channel, one bar long (DRUM_BAR_TICKS points,
 * linearly interpolated between them). Holding FINE arms/continues
 * RECORD on the selected channel (or every channel, in global edit
 * mode) -- sampling live knob values once per bar_tick, wrapping and
 * overwriting continuously, so a hold of any length just keeps the
 * most recent lap. Releasing commits and starts PLAY, looping forever
 * until a manual knob edit cancels it (see cancel_automation_if_playing())
 * or FINE is held again for a fresh take. Playback always runs for
 * every channel regardless of selection, so switching which channel is
 * selected mid-loop doesn't stall one that's already playing. */
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
			d->automation_filter[tick] = d->filter;
			d->automation_decay[tick]  = d->decay;
			d->automation_other[tick]  = d->other;
			continue;
		}

		if (d->automation_state == AUTOMATION_RECORD) {
			d->automation_state = ((now_ms - record_start_ms[c]) >= AUTOMATION_MIN_RECORD_MS)
			                     ? AUTOMATION_PLAY : AUTOMATION_OFF;
		}

		if (d->automation_state != AUTOMATION_PLAY || !d->ops)
			continue;

		d->filter = _CLAMP_F(d->automation_filter[tick] + (d->automation_filter[next] - d->automation_filter[tick]) * frac, 0.0f, 1.0f);
		d->decay  = _CLAMP_F(d->automation_decay[tick]  + (d->automation_decay[next]  - d->automation_decay[tick])  * frac, 0.0f, 1.0f);
		d->other  = _CLAMP_F(d->automation_other[tick]  + (d->automation_other[next]  - d->automation_other[tick])  * frac, 0.0f, 1.0f);

		d->ops->set_filter(d->state, d->filter);
		d->ops->set_decay(d->state, d->decay);
		d->ops->set_other(d->state, d->other);
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
	was_performance_mode = 0;

	read_channel_sliders();
	read_channel_buttons();
	read_voice_encoders();
	read_pattern_encoder();
	read_pattern_engine_button();
	read_cv_mode_button();
	read_cv_filter_mod();
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

#define DRUM_HUMANIZE_MAX_DELAY_TICKS	12u		/* ~6.7ms at the 1.8kHz OSC_TIM rate; late-only, see schedule_pattern_hit() */
#define DRUM_HUMANIZE_VELOCITY_RANGE	0.4f	/* +/- this fraction of gain at humanize=1.0 */

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

	float jitter = (humanize_rand01() * 2.0f - 1.0f) * d->humanize * DRUM_HUMANIZE_VELOCITY_RANGE;
	d->accent_gain = _CLAMP_F(base_gain * (1.0f + jitter), 0.15f, 1.3f);

	uint8_t delay = (uint8_t)(humanize_rand01() * d->humanize * DRUM_HUMANIZE_MAX_DELAY_TICKS);
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
		 * only in CV_MODE_TRIGGER -- CV_MODE_DENSITY/FILTER are read
		 * elsewhere (read_channel_sliders()/read_cv_filter_mod(), main
		 * loop) and leave the pattern engine driving triggers normally
		 * here. Either way the pattern keeps advancing underneath, so
		 * switching mode or unplugging drops back in sync rather than
		 * at a stale step. */
		uint8_t cv_override = (d->cv_mode == CV_MODE_TRIGGER) && analog_jack_plugged(A_VOCT + c);
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
