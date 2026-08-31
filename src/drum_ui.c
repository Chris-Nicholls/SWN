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
uint8_t		grids_x     = 128;
uint8_t		grids_y     = 128;
uint8_t		grids_chaos = 0;	/* off by default: the map alone is already musical */

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

/* Each channel has a fixed role (category), one per DrumVoiceCategory,
 * in enum order -- channel c's category is literally (DrumVoiceCategory)c
 * (NUM_CHANNELS == NUM_DRUM_CATEGORIES == 6, by design). rotm_LFOSHAPE
 * cycles a channel through only its own category's registry entries
 * (see read_voice_encoders()), so a channel is always "a kick" (etc.)
 * no matter how far you turn it. This table is just the factory-
 * default pick within each category. */
static const DrumVoiceOps *const kMvpKit[NUM_CHANNELS] = {
	&drum_voice_mpump_kick,			/* DRUM_CAT_KICK */
	&drum_voice_mpump_snare,		/* DRUM_CAT_SNARE */
	&drum_voice_mpump_closed_hat,	/* DRUM_CAT_CLOSED_HAT */
	&drum_voice_mpump_open_hat,		/* DRUM_CAT_OPEN_HAT */
	&drum_voice_mpump_crash,		/* DRUM_CAT_CRASH */
	&drum_voice_mpump_cowbell,		/* DRUM_CAT_OTHER */
};

/* Channel roles are fixed by category (channel c IS category c, see
 * kMvpKit), so this is a plain table, not a per-voice lookup. Grids
 * authors only three parts; both hi-hat channels read the one hihat
 * part (their densities stay independent, so open/closed still thin
 * out separately), and Crash/Other have no Grids data at all. */
static const int8_t kChanGridsPart[NUM_CHANNELS] = {
	0,	/* DRUM_CAT_KICK       -> Grids kick */
	1,	/* DRUM_CAT_SNARE      -> Grids snare */
	2,	/* DRUM_CAT_CLOSED_HAT -> Grids hihat */
	2,	/* DRUM_CAT_OPEN_HAT   -> Grids hihat, shared */
	-1,	/* DRUM_CAT_CRASH      -> always euclidean */
	-1,	/* DRUM_CAT_OTHER      -> always euclidean */
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
#define DRUM_ACCENT_GAIN		1.0f	/* Grids-accented hit (level > GRIDS_ACCENT_LEVEL): full loudness */
#define DRUM_UNACCENT_GAIN		0.6f	/* everything else: pulled back rather than boosted, so accents can't newly clip */
#define DRUM_DENSITY_DETENTS	32u		/* slider -> Grids density resolution, see read_channel_sliders() */
#define DRUM_GRIDS_XY_STEP		6		/* encoder clicks are coarse: ~42 turns spans the whole map */
#define DRUM_PARAM_STEP			0.02f
#define DRUM_PITCH_STEP			1.0f	/* one semitone per encoder click */
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
		d->decay  = 0.5f;
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

static void read_channel_sliders(void)
{
	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		EuclidChannelState *e = &drum_chan[c].euclid;
		float slider01 = _CLAMP_F(analog[A_SLIDER + c].lpf_val / 4095.0f, 0.0f, 1.0f);

		/* Grids has no k -- the same slider becomes that part's
		 * density threshold instead. Crash/Other have no Grids data,
		 * so they keep driving k in either mode. */
		if (chan_is_grids_driven(c)) {
			/* Quantized to DRUM_DENSITY_DETENTS steps rather than the
			 * full 0..255, both for a coarser/more usable feel and so
			 * the hysteresis margin below means something (a margin on
			 * a 256-level range would be sub-single-bit). */
			uint8_t current_detent = (uint8_t)(((uint16_t)drum_chan[c].density * DRUM_DENSITY_DETENTS + 127u) / 255u);
			int target_detent = slider_to_level_hysteretic(slider01, current_detent, DRUM_DENSITY_DETENTS);
			uint8_t target_density = (uint8_t)(((uint32_t)target_detent * 255u) / DRUM_DENSITY_DETENTS);
			if (target_density != drum_chan[c].density) {
				drum_chan[c].density = target_density;
				drum_selected_chan = c;
			}
			continue;
		}

		int target_k = slider_to_level_hysteretic(slider01, e->k, e->n);

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

/* Applies `delta` to *one* channel's filter, re-pushing to its voice.
 * Broken out so the DEPTH handler below can hit either just the
 * selected channel or (in global edit mode) all six identically. */
static void apply_filter_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	dc->filter = _CLAMP_F(dc->filter + delta, 0.0f, 1.0f);
	if (dc->ops) dc->ops->set_filter(dc->state, dc->filter);
}

static void apply_decay_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	dc->decay = _CLAMP_F(dc->decay + delta, 0.0f, 1.0f);
	if (dc->ops) dc->ops->set_decay(dc->state, dc->decay);
}

static void apply_other_delta(uint8_t c, float delta)
{
	o_drum_chan *dc = &drum_chan[c];
	dc->other = _CLAMP_F(dc->other + delta, 0.0f, 1.0f);
	if (dc->ops) dc->ops->set_other(dc->state, dc->other);
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

	enc = pop_encoder_q(pec_LATITUDE);
	if (enc) {
		apply_to_selected_or_all(apply_decay_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_DECAY);
	}

	enc = pop_encoder_q(pec_LONGITUDE);
	if (enc) {
		apply_to_selected_or_all(apply_other_delta, (float)enc * DRUM_PARAM_STEP);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_OTHER);
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

	/* Clock divide/multiply -- global-edit-mode-aware like the others
	 * above, but otherwise still per-channel by default: FINE gives
	 * fractional crossfade between ratios, matching that control's
	 * feel elsewhere on the panel. */
	enc = pop_encoder_q(pec_LFOSPEED);
	if (enc) {
		float step = switch_pressed(FINE_BUTTON) ? (float)enc * F_SCALING_FINE_LFO_SPEED : (float)enc;
		apply_to_selected_or_all(apply_clock_rate_delta, step);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_SPEED);
	}

	/* Voice selection deliberately ignores global edit mode -- "same
	 * position in each channel's own category" isn't a coherent action
	 * across channels with different categories, unlike a plain knob
	 * delta. Cycle the selected channel through its own category's voices only
	 * (see kMvpKit's comment -- channel c's category is fixed at
	 * (DrumVoiceCategory)c), so a channel is always "a kick" (etc.) no
	 * matter how far this gets turned. Re-init rather than carry over
	 * DSP state across a voice swap -- the old voice's envelope/
	 * oscillator phase means nothing to the new one -- then re-push
	 * filter/decay/other so the knobs don't silently reset to that
	 * voice's own defaults. */
	enc = pop_encoder_q(pec_LFOSHAPE);
	if (enc) {
		DrumVoiceCategory cat = (DrumVoiceCategory)drum_selected_chan;
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
			} else if (switch_pressed(FINE_BUTTON))
				grids_chaos = (uint8_t)_CLAMP_I32((int32_t)grids_chaos + enc * DRUM_GRIDS_XY_STEP, 0, 255);
			else
				grids_x = (uint8_t)_CLAMP_I32((int32_t)grids_x + enc * DRUM_GRIDS_XY_STEP, 0, 255);
		}
		if (enc2)
			grids_y = (uint8_t)_CLAMP_I32((int32_t)grids_y + enc2 * DRUM_GRIDS_XY_STEP, 0, 255);
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

void read_drum_ui(void)
{
	read_channel_sliders();
	read_channel_buttons();
	read_voice_encoders();
	read_pattern_encoder();
	read_pattern_engine_button();
}

/* ── Clock / trigger ─────────────────────────────────────────────────── */

static void fire(uint8_t chan)
{
	drum_chan[chan].trigger_pending = 1;
	drum_trig_flash[chan] = DRUM_FLASH_TICKS;
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

/* LFO CV in as a hard pattern reset: on a rising edge past half scale,
 * every channel jumps straight to step 0 (Grids' shared step included)
 * and the shared bar counter restarts with it, so the whole kit's
 * patterns realign to the first beat together. This used to be a
 * "Global VCA" ducking input (see the removed read_lfo_cv() call in
 * params_lfo.c's update_lfo_params()) -- a continuous duck level and an
 * edge-triggered reset can't both live on the same jack, and the reset
 * is more useful for a drum station. Doesn't fire anything -- it just
 * repositions; the next clock tick advances (and triggers) normally
 * from step 0. */
static void reset_all_patterns(uint16_t *bar_tick)
{
	*bar_tick = 0;
	grids_state.step = 0;
	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		drum_chan[c].euclid.current_step = 0;
		drum_chan[c].step_phase = 0.0f;
	}
}

static void read_reset_trigger(uint16_t *bar_tick)
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
	static uint16_t	bar_tick = 0;	/* 0..DRUM_BAR_TICKS-1, shared by every channel */

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

	read_reset_trigger(&bar_tick);

	if (pattern_resync_pending) {
		pattern_resync_pending = 0;
		reset_all_patterns(&bar_tick);
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

	uint8_t bar_start = clk_step && (bar_tick == 0);

	/* Grids ticks 1:1 with the master clock and shares one 32-step
	 * position across all its parts, so it advances once here rather
	 * than per-channel -- none of the euclidean bar-sync/step_phase
	 * machinery below applies to it. */
	if (clk_step && drum_pattern_engine == PATTERN_ENGINE_GRIDS)
		grids_advance(&grids_state);

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

		/* A plugged CV jack takes over as this channel's trigger source,
		 * but the pattern keeps advancing underneath so unplugging drops
		 * back in sync rather than at a stale step. */
		uint8_t cv_override = analog_jack_plugged(A_VOCT + c);
		if (cv_override) {
			float cv = analog[A_VOCT + c].lpf_val / 4095.0f;
			cv_high = (cv > DRUM_CV_TRIG_THRESHOLD);
		}

		uint8_t pattern_hit = 0;
		int8_t grids_part = (drum_pattern_engine == PATTERN_ENGINE_GRIDS) ? drum_chan_grids_part(c) : -1;

		if (grids_part >= 0) {
			uint8_t out_level = 0;

			if (clk_step)
				pattern_hit = grids_step_active(&grids_state, (uint8_t)grids_part, grids_state.step,
				                                grids_x, grids_y, d->density, grids_chaos, &out_level);

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
		if (d->step_phase >= 1.0f) {
			d->step_phase -= 1.0f;
			if (euclid_advance(&d->euclid))
				pattern_hit = 1;
		}

		if (cv_override) {
			if (cv_high && !prev_cv_high[c])
				fire(c);
		} else if (pattern_hit) {
			schedule_pattern_hit(c, 1.0f);
		}

		prev_cv_high[c] = cv_high;
	}

	if (clk_step)
		bar_tick = (bar_tick + 1) % DRUM_BAR_TICKS;
}

/* ── Audio ─────────────────────────────────────────────────────────────── */

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
		d->ops->trigger(d->state, d->pitch);
	}

	d->ops->render(d->state, out, n);
}
