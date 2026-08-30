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

#define DRUM_CV_TRIG_THRESHOLD	0.2f
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
}

/* ── Control surface ─────────────────────────────────────────────────── */

/* Each channel's own slider sets its own pattern density (active-step
 * count k) directly -- no channel selection needed to sculpt a beat.
 * Actually moving a slider enough to change k also jumps edit focus to
 * that channel, so Depth/Latitude/Longitude/browse immediately act on
 * whichever voice you're touching. */
static void read_channel_sliders(void)
{
	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		EuclidChannelState *e = &drum_chan[c].euclid;
		float slider01 = _CLAMP_F(analog[A_SLIDER + c].lpf_val / 4095.0f, 0.0f, 1.0f);
		int target_k = (int)(slider01 * (float)e->n + 0.5f);

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
		if (now && !prev_pressed[c])
			drum_selected_chan = c;
		if (now)
			held = (int8_t)c;
		prev_pressed[c] = now;
	}
	drum_held_chan = held;
}

static void read_voice_encoders(void)
{
	o_drum_chan *d = &drum_chan[drum_selected_chan];
	int16_t enc;

	enc = pop_encoder_q(pec_DEPTH);
	if (enc) {
		d->filter = _CLAMP_F(d->filter + (float)enc * DRUM_PARAM_STEP, 0.0f, 1.0f);
		if (d->ops) d->ops->set_filter(d->state, d->filter);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_FILTER);
	}

	enc = pop_encoder_q(pec_LATITUDE);
	if (enc) {
		d->decay = _CLAMP_F(d->decay + (float)enc * DRUM_PARAM_STEP, 0.0f, 1.0f);
		if (d->ops) d->ops->set_decay(d->state, d->decay);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_DECAY);
	}

	enc = pop_encoder_q(pec_LONGITUDE);
	if (enc) {
		d->other = _CLAMP_F(d->other + (float)enc * DRUM_PARAM_STEP, 0.0f, 1.0f);
		if (d->ops) d->ops->set_other(d->state, d->other);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_OTHER);
	}

	enc = pop_encoder_q(pec_TRANSPOSE);
	if (enc)
		d->pitch = _CLAMP_F(d->pitch + (float)enc * DRUM_PITCH_STEP, -24.0f, 24.0f);

	/* Clock divide/multiply for the selected channel only -- deliberately
	 * not global, unlike the old per-module LFO speed this encoder used
	 * to drive. FINE gives fractional crossfade between ratios, matching
	 * that control's feel elsewhere on the panel. */
	enc = pop_encoder_q(pec_LFOSPEED);
	if (enc) {
		float step = switch_pressed(FINE_BUTTON) ? (float)enc * F_SCALING_FINE_LFO_SPEED : (float)enc;
		d->clock_divmult_id = _CLAMP_F(d->clock_divmult_id + step, LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID);
		d->clock_rate = calc_divmult_amount(d->clock_divmult_id);
		start_ongoing_display_drum_param(DRUM_PARAM_DISP_SPEED);
	}

	/* Cycle the selected channel through its own category's voices only
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

void read_drum_ui(void)
{
	read_channel_sliders();
	read_channel_buttons();
	read_voice_encoders();
	read_pattern_encoder();
}

/* ── Clock / trigger ─────────────────────────────────────────────────── */

static void fire(uint8_t chan)
{
	drum_chan[chan].trigger_pending = 1;
	drum_trig_flash[chan] = DRUM_FLASH_TICKS;
	drum_gate_ticks[chan] = DRUM_GATE_TICKS;
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
	 * so every channel's n-step pattern takes exactly one bar
	 * regardless of n, and clock_rate then scales that per-channel on
	 * top (LFO speed), independent of every other channel.
	 *
	 * That per-channel step_phase is still float-accumulated, so it can
	 * drift out of sync with the other channels over time (e.g. editing
	 * n and changing it back doesn't necessarily land step_phase back
	 * where it started). To guarantee "same n -> same step, always",
	 * every channel is hard-reset to step 0 at the start of each shared
	 * bar_tick cycle, regardless of what happened to it mid-bar. */
	float clk_pos = lfos.cycle_pos[GLO_CLK];
	uint8_t clk_step = (clk_pos < prev_clk_pos);
	prev_clk_pos = clk_pos;

	uint8_t bar_start = clk_step && (bar_tick == 0);

	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
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
		if (clk_step) {
			o_drum_chan *d = &drum_chan[c];

			if (bar_start) {
				/* Force the *next* advance below to land exactly on
				 * step 0: parking current_step one before it and
				 * step_phase at exactly 1.0 guarantees precisely one
				 * euclid_advance() call this tick, through the same
				 * single mechanism every other step uses. (Previously
				 * this resynced to step 0 with an explicit check AND
				 * still ran the normal per-tick advance below, which
				 * silently advanced an extra step every bar --
				 * double-firing step 0 and eating a step's worth of
				 * budget that made the pattern's last step
				 * intermittently never get its own tick.) */
				d->euclid.current_step = d->euclid.n - 1;
				d->step_phase = 1.0f;
			} else {
				d->step_phase += (d->euclid.n / (float)DRUM_BAR_TICKS) * d->clock_rate;
			}

			/* Bounded catch-up: a pathological clock_rate can't spin
			 * this loop forever and starve the ISR. */
			for (uint8_t guard = 0; guard < 16 && d->step_phase >= 1.0f; guard++) {
				d->step_phase -= 1.0f;
				if (euclid_advance(&d->euclid))
					pattern_hit = 1;
			}
		}

		if (cv_override) {
			if (cv_high && !prev_cv_high[c])
				fire(c);
		} else if (pattern_hit) {
			fire(c);
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

	if (d->trigger_pending) {
		d->trigger_pending = 0;
		d->ops->trigger(d->state, d->pitch);
	}

	d->ops->render(d->state, out, n);
}
