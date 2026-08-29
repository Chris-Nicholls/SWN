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
#include "math_util.h"

extern o_analog	analog[NUM_ANALOG_ELEMENTS];

o_drum_chan	drum_chan[NUM_CHANNELS];
uint8_t		drum_selected_chan = 0;

/* Counts down in LED-update ticks; non-zero means "this channel just hit"
 * and its button LED is flashed. Written from OSC_TIM, read/decremented
 * by led_cont.c. */
volatile uint8_t drum_trig_flash[NUM_CHANNELS];

/* MVP kit: kick on A, snare on B, remaining channels silent until the
 * rest of the mpump family is ported. Indexed by channel so adding a
 * voice is a one-line change here, not a new special case. */
static const DrumVoiceOps *const kMvpKit[NUM_CHANNELS] = {
	&drum_voice_mpump_kick,
	&drum_voice_mpump_snare,
	0, 0, 0, 0
};

#define DRUM_CV_TRIG_THRESHOLD	0.2f
#define DRUM_PARAM_STEP			0.02f
#define DRUM_PITCH_STEP			1.0f	/* one semitone per encoder click */

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

		/* A default pattern per channel so the module makes a beat as
		 * soon as a clock arrives, without the user having to dial one
		 * in first. */
		euclid_set_n(&d->euclid, 16);
		euclid_set_k(&d->euclid, (c == 0) ? 4 : 2);
		euclid_set_rotation(&d->euclid, (c == 0) ? 0 : 4);

		d->level  = 0.0f;
		d->pitch  = 0.0f;
		d->filter = 1.0f;
		d->decay  = 0.5f;
		d->other  = 0.5f;

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

static void read_channel_sliders(void)
{
	for (uint8_t c = 0; c < NUM_CHANNELS; c++)
		drum_chan[c].level = _CLAMP_F(analog[A_SLIDER + c].lpf_val / 4095.0f, 0.0f, 1.0f);
}

static void read_channel_buttons(void)
{
	static uint8_t prev_pressed[NUM_CHANNELS];

	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		uint8_t now = (button_pressed(c) != RELEASED);
		if (now && !prev_pressed[c])
			drum_selected_chan = c;
		prev_pressed[c] = now;
	}
}

static void read_voice_encoders(void)
{
	o_drum_chan *d = &drum_chan[drum_selected_chan];
	int16_t enc;

	enc = pop_encoder_q(pec_DEPTH);
	if (enc) {
		d->filter = _CLAMP_F(d->filter + (float)enc * DRUM_PARAM_STEP, 0.0f, 1.0f);
		if (d->ops) d->ops->set_filter(d->state, d->filter);
	}

	enc = pop_encoder_q(pec_LATITUDE);
	if (enc) {
		d->decay = _CLAMP_F(d->decay + (float)enc * DRUM_PARAM_STEP, 0.0f, 1.0f);
		if (d->ops) d->ops->set_decay(d->state, d->decay);
	}

	enc = pop_encoder_q(pec_LONGITUDE);
	if (enc) {
		d->other = _CLAMP_F(d->other + (float)enc * DRUM_PARAM_STEP, 0.0f, 1.0f);
		if (d->ops) d->ops->set_other(d->state, d->other);
	}

	enc = pop_encoder_q(pec_TRANSPOSE);
	if (enc)
		d->pitch = _CLAMP_F(d->pitch + (float)enc * DRUM_PITCH_STEP, -24.0f, 24.0f);
}

static void read_pattern_encoder(void)
{
	EuclidChannelState *e = &drum_chan[drum_selected_chan].euclid;

	/* Plain turn adjusts k; FINE held turns the same encoder into a
	 * pattern rotate, matching the FINE-modifier idiom used by the rest
	 * of the encoders on this panel. */
	int16_t enc = pop_encoder_q(pec_WBROWSE);
	if (enc) {
		if (switch_pressed(FINE_BUTTON))
			euclid_set_rotation(e, e->rotation + enc);
		else
			euclid_set_k(e, e->k + enc);
	}

	/* Push+turn: total step count. */
	int16_t enc2 = pop_encoder_q(sec_WTSEL);
	if (enc2)
		euclid_set_n(e, e->n + enc2);
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
	drum_trig_flash[chan] = 30;
}

void update_drum_triggers(void)
{
	static float	prev_clk_pos = 0.0f;
	static uint8_t	prev_cv_high[NUM_CHANNELS];

	/* The global clock's cycle position is advanced by update_lfos() off
	 * the recovered external/internal clock; a wrap back to ~0 is one
	 * step of the sequencer (1:1 with the clock for now). */
	float clk_pos = lfos.cycle_pos[GLO_CLK];
	uint8_t clk_step = (clk_pos < prev_clk_pos);
	prev_clk_pos = clk_pos;

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
		if (clk_step)
			pattern_hit = euclid_advance(&drum_chan[c].euclid);

		if (cv_override) {
			if (cv_high && !prev_cv_high[c])
				fire(c);
		} else if (pattern_hit) {
			fire(c);
		}

		prev_cv_high[c] = cv_high;
	}
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
