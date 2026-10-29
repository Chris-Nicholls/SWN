/*
 * drum_voice_deluge_cowbell.c
 *
 * Port of render_cowbell() from drum_prototype/render_deluge_drums.py:
 * a 3-operator chained FM stack (modulator2 -> modulator1 -> carrier)
 * on a short percussive envelope, soft-clipped at the output. No
 * filtering in this recipe beyond the shared Depth stage.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

/* ---- envelope ------------------------------------------------------------ */

typedef struct {
	int   a_n, d_n, r_n;
	float a_inv, d_inv, r_inv;
	float sustain;
	int   idx;
} DelugeAdsr;

#define ADSR_CURVE     5.0f
#define ADSR_ATT_NORM  1.0157473f

static int adsr_seg_len(float seconds)
{
	int n = (int)(seconds * DRUM_VOICE_SAMPLE_RATE);
	return (n < 1) ? 1 : n;
}

static void adsr_set(DelugeAdsr *e, float attack, float decay, float sustain, float release)
{
	e->a_n = adsr_seg_len(attack);
	e->d_n = adsr_seg_len(decay);
	e->r_n = adsr_seg_len(release);
	e->a_inv = (e->a_n > 1) ? 1.0f / (float)(e->a_n - 1) : 0.0f;
	e->d_inv = (e->d_n > 1) ? 1.0f / (float)(e->d_n - 1) : 0.0f;
	e->r_inv = (e->r_n > 1) ? 1.0f / (float)(e->r_n - 1) : 0.0f;
	e->sustain = sustain;
	e->idx = 0;
}

static int adsr_total(const DelugeAdsr *e)
{
	return e->a_n + e->d_n + e->r_n;
}

static float adsr_ramp(int i, int seg, float seg_inv)
{
	return (seg > 1) ? ((float)i * seg_inv) : 1.0f;
}

static float adsr_next(DelugeAdsr *e)
{
	int i = e->idx;
	float v;

	if (i < e->a_n) {
		v = (1.0f - drum_fast_expf(-ADSR_CURVE * adsr_ramp(i, e->a_n, e->a_inv))) * ADSR_ATT_NORM;
	} else if (i < e->a_n + e->d_n) {
		float u = adsr_ramp(i - e->a_n, e->d_n, e->d_inv);
		v = e->sustain + (1.0f - e->sustain) * drum_fast_expf(-ADSR_CURVE * u);
	} else if (i < adsr_total(e)) {
		float u = adsr_ramp(i - e->a_n - e->d_n, e->r_n, e->r_inv);
		v = e->sustain * drum_fast_expf(-ADSR_CURVE * u);
	} else {
		return 0.0f;
	}

	e->idx++;
	return v;
}

/* ---- soft clip ----------------------------------------------------------- */

static float fast_tanh(float x)
{
	if (x < -3.0f) return -1.0f;
	if (x >  3.0f) return  1.0f;
	float x2 = x * x;
	return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

/* ---- voice --------------------------------------------------------------- */

#define DELUGE_COWBELL_CARRIER_HZ 439.99064f   /* semitones_to_freq(21) */
#define DELUGE_COWBELL_MOD1_HZ    277.17673f   /* semitones_to_freq(13) */
#define DELUGE_COWBELL_MOD2_HZ    439.99064f   /* semitones_to_freq(21) */
#define DELUGE_COWBELL_MOD2_AMT   5.875f
#define DELUGE_COWBELL_DRIVE      1.3f
#define DELUGE_COWBELL_OUT_GAIN   0.82f

#define DELUGE_COWBELL_DECAY_MIN  0.03f
#define DELUGE_COWBELL_DECAY_MAX  0.5f

typedef struct {
	DrumSharedFilter filt;
	DelugeAdsr env;

	float decay;
	float mod1_amt;   /* modulator1 -> carrier index, from set_other */

	float carrier_phase, mod1_phase, mod2_phase;
	float carrier_hz, mod1_hz, mod2_hz;

	int n_samples;
	int sample_idx;
} DelugeCowbellState;

static void cowbell_trigger(void *state_v, float pitch)
{
	DelugeCowbellState *st = (DelugeCowbellState *)state_v;

	float r = powf(2.0f, pitch / 12.0f);
	st->carrier_hz = DELUGE_COWBELL_CARRIER_HZ * r;
	st->mod1_hz    = DELUGE_COWBELL_MOD1_HZ * r;
	st->mod2_hz    = DELUGE_COWBELL_MOD2_HZ * r;
	st->carrier_phase = 0.0f;
	st->mod1_phase    = 0.0f;
	st->mod2_phase    = 0.0f;

	adsr_set(&st->env, 0.0003f, st->decay, 0.0f, st->decay * 2.0f);

	st->n_samples  = adsr_total(&st->env);
	st->sample_idx = 0;
}

static void cowbell_render(void *state_v, float *out, int n)
{
	DelugeCowbellState *st = (DelugeCowbellState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	const float rad_to_turns = 1.0f / (2.0f * (float)M_PI);
	const float mod2_amt = DELUGE_COWBELL_MOD2_AMT * rad_to_turns;
	const float mod1_amt = st->mod1_amt * rad_to_turns;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		st->carrier_phase += st->carrier_hz * inv_sr;
		if (st->carrier_phase >= 1.0f) st->carrier_phase -= 1.0f;
		st->mod1_phase += st->mod1_hz * inv_sr;
		if (st->mod1_phase >= 1.0f) st->mod1_phase -= 1.0f;
		st->mod2_phase += st->mod2_hz * inv_sr;
		if (st->mod2_phase >= 1.0f) st->mod2_phase -= 1.0f;

		float mod2 = drum_fast_sin_turns(st->mod2_phase);
		float mod1 = drum_fast_sin_turns(st->mod1_phase + mod2_amt * mod2);
		float carrier = drum_fast_sin_turns(st->carrier_phase + mod1_amt * mod1);

		float sig = carrier * adsr_next(&st->env);
		out[i] = fast_tanh(sig * DELUGE_COWBELL_DRIVE) * (1.0f / DELUGE_COWBELL_DRIVE) *
			DELUGE_COWBELL_OUT_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void cowbell_set_filter(void *state_v, float cutoff01)
{
	DelugeCowbellState *st = (DelugeCowbellState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void cowbell_set_decay(void *state_v, float decay01)
{
	DelugeCowbellState *st = (DelugeCowbellState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = DELUGE_COWBELL_DECAY_MIN + decay01 * (DELUGE_COWBELL_DECAY_MAX - DELUGE_COWBELL_DECAY_MIN);
}

/* "other" -> the modulator1 -> carrier FM index: this voice has no
 * filter at all, so the inharmonic sideband spread the index produces is
 * the only thing that moves it between a hollow sine bonk and a clangy
 * metallic bell. */
static void cowbell_set_other(void *state_v, float other01)
{
	DelugeCowbellState *st = (DelugeCowbellState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->mod1_amt = other01 * 10.0f;
}

static void cowbell_init(void *state_v)
{
	DelugeCowbellState *st = (DelugeCowbellState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->decay    = 0.12f;
	st->mod1_amt = 4.8f;   /* python's derived default */
}

const DrumVoiceOps drum_voice_deluge_cowbell = {
	.init       = cowbell_init,
	.trigger    = cowbell_trigger,
	.render     = cowbell_render,
	.set_filter = cowbell_set_filter,
	.set_decay  = cowbell_set_decay,
	.set_other  = cowbell_set_other,
	.state_size = sizeof(DelugeCowbellState),
};

/* TODO: render_cymbal() (detuned saws + noise, long decay, wide HPF)
 * from the same python reference is not ported yet -- it is the least
 * critical member of this family and needs a longer-tail envelope
 * strategy than the other four. */
