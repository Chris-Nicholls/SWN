/*
 * drum_voice_deluge_open_hat.c
 *
 * Port of render_open_hat() from drum_prototype/render_deluge_drums.py:
 * saw+square tone plus noise whose depth is chattered by an 8Hz square
 * LFO (itself scaled by the amp envelope, reproducing the nested
 * envelope1 -> lfo1 -> noiseVolume patch in the Deluge preset), then
 * soft-clipped, ladder-filtered and highpassed.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <stdint.h>
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

/* ---- moog ladder --------------------------------------------------------- */

static float fast_tanh(float x)
{
	if (x < -3.0f) return -1.0f;
	if (x >  3.0f) return  1.0f;
	float x2 = x * x;
	return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

typedef struct {
	float g;
	float k;
	float s[4];
} MoogLadder;

static void moog_set(MoogLadder *lp, float cutoff_hz, float resonance01)
{
	if (cutoff_hz < 20.0f) cutoff_hz = 20.0f;
	if (cutoff_hz > DRUM_VOICE_SAMPLE_RATE * 0.45f) cutoff_hz = DRUM_VOICE_SAMPLE_RATE * 0.45f;
	float g = tanf((float)M_PI * cutoff_hz / DRUM_VOICE_SAMPLE_RATE);
	lp->g = g / (1.0f + g);
	lp->k = resonance01 * 4.0f;
}

static float moog_process1(MoogLadder *lp, float x)
{
	float u = fast_tanh(x - lp->k * lp->s[3]);
	for (int stage = 0; stage < 4; stage++) {
		lp->s[stage] += lp->g * (u - lp->s[stage]);
		u = lp->s[stage];
	}
	return lp->s[3];
}

/* ---- one-pole highpass --------------------------------------------------- */

typedef struct {
	float a;
	float prev_x, prev_y;
} OnePoleHpf;

static void hpf_set(OnePoleHpf *hp, float cutoff_hz)
{
	hp->a = expf(-2.0f * (float)M_PI * cutoff_hz / DRUM_VOICE_SAMPLE_RATE);
}

static float hpf_process1(OnePoleHpf *hp, float x)
{
	float y = hp->a * (hp->prev_y + x - hp->prev_x);
	hp->prev_x = x;
	hp->prev_y = y;
	return y;
}

/* ---- noise --------------------------------------------------------------- */

typedef struct { uint32_t s; } Xorshift32;

static float xorshift_uniform(Xorshift32 *rng)
{
	uint32_t x = rng->s;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	rng->s = x;
	return ((float)(x & 0xFFFFFF) / (float)0x800000) - 1.0f;
}

/* ---- voice --------------------------------------------------------------- */

#define DELUGE_OHAT_SAW_HZ      54.99883f   /* semitones_to_freq(-15) */
#define DELUGE_OHAT_SQ_HZ       130.81f     /* semitones_to_freq(0) */
#define DELUGE_OHAT_LFO_HZ      8.0f
#define DELUGE_OHAT_DRIVE       1.5f        /* clippingAmount from the patch */
#define DELUGE_OHAT_LADDER_RES  0.09688f
#define DELUGE_OHAT_HPF_HZ      2500.0f
#define DELUGE_OHAT_LADDER_MIN  2000.0f
#define DELUGE_OHAT_LADDER_SPAN 16000.0f    /* other01 = 0.5 lands on the python's ~10.1kHz */
#define DELUGE_OHAT_OUT_GAIN    3.6f

#define DELUGE_OHAT_DECAY_MIN   0.04f
#define DELUGE_OHAT_DECAY_MAX   1.2f

typedef struct {
	DrumSharedFilter filt;
	MoogLadder ladder;
	OnePoleHpf hpf;
	DelugeAdsr amp_env;
	Xorshift32 rng;

	float decay;
	float ladder_hz;

	float saw_phase, sq_phase, lfo_phase;
	float saw_hz, sq_hz;

	int n_samples;
	int sample_idx;
} DelugeOhatState;

static void ohat_trigger(void *state_v, float pitch)
{
	DelugeOhatState *st = (DelugeOhatState *)state_v;

	float r = powf(2.0f, pitch / 12.0f);
	st->saw_hz = DELUGE_OHAT_SAW_HZ * r;
	st->sq_hz  = DELUGE_OHAT_SQ_HZ * r;
	st->saw_phase = 0.0f;
	st->sq_phase  = 0.0f;
	st->lfo_phase = 0.0f;

	adsr_set(&st->amp_env, 0.00218f, st->decay, 0.0f, st->decay * 1.3f);

	moog_set(&st->ladder, st->ladder_hz, DELUGE_OHAT_LADDER_RES);
	st->ladder.s[0] = st->ladder.s[1] = st->ladder.s[2] = st->ladder.s[3] = 0.0f;
	hpf_set(&st->hpf, DELUGE_OHAT_HPF_HZ);
	st->hpf.prev_x = st->hpf.prev_y = 0.0f;

	st->n_samples  = adsr_total(&st->amp_env);
	st->sample_idx = 0;
}

static void ohat_render(void *state_v, float *out, int n)
{
	DelugeOhatState *st = (DelugeOhatState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		st->saw_phase += st->saw_hz * inv_sr;
		if (st->saw_phase >= 1.0f) st->saw_phase -= 1.0f;
		st->sq_phase += st->sq_hz * inv_sr;
		if (st->sq_phase >= 1.0f) st->sq_phase -= 1.0f;
		st->lfo_phase += DELUGE_OHAT_LFO_HZ * inv_sr;
		if (st->lfo_phase >= 1.0f) st->lfo_phase -= 1.0f;

		float tone = 0.5f * ((2.0f * st->saw_phase - 1.0f) +
		                     ((st->sq_phase < 0.5f) ? 1.0f : -1.0f));
		float nz  = xorshift_uniform(&st->rng);
		float lfo = (st->lfo_phase < 0.5f) ? 1.0f : 0.0f;

		float env = adsr_next(&st->amp_env);
		float nz_mod = nz * (0.3f + 0.7f * lfo * env);

		float sig = (0.3f * tone + 0.9f * nz_mod) * env;
		sig = fast_tanh(sig * DELUGE_OHAT_DRIVE) * (1.0f / DELUGE_OHAT_DRIVE);
		sig = moog_process1(&st->ladder, sig);
		out[i] = hpf_process1(&st->hpf, sig) * DELUGE_OHAT_OUT_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void ohat_set_filter(void *state_v, float cutoff01)
{
	DelugeOhatState *st = (DelugeOhatState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void ohat_set_decay(void *state_v, float decay01)
{
	DelugeOhatState *st = (DelugeOhatState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = DELUGE_OHAT_DECAY_MIN + decay01 * (DELUGE_OHAT_DECAY_MAX - DELUGE_OHAT_DECAY_MIN);
}

/* "other" -> the voice's own ladder cutoff: with a fixed 2.5kHz highpass
 * downstream, sweeping the ladder squeezes/opens the surviving band and
 * is the difference between a dull "shh" and an open, splashy hat --
 * the LFO chatter depth and drive are both far less audible here. */
static void ohat_set_other(void *state_v, float other01)
{
	DelugeOhatState *st = (DelugeOhatState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->ladder_hz = DELUGE_OHAT_LADDER_MIN + other01 * DELUGE_OHAT_LADDER_SPAN;
	moog_set(&st->ladder, st->ladder_hz, DELUGE_OHAT_LADDER_RES);
}

static void ohat_init(void *state_v)
{
	DelugeOhatState *st = (DelugeOhatState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->rng.s     = 0x6C078965u;
	st->decay     = 0.25f;
	st->ladder_hz = DELUGE_OHAT_LADDER_MIN + 0.5f * DELUGE_OHAT_LADDER_SPAN;
	moog_set(&st->ladder, st->ladder_hz, DELUGE_OHAT_LADDER_RES);
}

const DrumVoiceOps drum_voice_deluge_open_hat = {
	.init       = ohat_init,
	.trigger    = ohat_trigger,
	.render     = ohat_render,
	.set_filter = ohat_set_filter,
	.set_decay  = ohat_set_decay,
	.set_other  = ohat_set_other,
	.state_size = sizeof(DelugeOhatState),
};
