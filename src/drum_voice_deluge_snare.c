/*
 * drum_voice_deluge_snare.c
 *
 * Port of render_snare() from drum_prototype/render_deluge_drums.py:
 * a sine+square tone layer and a white-noise layer on separate
 * envelopes, summed into a 4-pole Moog-ladder lowpass and a one-pole
 * highpass that strips the low thud.
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

#define DELUGE_SNARE_SINE_HZ    146.82926f   /* semitones_to_freq(2) */
#define DELUGE_SNARE_SQ_HZ      130.81f      /* semitones_to_freq(0) */
#define DELUGE_SNARE_LADDER_HZ  6581.25f
#define DELUGE_SNARE_LADDER_RES 0.15f
#define DELUGE_SNARE_HPF_HZ     120.0f
#define DELUGE_SNARE_NOISE_FRAC 0.75f        /* noise envelope is snappier than the body, as in the python */
#define DELUGE_SNARE_OUT_GAIN   1.35f

#define DELUGE_SNARE_DECAY_MIN  0.06f
#define DELUGE_SNARE_DECAY_MAX  1.2f

typedef struct {
	DrumSharedFilter filt;
	MoogLadder ladder;
	OnePoleHpf hpf;
	DelugeAdsr body_env;
	DelugeAdsr noise_env;
	Xorshift32 rng;

	float decay;
	float noise_mix;

	float sine_phase;
	float sq_phase;
	float sine_hz;
	float sq_hz;

	int n_samples;
	int sample_idx;
} DelugeSnareState;

static void snare_trigger(void *state_v, float pitch)
{
	DelugeSnareState *st = (DelugeSnareState *)state_v;

	float r = powf(2.0f, pitch / 12.0f);
	st->sine_hz = DELUGE_SNARE_SINE_HZ * r;
	st->sq_hz   = DELUGE_SNARE_SQ_HZ * r;
	st->sine_phase = 0.0f;
	st->sq_phase   = 0.0f;

	adsr_set(&st->body_env, 0.0005f, st->decay, 0.0f, st->decay);
	adsr_set(&st->noise_env, 0.0003f, st->decay * DELUGE_SNARE_NOISE_FRAC, 0.0f,
	         st->decay * DELUGE_SNARE_NOISE_FRAC);

	moog_set(&st->ladder, DELUGE_SNARE_LADDER_HZ, DELUGE_SNARE_LADDER_RES);
	st->ladder.s[0] = st->ladder.s[1] = st->ladder.s[2] = st->ladder.s[3] = 0.0f;
	hpf_set(&st->hpf, DELUGE_SNARE_HPF_HZ);
	st->hpf.prev_x = st->hpf.prev_y = 0.0f;

	st->n_samples  = adsr_total(&st->body_env);
	st->sample_idx = 0;
}

static void snare_render(void *state_v, float *out, int n)
{
	DelugeSnareState *st = (DelugeSnareState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	/* python's fixed 0.5 tone / 1.0 noise levels sit at noise_mix = 0.5 */
	const float tone_level  = 2.0f * (1.0f - st->noise_mix) * 0.5f;
	const float noise_level = 2.0f * st->noise_mix;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		st->sine_phase += st->sine_hz * inv_sr;
		if (st->sine_phase >= 1.0f) st->sine_phase -= 1.0f;
		st->sq_phase += st->sq_hz * inv_sr;
		if (st->sq_phase >= 1.0f) st->sq_phase -= 1.0f;

		float tone = 0.6f * drum_fast_sin_turns(st->sine_phase) +
			0.4f * ((st->sq_phase < 0.5f) ? 1.0f : -1.0f);

		float nz = xorshift_uniform(&st->rng);

		float sig = tone * adsr_next(&st->body_env) * tone_level +
			nz * adsr_next(&st->noise_env) * noise_level;

		sig = moog_process1(&st->ladder, sig);
		out[i] = hpf_process1(&st->hpf, sig) * DELUGE_SNARE_OUT_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void snare_set_filter(void *state_v, float cutoff01)
{
	DelugeSnareState *st = (DelugeSnareState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void snare_set_decay(void *state_v, float decay01)
{
	DelugeSnareState *st = (DelugeSnareState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = DELUGE_SNARE_DECAY_MIN + decay01 * (DELUGE_SNARE_DECAY_MAX - DELUGE_SNARE_DECAY_MIN);
}

/* "other" -> tone/noise balance: the two layers already have separate
 * envelopes and separate spectra, so crossfading them is what walks the
 * voice from a rimshot-ish pitched crack to a pure white-noise splash --
 * far more range than any filter tweak on this topology gives. */
static void snare_set_other(void *state_v, float other01)
{
	DelugeSnareState *st = (DelugeSnareState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->noise_mix = other01;
}

static void snare_init(void *state_v)
{
	DelugeSnareState *st = (DelugeSnareState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->rng.s     = 0x2545F491u;
	st->decay     = 0.2f;
	st->noise_mix = 0.5f;
}

const DrumVoiceOps drum_voice_deluge_snare = {
	.init       = snare_init,
	.trigger    = snare_trigger,
	.render     = snare_render,
	.set_filter = snare_set_filter,
	.set_decay  = snare_set_decay,
	.set_other  = snare_set_other,
	.state_size = sizeof(DelugeSnareState),
};
