/*
 * drum_voice_mpump_clap.c
 *
 * Port of synth_clap() from drum_prototype/render_mpump_style_drums.py:
 * four overlapping exponentially-decaying noise bursts at fixed,
 * slightly irregular offsets (the "several hands, not quite together"
 * trick), blended with a resonant bandpass copy of themselves.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

#define CLAP_NUM_BURSTS 4

/* The python reference jitters these from a seeded RNG; frozen here to
 * the values that seed produces, so every hit is identical (a clap
 * whose burst spacing wobbles per trigger reads as a flam, not a clap). */
static const float kClapOffsets[CLAP_NUM_BURSTS] = { 0.0f, 0.011150f, 0.023002f, 0.032740f };

#define CLAP_N_SECONDS   0.25f
#define CLAP_MAX_SECONDS 2.0f

#define CLAP_DECAY_MIN 0.3f
#define CLAP_DECAY_MAX 3.0f

#define CLAP_BP_FREQ 3200.0f
#define CLAP_BP_Q    3.0f

/* Four overlapping bursts can sum well past unity; stands in for the
 * python reference's peak normalisation. */
#define CLAP_GAIN 0.60f

typedef struct {
	float b0, a0, a1, a2;
	float x1, x2, y1, y2;
} Biquad;

static void biquad_set_bandpass(Biquad *bq, float freq, float q, float sr)
{
	float w0 = 2.0f * (float)M_PI * freq / sr;
	float alpha = sinf(w0) / (2.0f * q);
	bq->b0 = alpha;
	bq->a0 = 1.0f + alpha;
	bq->a1 = -2.0f * cosf(w0);
	bq->a2 = 1.0f - alpha;
}

static float biquad_process1(Biquad *bq, float x)
{
	float y = (bq->b0 * x - bq->b0 * bq->x2 - bq->a1 * bq->y1 - bq->a2 * bq->y2) / bq->a0;
	bq->x2 = bq->x1; bq->x1 = x;
	bq->y2 = bq->y1; bq->y1 = y;
	return y;
}

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

typedef struct {
	DrumSharedFilter filt;
	Biquad body_bp;
	Xorshift32 rng;

	float pitch_ratio;
	float decay;
	float noise_mix;   /* 0 = all bandpassed, 1 = all raw broadband */

	int n_samples;
	int sample_idx;
} ClapState;

static void clap_trigger(void *state_v, float pitch)
{
	ClapState *st = (ClapState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	biquad_set_bandpass(&st->body_bp, CLAP_BP_FREQ * st->pitch_ratio, CLAP_BP_Q,
	                    DRUM_VOICE_SAMPLE_RATE);
	st->body_bp.x1 = st->body_bp.x2 = st->body_bp.y1 = st->body_bp.y2 = 0.0f;

	float n_seconds = CLAP_N_SECONDS * st->decay;
	if (n_seconds > CLAP_MAX_SECONDS) n_seconds = CLAP_MAX_SECONDS;
	st->n_samples  = (int)(n_seconds * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void clap_render(void *state_v, float *out, int n)
{
	ClapState *st = (ClapState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	float burst_rate = 35.0f / st->decay;
	float raw_level = st->noise_mix;
	float bp_level  = 1.0f - st->noise_mix;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;

		float bursts = 0.0f;
		for (int b = 0; b < CLAP_NUM_BURSTS; b++) {
			float bt = t - kClapOffsets[b];
			if (bt >= 0.0f)
				bursts += drum_fast_expf(-bt * burst_rate) * 0.5f;
		}

		float raw = xorshift_uniform(&st->rng) * bursts;
		float shaped = biquad_process1(&st->body_bp, raw);

		out[i] = (raw * raw_level + shaped * bp_level) * CLAP_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void clap_set_filter(void *state_v, float cutoff01)
{
	ClapState *st = (ClapState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void clap_set_decay(void *state_v, float decay01)
{
	ClapState *st = (ClapState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = CLAP_DECAY_MIN + decay01 * (CLAP_DECAY_MAX - CLAP_DECAY_MIN);
}

/* "other" -> noise_mix: synth_clap() is entirely noise, so its one
 * timbral axis is how much of it stays broadband versus how much goes
 * through the 3.2kHz "room" bandpass -- i.e. dry hand-slap at 1.0
 * versus tight resonant 909 clap at 0.0. */
static void clap_set_other(void *state_v, float other01)
{
	ClapState *st = (ClapState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->noise_mix = other01;
}

static void clap_init(void *state_v)
{
	ClapState *st = (ClapState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->rng.s = 0xC2B2AE35u;   /* nonzero xorshift seed */
	st->decay = 1.0f;
	st->noise_mix = 0.5f;
}

const DrumVoiceOps drum_voice_mpump_clap = {
	.init       = clap_init,
	.trigger    = clap_trigger,
	.render     = clap_render,
	.set_filter = clap_set_filter,
	.set_decay  = clap_set_decay,
	.set_other  = clap_set_other,
	.state_size = sizeof(ClapState),
};
