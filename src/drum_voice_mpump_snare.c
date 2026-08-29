/*
 * drum_voice_mpump_snare.c
 *
 * Port of synth_snare() from drum_prototype/render_mpump_style_drums.py:
 * a tonal body (with a fast pitch-drop "snap"), a low partial, and a
 * noise layer blending raw white noise with a fixed resonant bandpass
 * ("wire" buzz). Faithfully reproduces the python formulas, including
 * its non phase-integrated body oscillator (sin(2*pi*f*pitch_env(t)*t),
 * evaluated directly from elapsed time rather than integrating
 * instantaneous frequency) since that is what gives the voice its
 * characteristic pitch-snap.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>
#include <stdint.h>

#include "drum_voice.h"
#include "drum_shared_filter.h"

/* Small stateful biquad bandpass, direct-form I, matching
 * biquad_bandpass() in the python reference exactly (same
 * coefficient formulas), used for the snare's noise "wire" layer. */
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

/* Tiny xorshift32 PRNG for the noise layer -- not trying to match the
 * python's numpy Generator bit-for-bit, just its statistics (uniform
 * white noise), since this only needs to sound right. */
typedef struct { uint32_t s; } Xorshift32;

static float xorshift_uniform(Xorshift32 *rng)
{
	uint32_t x = rng->s;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	rng->s = x;
	return ((float)(x & 0xFFFFFF) / (float)0x800000) - 1.0f;   /* ~uniform(-1,1) */
}

typedef struct {
	DrumSharedFilter filt;
	Biquad noise_bp;
	Xorshift32 rng;

	float pitch_ratio;
	float decay;
	float noise_mix;

	int n_samples;
	int sample_idx;
} SnareState;

#define SNARE_DECAY_MIN 0.2f
#define SNARE_DECAY_MAX 2.5f

static void snare_trigger(void *state_v, float pitch)
{
	SnareState *st = (SnareState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	biquad_set_bandpass(&st->noise_bp, 3800.0f * st->pitch_ratio, 3.0f, DRUM_VOICE_SAMPLE_RATE);
	st->noise_bp.x1 = st->noise_bp.x2 = st->noise_bp.y1 = st->noise_bp.y2 = 0.0f;

	float n_seconds = 0.3f * st->decay;
	if (n_seconds > 2.0f) n_seconds = 2.0f;
	st->n_samples  = (int)(n_seconds * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void snare_render(void *state_v, float *out, int n)
{
	SnareState *st = (SnareState *)state_v;
	const float sr = DRUM_VOICE_SAMPLE_RATE;
	float decay = st->decay;
	float tone_level  = 1.0f - st->noise_mix;
	float noise_level = st->noise_mix;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx / sr;

		float raw_noise = xorshift_uniform(&st->rng);
		float shaped = biquad_process1(&st->noise_bp, raw_noise);

		float pitch_env = 1.0f + 0.5f * expf(-t * 60.0f);
		float body = sinf(2.0f * (float)M_PI * 185.0f * st->pitch_ratio * pitch_env * t) *
			expf(-t * (18.0f / decay)) * tone_level;
		float low = sinf(2.0f * (float)M_PI * 110.0f * st->pitch_ratio * t) *
			expf(-t * (22.0f / decay)) * (tone_level * 0.2f);
		float noise_env = expf(-t * (14.0f / decay));
		float noise = (raw_noise * 0.45f + shaped * 0.55f) * noise_env * noise_level;

		out[i] = body + low + noise;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void snare_set_filter(void *state_v, float cutoff01)
{
	SnareState *st = (SnareState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void snare_set_decay(void *state_v, float decay01)
{
	SnareState *st = (SnareState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = SNARE_DECAY_MIN + decay01 * (SNARE_DECAY_MAX - SNARE_DECAY_MIN);
}

/* "other" -> noise_mix: the tone/noise balance is the defining
 * character control for an 808/909-style snare, and is already a
 * native 0..1 parameter in the python original. */
static void snare_set_other(void *state_v, float other01)
{
	SnareState *st = (SnareState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->noise_mix = other01;
}

static void snare_init(void *state_v)
{
	SnareState *st = (SnareState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->rng.s = 0x9E3779B9u;   /* nonzero xorshift seed */
	st->decay = 1.0f;
	st->noise_mix = 0.55f;
}

const DrumVoiceOps drum_voice_mpump_snare = {
	.init       = snare_init,
	.trigger    = snare_trigger,
	.render     = snare_render,
	.set_filter = snare_set_filter,
	.set_decay  = snare_set_decay,
	.set_other  = snare_set_other,
	.state_size = sizeof(SnareState),
};
