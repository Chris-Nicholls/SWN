/*
 * drum_voice_mpump_crash.c
 *
 * Port of synth_crash() from drum_prototype/render_mpump_style_drums.py:
 * a short noise transient, a long differentiated-noise wash, and five
 * inharmonic sine partials whose decay rates increase with partial
 * index, so the spectrum darkens as the hit rings out.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

#define CRASH_NUM_PARTIALS 5

static const float kCrashFreqs[CRASH_NUM_PARTIALS] = { 3200.0f, 5000.0f, 6800.0f, 8500.0f, 11000.0f };
static const float kCrashAmps[CRASH_NUM_PARTIALS]  = { 0.08f, 0.10f, 0.08f, 0.06f, 0.04f };

#define CRASH_N_SECONDS   1.0f
#define CRASH_MAX_SECONDS 3.0f

#define CRASH_DECAY_MIN 0.5f
#define CRASH_DECAY_MAX 3.0f

/* Stands in for the python reference's peak normalisation. */
#define CRASH_GAIN 0.65f

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
	Xorshift32 rng;

	float pitch;
	float color;
	float decay;
	float freq_scale;

	float phase[CRASH_NUM_PARTIALS];
	float prev_noise;
	int   n_samples;
	int   sample_idx;
} CrashState;

static void crash_update_scale(CrashState *st)
{
	st->freq_scale = powf(2.0f, st->pitch / 12.0f) * powf(2.0f, st->color * 0.5f);
}

static void crash_trigger(void *state_v, float pitch)
{
	CrashState *st = (CrashState *)state_v;

	st->pitch = pitch;
	crash_update_scale(st);

	float n_seconds = CRASH_N_SECONDS * st->decay;
	if (n_seconds > CRASH_MAX_SECONDS) n_seconds = CRASH_MAX_SECONDS;
	st->n_samples  = (int)(n_seconds * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
	st->prev_noise = 0.0f;
	for (int p = 0; p < CRASH_NUM_PARTIALS; p++)
		st->phase[p] = 0.0f;
}

static void crash_render(void *state_v, float *out, int n)
{
	CrashState *st = (CrashState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;

	/* the bank lives in locals for the block: out[] may alias the state,
	 * so leaving the phases in the struct forces a reload per partial. */
	float inc[CRASH_NUM_PARTIALS];
	float ph[CRASH_NUM_PARTIALS];
	for (int p = 0; p < CRASH_NUM_PARTIALS; p++) {
		inc[p] = kCrashFreqs[p] * st->freq_scale * inv_sr;
		ph[p]  = st->phase[p];
	}

	const float inv_decay = 1.0f / st->decay;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;

		float raw = xorshift_uniform(&st->rng);
		float diff = raw - st->prev_noise;
		st->prev_noise = raw;

		float transient = drum_fast_expf(-t * 300.0f) * 0.35f;
		float noise = diff * drum_fast_expf(-t * (3.0f * inv_decay)) * 0.40f;

		float ring = 0.0f;
		for (int p = 0; p < CRASH_NUM_PARTIALS; p++) {
			float x = ph[p];
			float nx = x + inc[p];
			ph[p] = nx - (float)(int)nx;
			ring += drum_fast_sin_turns(x) * kCrashAmps[p] *
				drum_fast_expf(-t * ((3.0f + (float)p) * inv_decay));
		}

		out[i] = (transient * raw + noise + ring) * CRASH_GAIN;
		st->sample_idx++;
	}

	for (int p = 0; p < CRASH_NUM_PARTIALS; p++)
		st->phase[p] = ph[p];

	drum_shared_filter_process(&st->filt, out, n);
}

static void crash_set_filter(void *state_v, float cutoff01)
{
	CrashState *st = (CrashState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void crash_set_decay(void *state_v, float decay01)
{
	CrashState *st = (CrashState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = CRASH_DECAY_MIN + decay01 * (CRASH_DECAY_MAX - CRASH_DECAY_MIN);
}

/* "other" -> color: cymbal size is heard as where the partial comb
 * sits, so shifting the whole bank is what turns this from a small
 * splash into a big ride-sized crash. 0..1 maps to python's -1..+1. */
static void crash_set_other(void *state_v, float other01)
{
	CrashState *st = (CrashState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->color = other01 * 2.0f - 1.0f;
	crash_update_scale(st);
}

static void crash_init(void *state_v)
{
	CrashState *st = (CrashState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->rng.s = 0xB5297A4Du;   /* nonzero xorshift seed */
	st->decay = 1.0f;
	st->color = 0.0f;
	crash_update_scale(st);
}

const DrumVoiceOps drum_voice_mpump_crash = {
	.init       = crash_init,
	.trigger    = crash_trigger,
	.render     = crash_render,
	.set_filter = crash_set_filter,
	.set_decay  = crash_set_decay,
	.set_other  = crash_set_other,
	.state_size = sizeof(CrashState),
};
