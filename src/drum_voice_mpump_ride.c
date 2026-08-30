/*
 * drum_voice_mpump_ride.c
 *
 * Port of synth_ride() from drum_prototype/render_mpump_style_drums.py:
 * a very fast noise "stick" attack, a differentiated-noise wash, and a
 * six-partial comb that reaches down to a low 392Hz bell partial --
 * that low partial is what separates the ride from the crash.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

#define RIDE_NUM_PARTIALS 6

static const float kRideFreqs[RIDE_NUM_PARTIALS] = { 392.0f, 1200.0f, 2800.0f, 4600.0f, 6200.0f, 8500.0f };
static const float kRideAmps[RIDE_NUM_PARTIALS]  = { 0.04f, 0.05f, 0.05f, 0.04f, 0.03f, 0.02f };

#define RIDE_N_SECONDS   0.6f
#define RIDE_MAX_SECONDS 3.0f

#define RIDE_DECAY_MIN 0.5f
#define RIDE_DECAY_MAX 3.0f

/* Stands in for the python reference's peak normalisation. */
#define RIDE_GAIN 0.75f

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

	float phase[RIDE_NUM_PARTIALS];
	float prev_noise;
	int   n_samples;
	int   sample_idx;
} RideState;

static void ride_update_scale(RideState *st)
{
	st->freq_scale = powf(2.0f, st->pitch / 12.0f) * powf(2.0f, st->color * 0.5f);
}

static void ride_trigger(void *state_v, float pitch)
{
	RideState *st = (RideState *)state_v;

	st->pitch = pitch;
	ride_update_scale(st);

	float n_seconds = RIDE_N_SECONDS * st->decay;
	if (n_seconds > RIDE_MAX_SECONDS) n_seconds = RIDE_MAX_SECONDS;
	st->n_samples  = (int)(n_seconds * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
	st->prev_noise = 0.0f;
	for (int p = 0; p < RIDE_NUM_PARTIALS; p++)
		st->phase[p] = 0.0f;
}

static void ride_render(void *state_v, float *out, int n)
{
	RideState *st = (RideState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;

	/* the bank lives in locals for the block: out[] may alias the state,
	 * so leaving the phases in the struct forces a reload per partial. */
	float inc[RIDE_NUM_PARTIALS];
	float ph[RIDE_NUM_PARTIALS];
	for (int p = 0; p < RIDE_NUM_PARTIALS; p++) {
		inc[p] = kRideFreqs[p] * st->freq_scale * inv_sr;
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

		float stick = drum_fast_expf(-t * 400.0f) * 0.18f;
		float noise = diff * 0.35f * drum_fast_expf(-t * (5.0f * inv_decay));

		float ring = 0.0f;
		for (int p = 0; p < RIDE_NUM_PARTIALS; p++) {
			float x = ph[p];
			float nx = x + inc[p];
			ph[p] = nx - (float)(int)nx;
			ring += drum_fast_sin_turns(x) * kRideAmps[p];
		}
		ring *= drum_fast_expf(-t * (8.0f * inv_decay));

		out[i] = (stick * raw + noise + ring) * RIDE_GAIN;
		st->sample_idx++;
	}

	for (int p = 0; p < RIDE_NUM_PARTIALS; p++)
		st->phase[p] = ph[p];

	drum_shared_filter_process(&st->filt, out, n);
}

static void ride_set_filter(void *state_v, float cutoff01)
{
	RideState *st = (RideState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void ride_set_decay(void *state_v, float decay01)
{
	RideState *st = (RideState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = RIDE_DECAY_MIN + decay01 * (RIDE_DECAY_MAX - RIDE_DECAY_MIN);
}

/* "other" -> color: shifting the partial comb moves the bell partial
 * with it, which is exactly the "ping vs wash" character a ride is
 * played for. 0..1 maps to python's -1..+1. */
static void ride_set_other(void *state_v, float other01)
{
	RideState *st = (RideState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->color = other01 * 2.0f - 1.0f;
	ride_update_scale(st);
}

static void ride_init(void *state_v)
{
	RideState *st = (RideState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->rng.s = 0x68E31DA4u;   /* nonzero xorshift seed */
	st->decay = 1.0f;
	st->color = 0.0f;
	ride_update_scale(st);
}

const DrumVoiceOps drum_voice_mpump_ride = {
	.init       = ride_init,
	.trigger    = ride_trigger,
	.render     = ride_render,
	.set_filter = ride_set_filter,
	.set_decay  = ride_set_decay,
	.set_other  = ride_set_other,
	.state_size = sizeof(RideState),
};
