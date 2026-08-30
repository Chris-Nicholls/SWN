/*
 * drum_voice_mpump_closed_hat.c
 *
 * Port of synth_closed_hat() / _hat_like() from
 * drum_prototype/render_mpump_style_drums.py: a fast noise-burst
 * transient, a differentiated (high-passed) noise layer, and a bank of
 * six inharmonic sine partials, each with its own exponential decay.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

#define CHAT_NUM_PARTIALS 6

static const float kChatFreqs[CHAT_NUM_PARTIALS] = { 3500.0f, 5200.0f, 7500.0f, 4100.0f, 6300.0f, 8800.0f };
static const float kChatAmps[CHAT_NUM_PARTIALS]  = { 0.06f, 0.04f, 0.08f, 0.04f, 0.06f, 0.02f };

#define CHAT_TRANSIENT_RATE 1000.0f
#define CHAT_TRANSIENT_AMP  0.25f
#define CHAT_NOISE_RATE     50.0f
#define CHAT_NOISE_AMP      0.45f
#define CHAT_RING_RATE      120.0f
#define CHAT_N_SECONDS      0.08f
#define CHAT_MAX_SECONDS    1.0f

#define CHAT_DECAY_MIN 0.3f
#define CHAT_DECAY_MAX 3.0f

/* The python reference normalises every render to 0.9 peak; the
 * firmware cannot, so the raw sum of transient + differentiated noise +
 * partials is trimmed here to keep worst-case peaks under unity. */
#define CHAT_GAIN 0.65f

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

	float pitch;         /* semitones, as handed to trigger() */
	float color;         /* -1..1, partial-bank frequency shift */
	float decay;
	float freq_scale;    /* 2^(pitch/12) * 2^(color/2), applied to every partial */

	float phase[CHAT_NUM_PARTIALS];
	float prev_noise;    /* keeps the noise differentiator continuous across render() calls */
	int   n_samples;
	int   sample_idx;
} ClosedHatState;

static void chat_update_scale(ClosedHatState *st)
{
	st->freq_scale = powf(2.0f, st->pitch / 12.0f) * powf(2.0f, st->color * 0.5f);
}

static void chat_trigger(void *state_v, float pitch)
{
	ClosedHatState *st = (ClosedHatState *)state_v;

	st->pitch = pitch;
	chat_update_scale(st);

	float n_seconds = CHAT_N_SECONDS * st->decay;
	if (n_seconds > CHAT_MAX_SECONDS) n_seconds = CHAT_MAX_SECONDS;
	st->n_samples  = (int)(n_seconds * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
	st->prev_noise = 0.0f;
	for (int p = 0; p < CHAT_NUM_PARTIALS; p++)
		st->phase[p] = 0.0f;
}

static void chat_render(void *state_v, float *out, int n)
{
	ClosedHatState *st = (ClosedHatState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;

	/* the bank lives in locals for the block: out[] may alias the state,
	 * so leaving the phases in the struct forces a reload per partial. */
	float inc[CHAT_NUM_PARTIALS];
	float ph[CHAT_NUM_PARTIALS];
	for (int p = 0; p < CHAT_NUM_PARTIALS; p++) {
		inc[p] = kChatFreqs[p] * st->freq_scale * inv_sr;
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

		float transient = drum_fast_expf(-t * CHAT_TRANSIENT_RATE) * CHAT_TRANSIENT_AMP;
		float noise = diff * drum_fast_expf(-t * (CHAT_NOISE_RATE * inv_decay)) * CHAT_NOISE_AMP;

		float ring = 0.0f;
		for (int p = 0; p < CHAT_NUM_PARTIALS; p++) {
			float x = ph[p];
			float nx = x + inc[p];
			ph[p] = nx - (float)(int)nx;
			ring += drum_fast_sin_turns(x) * kChatAmps[p];
		}
		ring *= drum_fast_expf(-t * (CHAT_RING_RATE * inv_decay));

		out[i] = (transient * raw + noise + ring) * CHAT_GAIN;
		st->sample_idx++;
	}

	for (int p = 0; p < CHAT_NUM_PARTIALS; p++)
		st->phase[p] = ph[p];

	drum_shared_filter_process(&st->filt, out, n);
}

static void chat_set_filter(void *state_v, float cutoff01)
{
	ClosedHatState *st = (ClosedHatState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void chat_set_decay(void *state_v, float decay01)
{
	ClosedHatState *st = (ClosedHatState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = CHAT_DECAY_MIN + decay01 * (CHAT_DECAY_MAX - CHAT_DECAY_MIN);
}

/* "other" -> color: a closed hat is defined by where its inharmonic
 * partial bank sits, and colour is the only knob in _hat_like() that
 * moves it; decay is already its own control and the noise/tone balance
 * is fixed by the recipe. 0..1 maps onto the python range -1..+1. */
static void chat_set_other(void *state_v, float other01)
{
	ClosedHatState *st = (ClosedHatState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->color = other01 * 2.0f - 1.0f;
	chat_update_scale(st);
}

static void chat_init(void *state_v)
{
	ClosedHatState *st = (ClosedHatState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->rng.s = 0x2A9F1D3Bu;   /* nonzero xorshift seed */
	st->decay = 1.0f;
	st->color = 0.0f;
	chat_update_scale(st);
}

const DrumVoiceOps drum_voice_mpump_closed_hat = {
	.init       = chat_init,
	.trigger    = chat_trigger,
	.render     = chat_render,
	.set_filter = chat_set_filter,
	.set_decay  = chat_set_decay,
	.set_other  = chat_set_other,
	.state_size = sizeof(ClosedHatState),
};
