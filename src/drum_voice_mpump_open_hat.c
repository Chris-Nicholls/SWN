/*
 * drum_voice_mpump_open_hat.c
 *
 * Port of synth_open_hat() / _hat_like() from
 * drum_prototype/render_mpump_style_drums.py. Same structure as the
 * closed hat (noise transient + differentiated noise + inharmonic
 * partial bank) with louder partials and far slower noise/ring decays,
 * which is what makes it read as "open" rather than "choked".
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

#define OHAT_NUM_PARTIALS 6

static const float kOhatFreqs[OHAT_NUM_PARTIALS] = { 3500.0f, 5200.0f, 7500.0f, 4100.0f, 6300.0f, 8800.0f };
static const float kOhatAmps[OHAT_NUM_PARTIALS]  = { 0.10f, 0.07f, 0.12f, 0.07f, 0.09f, 0.04f };

#define OHAT_TRANSIENT_RATE 600.0f
#define OHAT_TRANSIENT_AMP  0.18f
#define OHAT_NOISE_RATE     6.0f
#define OHAT_NOISE_AMP      0.35f
#define OHAT_RING_RATE      5.0f
#define OHAT_N_SECONDS      0.3f
#define OHAT_MAX_SECONDS    2.0f

#define OHAT_DECAY_MIN 0.3f
#define OHAT_DECAY_MAX 3.0f

/* Compensates for the python reference's post-hoc peak normalisation,
 * which the firmware has no chance to do; see closed hat. */
#define OHAT_GAIN 0.70f

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

	float phase[OHAT_NUM_PARTIALS];
	float prev_noise;
	int   n_samples;
	int   sample_idx;
} OpenHatState;

static void ohat_update_scale(OpenHatState *st)
{
	st->freq_scale = powf(2.0f, st->pitch / 12.0f) * powf(2.0f, st->color * 0.5f);
}

static void ohat_trigger(void *state_v, float pitch)
{
	OpenHatState *st = (OpenHatState *)state_v;

	st->pitch = pitch;
	ohat_update_scale(st);

	float n_seconds = OHAT_N_SECONDS * st->decay;
	if (n_seconds > OHAT_MAX_SECONDS) n_seconds = OHAT_MAX_SECONDS;
	st->n_samples  = (int)(n_seconds * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
	st->prev_noise = 0.0f;
	for (int p = 0; p < OHAT_NUM_PARTIALS; p++)
		st->phase[p] = 0.0f;
}

static void ohat_render(void *state_v, float *out, int n)
{
	OpenHatState *st = (OpenHatState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;

	/* the bank lives in locals for the block: out[] may alias the state,
	 * so leaving the phases in the struct forces a reload per partial. */
	float inc[OHAT_NUM_PARTIALS];
	float ph[OHAT_NUM_PARTIALS];
	for (int p = 0; p < OHAT_NUM_PARTIALS; p++) {
		inc[p] = kOhatFreqs[p] * st->freq_scale * inv_sr;
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

		float transient = drum_fast_expf(-t * OHAT_TRANSIENT_RATE) * OHAT_TRANSIENT_AMP;
		float noise = diff * drum_fast_expf(-t * (OHAT_NOISE_RATE * inv_decay)) * OHAT_NOISE_AMP;

		float ring = 0.0f;
		for (int p = 0; p < OHAT_NUM_PARTIALS; p++) {
			float x = ph[p];
			float nx = x + inc[p];
			ph[p] = nx - (float)(int)nx;
			ring += drum_fast_sin_turns(x) * kOhatAmps[p];
		}
		ring *= drum_fast_expf(-t * (OHAT_RING_RATE * inv_decay));

		out[i] = (transient * raw + noise + ring) * OHAT_GAIN;
		st->sample_idx++;
	}

	for (int p = 0; p < OHAT_NUM_PARTIALS; p++)
		st->phase[p] = ph[p];

	drum_shared_filter_process(&st->filt, out, n);
}

static void ohat_set_filter(void *state_v, float cutoff01)
{
	OpenHatState *st = (OpenHatState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void ohat_set_decay(void *state_v, float decay01)
{
	OpenHatState *st = (OpenHatState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = OHAT_DECAY_MIN + decay01 * (OHAT_DECAY_MAX - OHAT_DECAY_MIN);
}

/* "other" -> color, for the same reason as the closed hat: the partial
 * bank's placement is the only timbral axis _hat_like() exposes, and
 * sweeping it moves the pair convincingly from "small tight" to "big
 * splashy" without touching decay. */
static void ohat_set_other(void *state_v, float other01)
{
	OpenHatState *st = (OpenHatState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->color = other01 * 2.0f - 1.0f;
	ohat_update_scale(st);
}

static void ohat_init(void *state_v)
{
	OpenHatState *st = (OpenHatState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->rng.s = 0x7F4A7C15u;   /* nonzero xorshift seed */
	st->decay = 1.0f;
	st->color = 0.0f;
	ohat_update_scale(st);
}

const DrumVoiceOps drum_voice_mpump_open_hat = {
	.init       = ohat_init,
	.trigger    = ohat_trigger,
	.render     = ohat_render,
	.set_filter = ohat_set_filter,
	.set_decay  = ohat_set_decay,
	.set_other  = ohat_set_other,
	.state_size = sizeof(OpenHatState),
};
