/*
 * drum_voice_mpump_rimshot.c
 *
 * Port of synth_rimshot() from
 * drum_prototype/render_mpump_style_drums.py: two short sine tones plus
 * a splash of raw noise under one very fast shared exponential -- the
 * whole hit is a handful of milliseconds long.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

#define RIM_N_SECONDS   0.04f
#define RIM_MAX_SECONDS 1.0f

/* Short even at the top of the range: a rimshot that rings is a
 * woodblock, so the max stays well under the other voices'. */
#define RIM_DECAY_MIN 0.3f
#define RIM_DECAY_MAX 3.0f

#define RIM_TUNE_RANGE_SEMI 12.0f

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

	float pitch;         /* semitones from trigger() */
	float tune;          /* semitones from set_other() */
	float decay;
	float pitch_ratio;

	int n_samples;
	int sample_idx;
} RimshotState;

static void rim_update_ratio(RimshotState *st)
{
	st->pitch_ratio = powf(2.0f, (st->pitch + st->tune) / 12.0f);
}

static void rim_trigger(void *state_v, float pitch)
{
	RimshotState *st = (RimshotState *)state_v;

	st->pitch = pitch;
	rim_update_ratio(st);

	float n_seconds = RIM_N_SECONDS * st->decay;
	if (n_seconds > RIM_MAX_SECONDS) n_seconds = RIM_MAX_SECONDS;
	st->n_samples  = (int)(n_seconds * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void rim_render(void *state_v, float *out, int n)
{
	RimshotState *st = (RimshotState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	const float inv_decay = 1.0f / st->decay;
	float r = st->pitch_ratio;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;

		float tone1 = drum_fast_sin_turns(920.0f * r * t) * 0.3f;
		float tone2 = drum_fast_sin_turns(1600.0f * r * t) * 0.2f *
			drum_fast_expf(-t * (100.0f * inv_decay));
		float noise = xorshift_uniform(&st->rng) * 0.15f;

		out[i] = (tone1 + tone2 + noise) * drum_fast_expf(-t * (80.0f * inv_decay));
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void rim_set_filter(void *state_v, float cutoff01)
{
	RimshotState *st = (RimshotState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void rim_set_decay(void *state_v, float decay01)
{
	RimshotState *st = (RimshotState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = RIM_DECAY_MIN + decay01 * (RIM_DECAY_MAX - RIM_DECAY_MIN);
}

/* "other" -> tune: synth_rimshot() only exposes tune and decay, and
 * pitch is what distinguishes a wooden rim click from a metallic one.
 * 0..1 maps to +/-1 octave around the nominal 920/1600Hz pair. */
static void rim_set_other(void *state_v, float other01)
{
	RimshotState *st = (RimshotState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->tune = (other01 * 2.0f - 1.0f) * RIM_TUNE_RANGE_SEMI;
	rim_update_ratio(st);
}

static void rim_init(void *state_v)
{
	RimshotState *st = (RimshotState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->rng.s = 0x1F123BB5u;   /* nonzero xorshift seed */
	st->decay = 1.0f;
	st->tune  = 0.0f;
	rim_update_ratio(st);
}

const DrumVoiceOps drum_voice_mpump_rimshot = {
	.init       = rim_init,
	.trigger    = rim_trigger,
	.render     = rim_render,
	.set_filter = rim_set_filter,
	.set_decay  = rim_set_decay,
	.set_other  = rim_set_other,
	.state_size = sizeof(RimshotState),
};
