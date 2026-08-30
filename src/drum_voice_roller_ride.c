/*
 * drum_voice_roller_ride.c
 *
 * Port of synth_ride() from drum_prototype/render_roller_style_drums.py:
 * the shared six-square metal bank (see drum_roller_metal.h) voiced low
 * and long for the wash, plus a 2.9kHz sine "bow" ping on top.
 *
 * -----------------------------------------------------------------------------
 */

#include <string.h>

#include "drum_roller_metal.h"
#include "drum_shared_filter.h"
#include "drum_voice.h"

#define RIDE_DECAY_MIN 0.30f
#define RIDE_DECAY_MAX 2.50f

#define RIDE_LEVEL     0.2f
#define RIDE_BP_HZ     3600.0f
#define RIDE_HP_HZ     1400.0f
#define RIDE_RATIO     1.75f
#define RIDE_NOISE_MIX 0.22f

#define RIDE_BOW_HZ    2900.0f
#define RIDE_BOW_LVL   0.06f
#define RIDE_BOW_DEC   0.14f
#define RIDE_BOW_GATE  0.16f

#define RIDE_BRIGHT_MIN 0.60f
#define RIDE_BRIGHT_MAX 1.60f

/* Stands in for the python reference's peak normalisation. */
#define RIDE_GAIN 1.60f

typedef struct {
	DrumSharedFilter filt;
	DrumRollerMetal metal;
	RollerEnv bow_env;

	float decay;
	float bright;
	float pitch_ratio;

	float bow_phase;
	float bow_inc;

	int n_samples;
	int sample_idx;
} RollerRideState;

static void ride_rebuild(RollerRideState *st)
{
	float scale = st->bright * st->pitch_ratio;
	drum_roller_metal_config(&st->metal, RIDE_BP_HZ * scale, RIDE_HP_HZ,
	                         RIDE_RATIO * scale, RIDE_NOISE_MIX);
	drum_roller_metal_set_env(&st->metal, RIDE_LEVEL, st->decay);

	roller_env_set(&st->bow_env, 0.0f, RIDE_BOW_LVL, RIDE_BOW_DEC, 0.0006f);
	st->bow_inc = RIDE_BOW_HZ * scale / DRUM_VOICE_SAMPLE_RATE;
}

static void ride_trigger(void *state_v, float pitch)
{
	RollerRideState *st = (RollerRideState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	ride_rebuild(st);
	drum_roller_metal_reset(&st->metal);

	st->bow_phase  = 0.0f;
	st->n_samples  = (int)((st->decay + 0.08f) * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void ride_render(void *state_v, float *out, int n)
{
	RollerRideState *st = (RollerRideState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;

		st->bow_phase += st->bow_inc;
		if (st->bow_phase >= 1.0f) st->bow_phase -= 1.0f;

		float bow = 0.0f;
		if (t <= RIDE_BOW_GATE)
			bow = drum_fast_sin_turns(st->bow_phase) * roller_env_at(&st->bow_env, t);

		out[i] = (drum_roller_metal_tick(&st->metal, t) + bow) * RIDE_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void ride_set_filter(void *state_v, float cutoff01)
{
	RollerRideState *st = (RollerRideState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void ride_set_decay(void *state_v, float decay01)
{
	RollerRideState *st = (RollerRideState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = RIDE_DECAY_MIN + decay01 * (RIDE_DECAY_MAX - RIDE_DECAY_MIN);
	ride_rebuild(st);
}

/* "other" -> brightness: moves the square bank, its bandpass and the bow
 * ping together, which is heard as cymbal size -- a small bright ride at
 * one end, a dark washy 22" at the other. */
static void ride_set_other(void *state_v, float other01)
{
	RollerRideState *st = (RollerRideState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->bright = RIDE_BRIGHT_MIN + other01 * (RIDE_BRIGHT_MAX - RIDE_BRIGHT_MIN);
	ride_rebuild(st);
}

static void ride_init(void *state_v)
{
	RollerRideState *st = (RollerRideState *)state_v;
	memset(st, 0, sizeof(*st));

	drum_shared_filter_init(&st->filt);
	drum_roller_metal_init(&st->metal, 0x6C82D4F3u);

	st->decay       = 0.7f;
	st->bright      = 1.0f;
	st->pitch_ratio = 1.0f;
	ride_rebuild(st);
}

const DrumVoiceOps drum_voice_roller_ride = {
	.init       = ride_init,
	.trigger    = ride_trigger,
	.render     = ride_render,
	.set_filter = ride_set_filter,
	.set_decay  = ride_set_decay,
	.set_other  = ride_set_other,
	.state_size = sizeof(RollerRideState),
};
