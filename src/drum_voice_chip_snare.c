/*
 * drum_voice_chip_snare.c
 *
 * Port of chip_snare() from drum_prototype/render_chiptune_drums.py:
 * long-mode LFSR noise (period 64 ticks) under a 16-step staircase
 * envelope (period 1) gated at 120ms, plus a very short 180Hz square
 * "body" thump with its own faster staircase (period 0) gated at 20ms.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_voice.h"
#include "drum_voice_chip_lfsr.h"
#include "drum_shared_filter.h"

#define CHIP_SNARE_NOISE_TICKS 64.0f
#define CHIP_SNARE_ENV_PER     1.0f
#define CHIP_SNARE_GATE_S      0.12f
#define CHIP_SNARE_BODY_HZ     180.0f
#define CHIP_SNARE_BODY_ENV    0.0f
#define CHIP_SNARE_BODY_GATE_S 0.02f
#define CHIP_SNARE_NOISE_MIX   0.85f
#define CHIP_SNARE_BODY_MIX    0.3f

/* The two layers sum to 1.15 at worst, and the shortest noise periods
 * push most of their energy right into the resonant shared filter's
 * corner, so the trim needed to stay under unity is large. */
#define CHIP_SNARE_GAIN        0.5f

#define CHIP_SNARE_DECAY_MIN   0.4f
#define CHIP_SNARE_DECAY_MAX   3.0f

typedef struct {
	DrumSharedFilter filt;
	ChipLfsr noise;
	ChipEnv env;
	ChipEnv body_env;

	float decay_scale;
	float noise_ticks;   /* from set_other, before pitch scaling */
	float pitch_ratio;

	float body_phase;
	float body_hz;

	int n_samples;
	int body_samples;
	int sample_idx;
} ChipSnareState;

static void chip_snare_trigger(void *state_v, float pitch)
{
	ChipSnareState *st = (ChipSnareState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	st->body_hz     = CHIP_SNARE_BODY_HZ * st->pitch_ratio;
	st->body_phase  = 0.0f;

	/* A shorter noise period clocks the LFSR faster, i.e. pitches the
	 * noise up, so pitch divides rather than multiplies here. */
	chip_lfsr_set_period(&st->noise, st->noise_ticks / st->pitch_ratio);
	chip_lfsr_reset(&st->noise, 1u);

	chip_env_set(&st->env, CHIP_SNARE_ENV_PER, st->decay_scale);
	chip_env_reset(&st->env);
	chip_env_set(&st->body_env, CHIP_SNARE_BODY_ENV, st->decay_scale);
	chip_env_reset(&st->body_env);

	st->n_samples    = (int)(CHIP_SNARE_GATE_S * st->decay_scale * DRUM_VOICE_SAMPLE_RATE);
	st->body_samples = (int)(CHIP_SNARE_BODY_GATE_S * st->decay_scale * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx   = 0;
}

static void chip_snare_render(void *state_v, float *out, int n)
{
	ChipSnareState *st = (ChipSnareState *)state_v;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float noise = chip_lfsr_next(&st->noise) * chip_env_next(&st->env) *
			CHIP_SNARE_NOISE_MIX;

		float body = 0.0f;
		if (st->sample_idx < st->body_samples)
			body = chip_pulse_next(&st->body_phase, st->body_hz, 0.5f) *
				chip_env_next(&st->body_env) * CHIP_SNARE_BODY_MIX;

		out[i] = (noise + body) * CHIP_SNARE_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void chip_snare_set_filter(void *state_v, float cutoff01)
{
	ChipSnareState *st = (ChipSnareState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void chip_snare_set_decay(void *state_v, float decay01)
{
	ChipSnareState *st = (ChipSnareState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay_scale = CHIP_SNARE_DECAY_MIN + decay01 * (CHIP_SNARE_DECAY_MAX - CHIP_SNARE_DECAY_MIN);
}

/* "other" -> noise-period table index: the noise channel's period is the
 * only timbre control the 2A03 gives it, and it moves the snare from a
 * thin white hiss to a low grainy rattle. Deliberately stepped through
 * the 16 hardware values rather than interpolated. */
static void chip_snare_set_other(void *state_v, float other01)
{
	ChipSnareState *st = (ChipSnareState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->noise_ticks = chip_noise_period(chip_noise_period_index(other01));
	chip_lfsr_set_period(&st->noise, st->noise_ticks / st->pitch_ratio);
}

static void chip_snare_init(void *state_v)
{
	ChipSnareState *st = (ChipSnareState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->decay_scale = 1.0f;
	st->pitch_ratio = 1.0f;
	st->noise_ticks = CHIP_SNARE_NOISE_TICKS;
	chip_lfsr_init(&st->noise, 1u, 0u, CHIP_SNARE_NOISE_TICKS);
	chip_env_set(&st->env, CHIP_SNARE_ENV_PER, 1.0f);
	chip_env_reset(&st->env);
	chip_env_set(&st->body_env, CHIP_SNARE_BODY_ENV, 1.0f);
	chip_env_reset(&st->body_env);
}

const DrumVoiceOps drum_voice_chip_snare = {
	.init       = chip_snare_init,
	.trigger    = chip_snare_trigger,
	.render     = chip_snare_render,
	.set_filter = chip_snare_set_filter,
	.set_decay  = chip_snare_set_decay,
	.set_other  = chip_snare_set_other,
	.state_size = sizeof(ChipSnareState),
};
