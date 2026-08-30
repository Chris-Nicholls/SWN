/*
 * drum_voice_chip_open_hat.c
 *
 * Port of chip_open_hat() from drum_prototype/render_chiptune_drums.py:
 * the same metallic-mode LFSR noise as the closed hat, but under a much
 * slower 16-step staircase (period 3) and gated at 350ms so all sixteen
 * amplitude steps are individually audible.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_voice.h"
#include "drum_voice_chip_lfsr.h"
#include "drum_shared_filter.h"

#define CHIP_OHAT_PERIOD_IDX 2
#define CHIP_OHAT_ENV_PER    3.0f
#define CHIP_OHAT_GATE_S     0.35f
#define CHIP_OHAT_GAIN       0.5f

#define CHIP_OHAT_DECAY_MIN  0.35f
#define CHIP_OHAT_DECAY_MAX  3.0f

typedef struct {
	DrumSharedFilter filt;
	ChipLfsr noise;
	ChipEnv env;

	float decay_scale;
	float noise_ticks;
	uint8_t metallic;
	float pitch_ratio;

	int n_samples;
	int sample_idx;
} ChipOhatState;

static void chip_ohat_trigger(void *state_v, float pitch)
{
	ChipOhatState *st = (ChipOhatState *)state_v;

	st->pitch_ratio    = powf(2.0f, pitch / 12.0f);
	st->noise.metallic = st->metallic;
	chip_lfsr_set_period(&st->noise, st->noise_ticks / st->pitch_ratio);
	chip_lfsr_reset(&st->noise, 1u);

	chip_env_set(&st->env, CHIP_OHAT_ENV_PER, st->decay_scale);
	chip_env_reset(&st->env);

	st->n_samples  = (int)(CHIP_OHAT_GATE_S * st->decay_scale * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void chip_ohat_render(void *state_v, float *out, int n)
{
	ChipOhatState *st = (ChipOhatState *)state_v;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}
		out[i] = chip_lfsr_next(&st->noise) * chip_env_next(&st->env) * CHIP_OHAT_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void chip_ohat_set_filter(void *state_v, float cutoff01)
{
	ChipOhatState *st = (ChipOhatState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void chip_ohat_set_decay(void *state_v, float decay01)
{
	ChipOhatState *st = (ChipOhatState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay_scale = CHIP_OHAT_DECAY_MIN + decay01 * (CHIP_OHAT_DECAY_MAX - CHIP_OHAT_DECAY_MIN);
}

/* "other" -> LFSR feedback tap plus period: this is the one voice whose
 * tail is long enough for the 93-step metallic loop to read as a pitched
 * buzz rather than hiss, so the lower half of the knob sweeps the period
 * table in long (32767-step) mode and the upper half repeats it metallic. */
static void chip_ohat_set_other(void *state_v, float other01)
{
	ChipOhatState *st = (ChipOhatState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;

	st->metallic = (other01 >= 0.5f) ? 1u : 0u;
	float half = st->metallic ? (other01 - 0.5f) * 2.0f : other01 * 2.0f;

	st->noise_ticks    = chip_noise_period(chip_noise_period_index(half));
	st->noise.metallic = st->metallic;
	chip_lfsr_set_period(&st->noise, st->noise_ticks / st->pitch_ratio);
}

static void chip_ohat_init(void *state_v)
{
	ChipOhatState *st = (ChipOhatState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->decay_scale = 1.0f;
	st->pitch_ratio = 1.0f;
	st->metallic    = 1u;
	st->noise_ticks = chip_noise_period(CHIP_OHAT_PERIOD_IDX);
	chip_lfsr_init(&st->noise, 1u, 1u, st->noise_ticks);
	chip_env_set(&st->env, CHIP_OHAT_ENV_PER, 1.0f);
	chip_env_reset(&st->env);
}

const DrumVoiceOps drum_voice_chip_open_hat = {
	.init       = chip_ohat_init,
	.trigger    = chip_ohat_trigger,
	.render     = chip_ohat_render,
	.set_filter = chip_ohat_set_filter,
	.set_decay  = chip_ohat_set_decay,
	.set_other  = chip_ohat_set_other,
	.state_size = sizeof(ChipOhatState),
};
