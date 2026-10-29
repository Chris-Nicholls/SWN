/*
 * drum_voice_chip_cowbell.c
 *
 * Port of chip_cowbell() from drum_prototype/render_chiptune_drums.py:
 * two square pulses a fifth apart (540Hz / 810Hz) mixed with a dusting of
 * metallic-mode LFSR noise (table period 3), under a 16-step staircase
 * envelope (period 2) gated at 200ms.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_voice.h"
#include "drum_voice_chip_lfsr.h"
#include "drum_shared_filter.h"

#define CHIP_COWBELL_F1         540.0f
#define CHIP_COWBELL_RATIO      1.5f     /* 810 / 540 */
#define CHIP_COWBELL_PERIOD_IDX 3
#define CHIP_COWBELL_ENV_PER    2.0f
#define CHIP_COWBELL_GATE_S     0.2f
#define CHIP_COWBELL_TONE_MIX   0.8f
#define CHIP_COWBELL_NOISE_MIX  0.2f
#define CHIP_COWBELL_GAIN       0.8f

#define CHIP_COWBELL_DECAY_MIN  0.4f
#define CHIP_COWBELL_DECAY_MAX  3.0f

#define CHIP_COWBELL_RATIO_MIN  1.0f
#define CHIP_COWBELL_RATIO_MAX  2.0f

typedef struct {
	DrumSharedFilter filt;
	ChipLfsr noise;
	ChipEnv env;

	float decay_scale;
	float ratio;        /* second pulse's interval above the first */

	float phase1, phase2;
	float f1, f2;

	int n_samples;
	int sample_idx;
} ChipCowbellState;

static void chip_cowbell_trigger(void *state_v, float pitch)
{
	ChipCowbellState *st = (ChipCowbellState *)state_v;
	float pitch_ratio = powf(2.0f, pitch / 12.0f);

	st->f1     = CHIP_COWBELL_F1 * pitch_ratio;
	st->f2     = st->f1 * st->ratio;
	st->phase1 = 0.0f;
	st->phase2 = 0.0f;

	chip_lfsr_set_period(&st->noise, chip_noise_period(CHIP_COWBELL_PERIOD_IDX) / pitch_ratio);
	chip_lfsr_reset(&st->noise, 2u);

	chip_env_set(&st->env, CHIP_COWBELL_ENV_PER, st->decay_scale);
	chip_env_reset(&st->env);

	st->n_samples  = (int)(CHIP_COWBELL_GATE_S * st->decay_scale * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void chip_cowbell_render(void *state_v, float *out, int n)
{
	ChipCowbellState *st = (ChipCowbellState *)state_v;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float tone = 0.5f * chip_pulse_next(&st->phase1, st->f1, 0.5f) +
			0.5f * chip_pulse_next(&st->phase2, st->f2, 0.5f);
		float noise = chip_lfsr_next(&st->noise);

		out[i] = (tone * CHIP_COWBELL_TONE_MIX + noise * CHIP_COWBELL_NOISE_MIX) *
			chip_env_next(&st->env) * CHIP_COWBELL_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void chip_cowbell_set_filter(void *state_v, float cutoff01)
{
	ChipCowbellState *st = (ChipCowbellState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void chip_cowbell_set_decay(void *state_v, float decay01)
{
	ChipCowbellState *st = (ChipCowbellState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay_scale = CHIP_COWBELL_DECAY_MIN +
		decay01 * (CHIP_COWBELL_DECAY_MAX - CHIP_COWBELL_DECAY_MIN);
}

/* "other" -> the interval between the two pulses (unison up to an octave,
 * the reference's fifth at midpoint): both pulses are fixed at 50% duty
 * here, so their beating interval is what makes this read as a cowbell
 * rather than a plain square blip. */
static void chip_cowbell_set_other(void *state_v, float other01)
{
	ChipCowbellState *st = (ChipCowbellState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->ratio = CHIP_COWBELL_RATIO_MIN +
		other01 * (CHIP_COWBELL_RATIO_MAX - CHIP_COWBELL_RATIO_MIN);
	st->f2 = st->f1 * st->ratio;
}

static void chip_cowbell_init(void *state_v)
{
	ChipCowbellState *st = (ChipCowbellState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->decay_scale = 1.0f;
	st->ratio       = CHIP_COWBELL_RATIO;
	st->f1          = CHIP_COWBELL_F1;
	st->f2          = CHIP_COWBELL_F1 * CHIP_COWBELL_RATIO;
	chip_lfsr_init(&st->noise, 2u, 1u, chip_noise_period(CHIP_COWBELL_PERIOD_IDX));
	chip_env_set(&st->env, CHIP_COWBELL_ENV_PER, 1.0f);
	chip_env_reset(&st->env);
}

const DrumVoiceOps drum_voice_chip_cowbell = {
	.init       = chip_cowbell_init,
	.trigger    = chip_cowbell_trigger,
	.render     = chip_cowbell_render,
	.set_filter = chip_cowbell_set_filter,
	.set_decay  = chip_cowbell_set_decay,
	.set_other  = chip_cowbell_set_other,
	.state_size = sizeof(ChipCowbellState),
};
