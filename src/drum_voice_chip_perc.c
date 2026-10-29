/*
 * drum_voice_chip_perc.c
 *
 * Port of chip_perc() from drum_prototype/render_chiptune_drums.py:
 * a lo-fi clap/rimshot hybrid built from four staggered 30ms bursts of
 * the same long-mode LFSR noise sequence (table period 5), each with its
 * own fastest-rate 16-step staircase envelope.
 *
 * Each burst restarts the LFSR from the same seed, exactly as the python
 * reference re-slices noise[:seg_len] per burst -- the bursts being
 * identical waveforms at different offsets is what gives the flam its
 * comb-filtered, distinctly "chip clap" colour.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_voice.h"
#include "drum_voice_chip_lfsr.h"
#include "drum_shared_filter.h"

#define CHIP_PERC_BURSTS      4
#define CHIP_PERC_PERIOD_IDX  5
#define CHIP_PERC_ENV_PER     0.0f
#define CHIP_PERC_SEG_S       0.03f
#define CHIP_PERC_BURST_AMP   0.6f
#define CHIP_PERC_GAIN        0.35f

#define CHIP_PERC_DECAY_MIN   0.4f
#define CHIP_PERC_DECAY_MAX   3.0f

/* 0 -> all four bursts stacked (a single hard rimshot tick),
 * 1 -> spread out to ~2.5x the reference spacing (a loose clap). */
#define CHIP_PERC_SPREAD_MAX  2.5f

static const float kChipPercOffsets[CHIP_PERC_BURSTS] = { 0.0f, 0.014f, 0.03f, 0.05f };

typedef struct {
	DrumSharedFilter filt;
	ChipLfsr noise[CHIP_PERC_BURSTS];
	ChipEnv env[CHIP_PERC_BURSTS];

	float decay_scale;
	float spread;
	float pitch_ratio;
	float noise_ticks;

	int start[CHIP_PERC_BURSTS];
	int seg_samples;
	int n_samples;
	int sample_idx;
} ChipPercState;

static void chip_perc_trigger(void *state_v, float pitch)
{
	ChipPercState *st = (ChipPercState *)state_v;

	st->pitch_ratio  = powf(2.0f, pitch / 12.0f);
	st->seg_samples  = (int)(CHIP_PERC_SEG_S * st->decay_scale * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx   = 0;
	st->n_samples    = 0;

	for (int b = 0; b < CHIP_PERC_BURSTS; b++) {
		chip_lfsr_init(&st->noise[b], 1u, 0u, st->noise_ticks / st->pitch_ratio);
		chip_env_set(&st->env[b], CHIP_PERC_ENV_PER, st->decay_scale);
		chip_env_reset(&st->env[b]);

		st->start[b] = (int)(kChipPercOffsets[b] * st->spread * DRUM_VOICE_SAMPLE_RATE);
		int end = st->start[b] + st->seg_samples;
		if (end > st->n_samples) st->n_samples = end;
	}
}

static void chip_perc_render(void *state_v, float *out, int n)
{
	ChipPercState *st = (ChipPercState *)state_v;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float acc = 0.0f;
		for (int b = 0; b < CHIP_PERC_BURSTS; b++) {
			int rel = st->sample_idx - st->start[b];
			if (rel < 0 || rel >= st->seg_samples)
				continue;
			acc += chip_lfsr_next(&st->noise[b]) * chip_env_next(&st->env[b]) *
				CHIP_PERC_BURST_AMP;
		}

		out[i] = acc * CHIP_PERC_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void chip_perc_set_filter(void *state_v, float cutoff01)
{
	ChipPercState *st = (ChipPercState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void chip_perc_set_decay(void *state_v, float decay01)
{
	ChipPercState *st = (ChipPercState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay_scale = CHIP_PERC_DECAY_MIN + decay01 * (CHIP_PERC_DECAY_MAX - CHIP_PERC_DECAY_MIN);
}

/* "other" -> burst spacing: the noise period and envelope are fixed by
 * the recipe, and it is the gap between the four staggered bursts that
 * morphs this voice between its two intended identities -- collapsed to
 * zero it is a single hard rimshot tick, spread out it is a loose clap. */
static void chip_perc_set_other(void *state_v, float other01)
{
	ChipPercState *st = (ChipPercState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->spread = other01 * CHIP_PERC_SPREAD_MAX;
}

static void chip_perc_init(void *state_v)
{
	ChipPercState *st = (ChipPercState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->decay_scale = 1.0f;
	st->pitch_ratio = 1.0f;
	st->spread      = 1.0f;
	st->noise_ticks = chip_noise_period(CHIP_PERC_PERIOD_IDX);

	for (int b = 0; b < CHIP_PERC_BURSTS; b++) {
		chip_lfsr_init(&st->noise[b], 1u, 0u, st->noise_ticks);
		chip_env_set(&st->env[b], CHIP_PERC_ENV_PER, 1.0f);
		chip_env_reset(&st->env[b]);
	}
}

const DrumVoiceOps drum_voice_chip_perc = {
	.init       = chip_perc_init,
	.trigger    = chip_perc_trigger,
	.render     = chip_perc_render,
	.set_filter = chip_perc_set_filter,
	.set_decay  = chip_perc_set_decay,
	.set_other  = chip_perc_set_other,
	.state_size = sizeof(ChipPercState),
};
