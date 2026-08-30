/*
 * drum_voice_chip_closed_hat.c
 *
 * Port of chip_closed_hat() from drum_prototype/render_chiptune_drums.py:
 * metallic-mode (bit-6 tap, 93-step) LFSR noise at table period 2 under
 * the fastest 16-step staircase envelope (period 0), gated at 50ms.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_voice.h"
#include "drum_voice_chip_lfsr.h"
#include "drum_shared_filter.h"

#define CHIP_CHAT_PERIOD_IDX 2
#define CHIP_CHAT_ENV_PER    0.0f
#define CHIP_CHAT_GATE_S     0.05f
#define CHIP_CHAT_GAIN       0.5f

#define CHIP_CHAT_DECAY_MIN  0.4f
#define CHIP_CHAT_DECAY_MAX  4.0f

typedef struct {
	DrumSharedFilter filt;
	ChipLfsr noise;
	ChipEnv env;

	float decay_scale;
	float noise_ticks;
	float pitch_ratio;

	int n_samples;
	int sample_idx;
} ChipChatState;

static void chip_chat_trigger(void *state_v, float pitch)
{
	ChipChatState *st = (ChipChatState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	chip_lfsr_set_period(&st->noise, st->noise_ticks / st->pitch_ratio);
	chip_lfsr_reset(&st->noise, 1u);

	chip_env_set(&st->env, CHIP_CHAT_ENV_PER, st->decay_scale);
	chip_env_reset(&st->env);

	st->n_samples  = (int)(CHIP_CHAT_GATE_S * st->decay_scale * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void chip_chat_render(void *state_v, float *out, int n)
{
	ChipChatState *st = (ChipChatState *)state_v;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}
		out[i] = chip_lfsr_next(&st->noise) * chip_env_next(&st->env) * CHIP_CHAT_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void chip_chat_set_filter(void *state_v, float cutoff01)
{
	ChipChatState *st = (ChipChatState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void chip_chat_set_decay(void *state_v, float decay01)
{
	ChipChatState *st = (ChipChatState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay_scale = CHIP_CHAT_DECAY_MIN + decay01 * (CHIP_CHAT_DECAY_MAX - CHIP_CHAT_DECAY_MIN);
}

/* "other" -> noise-period table index: with only a 50ms window the tap
 * mode is barely audible, whereas stepping the period table walks the
 * hat from a bright tick down through the chip's characteristic
 * "buzzier and lower" grades. */
static void chip_chat_set_other(void *state_v, float other01)
{
	ChipChatState *st = (ChipChatState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->noise_ticks = chip_noise_period(chip_noise_period_index(other01));
	chip_lfsr_set_period(&st->noise, st->noise_ticks / st->pitch_ratio);
}

static void chip_chat_init(void *state_v)
{
	ChipChatState *st = (ChipChatState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->decay_scale = 1.0f;
	st->pitch_ratio = 1.0f;
	st->noise_ticks = chip_noise_period(CHIP_CHAT_PERIOD_IDX);
	chip_lfsr_init(&st->noise, 1u, 1u, st->noise_ticks);
	chip_env_set(&st->env, CHIP_CHAT_ENV_PER, 1.0f);
	chip_env_reset(&st->env);
}

const DrumVoiceOps drum_voice_chip_closed_hat = {
	.init       = chip_chat_init,
	.trigger    = chip_chat_trigger,
	.render     = chip_chat_render,
	.set_filter = chip_chat_set_filter,
	.set_decay  = chip_chat_set_decay,
	.set_other  = chip_chat_set_other,
	.state_size = sizeof(ChipChatState),
};
