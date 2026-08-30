/*
 * drum_voice_chip_kick.c
 *
 * Port of chip_kick() from drum_prototype/render_chiptune_drums.py:
 * a duty-cycle pulse oscillator swept exponentially from 320Hz to 48Hz
 * in 45ms, amplitude-shaped by the NES APU's 16-step linear staircase
 * envelope (period 2) and hard-gated at 220ms.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_voice_chip_lfsr.h"
#include "drum_shared_filter.h"

#define CHIP_KICK_F_START   320.0f
#define CHIP_KICK_F_END     48.0f
#define CHIP_KICK_SWEEP_S   0.045f
#define CHIP_KICK_GATE_S    0.22f
#define CHIP_KICK_ENV_PER   2.0f

/* The raw pulse peaks at exactly 1.0; the python reference normalises its
 * whole render, the firmware instead leaves headroom for the resonant
 * shared filter's overshoot (see drum_shared_filter.h). */
#define CHIP_KICK_GAIN      0.85f

/* Scales the staircase's total length (and the gate with it) without
 * changing its 16-step shape. */
#define CHIP_KICK_DECAY_MIN 0.4f
#define CHIP_KICK_DECAY_MAX 3.0f

typedef struct {
	DrumSharedFilter filt;
	ChipEnv env;

	float decay_scale;
	float duty;

	float phase;
	float f_start;
	float f_end;
	float sweep_n;      /* sweep length in samples */
	float log_ratio;    /* logf(f_end / f_start), precomputed for the sweep */

	int n_samples;
	int sample_idx;
} ChipKickState;

static void chip_kick_trigger(void *state_v, float pitch)
{
	ChipKickState *st = (ChipKickState *)state_v;
	float ratio = powf(2.0f, pitch / 12.0f);

	st->f_start   = CHIP_KICK_F_START * ratio;
	st->f_end     = CHIP_KICK_F_END * ratio;
	st->log_ratio = logf(CHIP_KICK_F_END / CHIP_KICK_F_START);
	st->sweep_n   = CHIP_KICK_SWEEP_S * DRUM_VOICE_SAMPLE_RATE;
	st->phase     = 0.0f;

	chip_env_set(&st->env, CHIP_KICK_ENV_PER, st->decay_scale);
	chip_env_reset(&st->env);

	st->n_samples  = (int)(CHIP_KICK_GATE_S * st->decay_scale * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void chip_kick_render(void *state_v, float *out, int n)
{
	ChipKickState *st = (ChipKickState *)state_v;
	const float inv_sweep_n = 1.0f / st->sweep_n;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float frac = (float)st->sample_idx * inv_sweep_n;
		if (frac > 1.0f) frac = 1.0f;
		float freq = st->f_start * drum_fast_expf(st->log_ratio * frac);

		out[i] = chip_pulse_next(&st->phase, freq, st->duty) *
			chip_env_next(&st->env) * CHIP_KICK_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void chip_kick_set_filter(void *state_v, float cutoff01)
{
	ChipKickState *st = (ChipKickState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void chip_kick_set_decay(void *state_v, float decay01)
{
	ChipKickState *st = (ChipKickState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay_scale = CHIP_KICK_DECAY_MIN + decay01 * (CHIP_KICK_DECAY_MAX - CHIP_KICK_DECAY_MIN);
}

/* "other" -> pulse duty cycle, quantised to the four settings the 2A03's
 * pulse channels can actually select: on a square-wave kick the duty is
 * the only timbre control the hardware has, and it swings the body from
 * hollow/nasal (12.5%) to full square. */
static void chip_kick_set_other(void *state_v, float other01)
{
	ChipKickState *st = (ChipKickState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->duty = chip_duty(other01);
}

static void chip_kick_init(void *state_v)
{
	ChipKickState *st = (ChipKickState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->decay_scale = 1.0f;
	st->duty        = 0.5f;
	chip_env_set(&st->env, CHIP_KICK_ENV_PER, 1.0f);
	chip_env_reset(&st->env);
}

const DrumVoiceOps drum_voice_chip_kick = {
	.init       = chip_kick_init,
	.trigger    = chip_kick_trigger,
	.render     = chip_kick_render,
	.set_filter = chip_kick_set_filter,
	.set_decay  = chip_kick_set_decay,
	.set_other  = chip_kick_set_other,
	.state_size = sizeof(ChipKickState),
};
