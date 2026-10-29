/*
 * drum_voice_roller_crash.c
 *
 * Port of synth_crash() from drum_prototype/render_roller_style_drums.py:
 * a highpassed noise wash pushed through a lowpass that sweeps 15kHz down
 * to 4.2kHz as the hit rings out, layered with a long, quiet run of the
 * shared metal bank (see drum_roller_metal.h).
 *
 * -----------------------------------------------------------------------------
 */

#include <string.h>

#include "drum_roller_metal.h"
#include "drum_shared_filter.h"
#include "drum_voice.h"

#define CRASH_DECAY_MIN 0.50f
#define CRASH_DECAY_MAX 3.00f

#define CRASH_WASH_DEC   1.8f     /* python defaults the other two times are expressed against */
#define CRASH_SWEEP_FRAC 0.944f   /* 1.7 / 1.8 */
#define CRASH_METAL_FRAC 0.722f   /* 1.3 / 1.8 */

#define CRASH_WASH_HP_HZ 4600.0f
#define CRASH_LP_START   15000.0f
#define CRASH_LP_END     4200.0f
#define CRASH_LP_Q       0.7f

#define CRASH_METAL_LEVEL 0.16f
#define CRASH_METAL_BP_HZ 7800.0f
#define CRASH_METAL_HP_HZ 5200.0f
#define CRASH_METAL_RATIO 2.6f

#define CRASH_BRIGHT_MIN 0.60f
#define CRASH_BRIGHT_MAX 1.60f

/* Recomputing the RBJ coefficients every sample (as the python reference
 * does) would cost a sinf+cosf per sample on the F7; the sweep spans
 * nearly two seconds, so a 16-sample coefficient grid is inaudible. */
#define CRASH_SWEEP_UPDATE 16

/* Stands in for the python reference's peak normalisation. */
#define CRASH_GAIN 1.00f

typedef struct {
	DrumSharedFilter filt;
	DrumRollerMetal metal;

	RollerNoise rng;
	RollerBiquad wash_hp;
	RollerBiquad wash_lp;
	RollerExpSeg lp_sweep;
	RollerEnv wash_env;

	float decay;
	float bright;
	float pitch_ratio;

	int n_samples;
	int sample_idx;
	int sweep_countdown;
} RollerCrashState;

static void crash_rebuild(RollerCrashState *st)
{
	float scale = st->bright * st->pitch_ratio;

	drum_roller_metal_config(&st->metal, CRASH_METAL_BP_HZ * scale, CRASH_METAL_HP_HZ,
	                         CRASH_METAL_RATIO * scale, 0.0f);
	drum_roller_metal_set_env(&st->metal, CRASH_METAL_LEVEL, st->decay * CRASH_METAL_FRAC);

	roller_env_set(&st->wash_env, 0.004f, 0.5f, st->decay, 0.0006f);
	roller_exp_seg_set(&st->lp_sweep, 0.0f, st->decay * CRASH_SWEEP_FRAC,
	                   CRASH_LP_START * scale, CRASH_LP_END * scale);
	roller_biquad_set(&st->wash_hp, ROLLER_BQ_HIGHPASS, CRASH_WASH_HP_HZ, 0.6f);
}

static void crash_trigger(void *state_v, float pitch)
{
	RollerCrashState *st = (RollerCrashState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	crash_rebuild(st);

	drum_roller_metal_reset(&st->metal);
	roller_biquad_reset(&st->wash_hp);
	roller_biquad_reset(&st->wash_lp);

	st->sweep_countdown = 0;
	st->n_samples  = (int)((st->decay + 0.12f) * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void crash_render(void *state_v, float *out, int n)
{
	RollerCrashState *st = (RollerCrashState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	const float wash_gate = st->decay + 0.1f;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;

		if (st->sweep_countdown-- <= 0) {
			roller_biquad_set(&st->wash_lp, ROLLER_BQ_LOWPASS,
			                  roller_exp_segs_at(&st->lp_sweep, 1, t), CRASH_LP_Q);
			st->sweep_countdown = CRASH_SWEEP_UPDATE - 1;
		}

		float wash = roller_biquad_tick(&st->wash_lp,
		                                roller_biquad_tick(&st->wash_hp,
		                                                   roller_noise_next(&st->rng)));
		wash = (t <= wash_gate) ? wash * roller_env_at(&st->wash_env, t) : 0.0f;

		out[i] = (wash + drum_roller_metal_tick(&st->metal, t)) * CRASH_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void crash_set_filter(void *state_v, float cutoff01)
{
	RollerCrashState *st = (RollerCrashState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void crash_set_decay(void *state_v, float decay01)
{
	RollerCrashState *st = (RollerCrashState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = CRASH_DECAY_MIN + decay01 * (CRASH_DECAY_MAX - CRASH_DECAY_MIN);
	crash_rebuild(st);
}

/* "other" -> brightness: the wash's lowpass sweep endpoints and the metal
 * bank move together, which is the difference between a dark, thick
 * china and a bright splashy crash -- the sweep alone is what gives this
 * voice its whole shape, so it is the only knob worth exposing. */
static void crash_set_other(void *state_v, float other01)
{
	RollerCrashState *st = (RollerCrashState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->bright = CRASH_BRIGHT_MIN + other01 * (CRASH_BRIGHT_MAX - CRASH_BRIGHT_MIN);
	crash_rebuild(st);
}

static void crash_init(void *state_v)
{
	RollerCrashState *st = (RollerCrashState *)state_v;
	memset(st, 0, sizeof(*st));

	drum_shared_filter_init(&st->filt);
	drum_roller_metal_init(&st->metal, 0x91B7E20Fu);
	roller_noise_seed(&st->rng, 0x4E7C1D83u);

	st->decay       = CRASH_WASH_DEC;
	st->bright      = 1.0f;
	st->pitch_ratio = 1.0f;
	crash_rebuild(st);
}

const DrumVoiceOps drum_voice_roller_crash = {
	.init       = crash_init,
	.trigger    = crash_trigger,
	.render     = crash_render,
	.set_filter = crash_set_filter,
	.set_decay  = crash_set_decay,
	.set_other  = crash_set_other,
	.state_size = sizeof(RollerCrashState),
};
