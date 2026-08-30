/*
 * drum_voice_roller_snare.c
 *
 * Port of synth_snare() from drum_prototype/render_roller_style_drums.py:
 * two pitch-dropping shell partials (178Hz sine + 268Hz triangle), a
 * bandpassed-then-highpassed noise "wires" layer, and a short highpassed
 * "crack" transient.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_roller_common.h"
#include "drum_shared_filter.h"
#include "drum_voice.h"

#define SNARE_DECAY_MIN 0.06f
#define SNARE_DECAY_MAX 0.60f

#define SNARE_SHELL_DEC 0.058f
#define SNARE_SHELL_LVL 0.52f
#define SNARE_WIRES_HP  620.0f
#define SNARE_CRACK_HP  6200.0f

/* Stands in for the python reference's peak normalisation. */
#define SNARE_GAIN 0.75f

typedef struct {
	DrumSharedFilter filt;
	RollerNoise wires_rng;
	RollerNoise crack_rng;

	RollerBiquad wires_bp;
	RollerBiquad wires_hp;
	RollerBiquad crack_hp;

	RollerExpSeg shell_freq[2];
	RollerEnv shell_env[2];
	RollerEnv wires_env;
	RollerEnv crack_env;

	float decay;
	float snap;
	float pitch_ratio;
	float crack_dec;

	float shell_phase[2];

	int n_samples;
	int sample_idx;
} RollerSnareState;

static void snare_rebuild(RollerSnareState *st)
{
	float tune = st->pitch_ratio;

	roller_exp_seg_set(&st->shell_freq[0], 0.0f, 0.02f, 178.0f * tune * 1.18f, 178.0f * tune);
	roller_exp_seg_set(&st->shell_freq[1], 0.0f, 0.02f, 268.0f * tune * 1.18f, 268.0f * tune);

	roller_env_set(&st->shell_env[0], 0.0f, SNARE_SHELL_LVL, SNARE_SHELL_DEC, 0.0008f);
	roller_env_set(&st->shell_env[1], 0.0f, SNARE_SHELL_LVL * 0.7f, SNARE_SHELL_DEC, 0.0008f);
	roller_env_set(&st->wires_env, 0.002f, 0.95f, st->decay, 0.0008f);

	st->crack_dec = 0.018f + 0.014f * st->snap;
	roller_env_set(&st->crack_env, 0.0f, 0.62f * st->snap, st->crack_dec, 0.0008f);

	roller_biquad_set(&st->wires_bp, ROLLER_BQ_BANDPASS, 1900.0f * tune, 0.62f);
	roller_biquad_set(&st->wires_hp, ROLLER_BQ_HIGHPASS, SNARE_WIRES_HP, 0.7f);
	roller_biquad_set(&st->crack_hp, ROLLER_BQ_HIGHPASS, SNARE_CRACK_HP, 0.8f);
}

static void snare_trigger(void *state_v, float pitch)
{
	RollerSnareState *st = (RollerSnareState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	snare_rebuild(st);

	roller_biquad_reset(&st->wires_bp);
	roller_biquad_reset(&st->wires_hp);
	roller_biquad_reset(&st->crack_hp);
	st->shell_phase[0] = st->shell_phase[1] = 0.0f;

	float len = st->decay + 0.07f;
	if (len < 0.12f) len = 0.12f;
	st->n_samples  = (int)(len * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void snare_render(void *state_v, float *out, int n)
{
	RollerSnareState *st = (RollerSnareState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	const float shell_gate = SNARE_SHELL_DEC + 0.02f;
	const float wires_gate = st->decay + 0.05f;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;

		float shell = 0.0f;
		for (int p = 0; p < 2; p++) {
			float f = roller_exp_segs_at(&st->shell_freq[p], 1, t);
			float ph = st->shell_phase[p] + f * inv_sr;
			if (ph >= 1.0f) ph -= 1.0f;
			st->shell_phase[p] = ph;

			if (t > shell_gate)
				continue;

			float osc = p ? (2.0f * fabsf(2.0f * ph - 1.0f) - 1.0f)
			              : drum_fast_sin_turns(ph);
			shell += osc * roller_env_at(&st->shell_env[p], t);
		}

		float wires = roller_biquad_tick(&st->wires_hp,
		                                 roller_biquad_tick(&st->wires_bp,
		                                                    roller_noise_next(&st->wires_rng)));
		wires = (t <= wires_gate) ? wires * roller_env_at(&st->wires_env, t) : 0.0f;

		float crack = roller_biquad_tick(&st->crack_hp, roller_noise_next(&st->crack_rng));
		crack = (st->snap > 0.01f && t <= 0.03f) ? crack * roller_env_at(&st->crack_env, t) : 0.0f;

		out[i] = (shell + wires + crack) * SNARE_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void snare_set_filter(void *state_v, float cutoff01)
{
	RollerSnareState *st = (RollerSnareState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void snare_set_decay(void *state_v, float decay01)
{
	RollerSnareState *st = (RollerSnareState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = SNARE_DECAY_MIN + decay01 * (SNARE_DECAY_MAX - SNARE_DECAY_MIN);
	snare_rebuild(st);
}

/* "other" -> snap: it is the only parameter that both gates a whole extra
 * layer in and out and lengthens it, so it slides the hit from a pure
 * shell-and-wires backbeat to a bright rimmy crack. 0..1 is already its
 * native range in the python original. */
static void snare_set_other(void *state_v, float other01)
{
	RollerSnareState *st = (RollerSnareState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->snap = other01;
	snare_rebuild(st);
}

static void snare_init(void *state_v)
{
	RollerSnareState *st = (RollerSnareState *)state_v;
	memset(st, 0, sizeof(*st));

	drum_shared_filter_init(&st->filt);
	roller_noise_seed(&st->wires_rng, 0x2B6C4A19u);
	roller_noise_seed(&st->crack_rng, 0x7E1D93A5u);

	st->decay       = 0.17f;
	st->snap        = 0.62f;
	st->pitch_ratio = 1.0f;
	snare_rebuild(st);
}

const DrumVoiceOps drum_voice_roller_snare = {
	.init       = snare_init,
	.trigger    = snare_trigger,
	.render     = snare_render,
	.set_filter = snare_set_filter,
	.set_decay  = snare_set_decay,
	.set_other  = snare_set_other,
	.state_size = sizeof(RollerSnareState),
};
