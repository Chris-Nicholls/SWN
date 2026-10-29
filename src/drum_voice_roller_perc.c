/*
 * drum_voice_roller_perc.c
 *
 * Port of synth_perc() from drum_prototype/render_roller_style_drums.py:
 * a sine tom body dropping an octave-and-a-bit over its first 30ms, plus
 * a short bandpassed noise stick transient.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_roller_common.h"
#include "drum_shared_filter.h"
#include "drum_voice.h"

#define PERC_DECAY_MIN 0.05f
#define PERC_DECAY_MAX 0.80f

/* Python picks f0 from velocity as 430/320/250Hz; the knob spans that
 * whole range and a little beyond at each end. */
#define PERC_TUNE_MIN_HZ 125.0f
#define PERC_TUNE_MAX_HZ 500.0f

#define PERC_BODY_LVL 0.78f
#define PERC_NZ_BP_HZ 3200.0f
#define PERC_NZ_BP_Q  1.4f
#define PERC_NZ_LVL   0.34f
#define PERC_NZ_DEC   0.035f
#define PERC_NZ_GATE  0.04f

/* Stands in for the python reference's peak normalisation. */
#define PERC_GAIN 1.00f

typedef struct {
	DrumSharedFilter filt;
	RollerNoise rng;
	RollerBiquad nz_bp;

	RollerExpSeg body_freq;
	RollerEnv body_env;
	RollerEnv nz_env;

	float decay;
	float tune_hz;
	float pitch_ratio;

	float body_phase;

	int n_samples;
	int sample_idx;
} RollerPercState;

static void perc_rebuild(RollerPercState *st)
{
	float f0 = st->tune_hz * st->pitch_ratio;

	roller_exp_seg_set(&st->body_freq, 0.0f, 0.03f, f0 * 2.1f, f0);
	roller_env_set(&st->body_env, 0.002f, PERC_BODY_LVL, st->decay, 0.0008f);
	roller_env_set(&st->nz_env, 0.0f, PERC_NZ_LVL, PERC_NZ_DEC, 0.0008f);
	roller_biquad_set(&st->nz_bp, ROLLER_BQ_BANDPASS, PERC_NZ_BP_HZ, PERC_NZ_BP_Q);
}

static void perc_trigger(void *state_v, float pitch)
{
	RollerPercState *st = (RollerPercState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	perc_rebuild(st);

	roller_biquad_reset(&st->nz_bp);
	st->body_phase = 0.0f;
	st->n_samples  = (int)((st->decay + 0.06f) * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void perc_render(void *state_v, float *out, int n)
{
	RollerPercState *st = (RollerPercState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	const float body_gate = st->decay + 0.04f;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;

		float f = roller_exp_segs_at(&st->body_freq, 1, t);
		st->body_phase += f * inv_sr;
		if (st->body_phase >= 1.0f) st->body_phase -= 1.0f;

		float body = 0.0f;
		if (t <= body_gate)
			body = drum_fast_sin_turns(st->body_phase) * roller_env_at(&st->body_env, t);

		float nz = roller_biquad_tick(&st->nz_bp, roller_noise_next(&st->rng));
		nz = (t <= PERC_NZ_GATE) ? nz * roller_env_at(&st->nz_env, t) : 0.0f;

		out[i] = (body + nz) * PERC_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void perc_set_filter(void *state_v, float cutoff01)
{
	RollerPercState *st = (RollerPercState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void perc_set_decay(void *state_v, float decay01)
{
	RollerPercState *st = (RollerPercState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = PERC_DECAY_MIN + decay01 * (PERC_DECAY_MAX - PERC_DECAY_MIN);
	perc_rebuild(st);
}

/* "other" -> tune: in the python original the only thing velocity really
 * changes about this voice is f0, and sweeping it walks the drum from a
 * floor tom up through rack toms to a conga/tabla ping. */
static void perc_set_other(void *state_v, float other01)
{
	RollerPercState *st = (RollerPercState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->tune_hz = PERC_TUNE_MIN_HZ + other01 * (PERC_TUNE_MAX_HZ - PERC_TUNE_MIN_HZ);
	perc_rebuild(st);
}

static void perc_init(void *state_v)
{
	RollerPercState *st = (RollerPercState *)state_v;
	memset(st, 0, sizeof(*st));

	drum_shared_filter_init(&st->filt);
	roller_noise_seed(&st->rng, 0x8C4B71E9u);

	st->decay       = 0.12f;
	st->tune_hz     = 250.0f;
	st->pitch_ratio = 1.0f;
	perc_rebuild(st);
}

const DrumVoiceOps drum_voice_roller_perc = {
	.init       = perc_init,
	.trigger    = perc_trigger,
	.render     = perc_render,
	.set_filter = perc_set_filter,
	.set_decay  = perc_set_decay,
	.set_other  = perc_set_other,
	.state_size = sizeof(RollerPercState),
};
