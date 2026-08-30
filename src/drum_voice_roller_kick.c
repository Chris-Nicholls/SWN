/*
 * drum_voice_roller_kick.c
 *
 * Port of synth_kick() from drum_prototype/render_roller_style_drums.py:
 * a three-segment exponential pitch drop (4.4x -> 1.25x -> 1x -> 0.93x)
 * driving a sine body through a tanh saturator, plus a 980Hz triangle
 * beater click and a highpassed noise "air" transient.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_roller_common.h"
#include "drum_shared_filter.h"
#include "drum_voice.h"

#define KICK_DECAY_MIN 0.12f
#define KICK_DECAY_MAX 0.90f

/* Python's ktune default is 49Hz; the knob spans a usefully deep sub to a
 * short tom-ish thump either side of it. */
#define KICK_TUNE_MIN_HZ 33.0f
#define KICK_TUNE_MAX_HZ 78.0f

#define KICK_SHAPE_K   1.6f
#define KICK_TAIL_S    0.12f
#define KICK_BEATER_HZ 980.0f

/* Stands in for the python reference's peak normalisation. */
#define KICK_GAIN 0.62f

typedef struct {
	DrumSharedFilter filt;
	RollerNoise rng;
	RollerBiquad air_hp;

	RollerExpSeg freq[3];
	RollerEnv body_env;
	RollerEnv beater_env;
	RollerEnv air_env;

	float decay;
	float tune_hz;
	float pitch_ratio;

	float body_phase;
	float beater_phase;
	float beater_inc;

	int n_samples;
	int sample_idx;
} RollerKickState;

static void kick_rebuild(RollerKickState *st)
{
	float f0 = st->tune_hz * st->pitch_ratio;
	float dec = st->decay;
	if (dec < 0.095f) dec = 0.095f;   /* keep the third pitch segment ahead of the second */

	roller_exp_seg_set(&st->freq[0], 0.0f, 0.022f, f0 * 4.4f, f0 * 1.25f);
	roller_exp_seg_set(&st->freq[1], 0.022f, 0.09f, f0 * 1.25f, f0);
	roller_exp_seg_set(&st->freq[2], 0.09f, dec, f0, f0 * 0.93f);

	roller_env_set(&st->body_env, 0.002f, 1.25f, st->decay, 0.0008f);
	roller_env_set(&st->beater_env, 0.0f, 0.5f, 0.009f, 0.0008f);
	roller_env_set(&st->air_env, 0.0f, 0.36f, 0.014f, 0.0008f);

	st->beater_inc = KICK_BEATER_HZ / DRUM_VOICE_SAMPLE_RATE;
}

static void kick_trigger(void *state_v, float pitch)
{
	RollerKickState *st = (RollerKickState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	kick_rebuild(st);

	roller_biquad_reset(&st->air_hp);
	st->body_phase   = 0.0f;
	st->beater_phase = 0.0f;
	st->sample_idx   = 0;
	st->n_samples    = (int)((st->decay + KICK_TAIL_S + 0.02f) * DRUM_VOICE_SAMPLE_RATE);
}

static void kick_render(void *state_v, float *out, int n)
{
	RollerKickState *st = (RollerKickState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	const float shape_norm_inv = 1.0f / drum_fast_tanhf(KICK_SHAPE_K);
	const float body_gate = st->decay + KICK_TAIL_S;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;

		float f = roller_exp_segs_at(st->freq, 3, t);
		st->body_phase += f * inv_sr;
		if (st->body_phase >= 1.0f) st->body_phase -= 1.0f;

		float body = 0.0f;
		if (t <= body_gate) {
			float osc = drum_fast_sin_turns(st->body_phase);
			body = roller_tanh_shape(osc * roller_env_at(&st->body_env, t),
			                         KICK_SHAPE_K, shape_norm_inv);
		}

		st->beater_phase += st->beater_inc;
		if (st->beater_phase >= 1.0f) st->beater_phase -= 1.0f;

		float beater = 0.0f;
		if (t <= 0.02f) {
			float tri = 2.0f * fabsf(2.0f * st->beater_phase - 1.0f) - 1.0f;
			beater = tri * roller_env_at(&st->beater_env, t);
		}

		float air = roller_biquad_tick(&st->air_hp, roller_noise_next(&st->rng));
		air = (t <= 0.016f) ? air * roller_env_at(&st->air_env, t) : 0.0f;

		out[i] = (body + beater + air) * KICK_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void kick_set_filter(void *state_v, float cutoff01)
{
	RollerKickState *st = (RollerKickState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void kick_set_decay(void *state_v, float decay01)
{
	RollerKickState *st = (RollerKickState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = KICK_DECAY_MIN + decay01 * (KICK_DECAY_MAX - KICK_DECAY_MIN);
	kick_rebuild(st);
}

/* "other" -> ktune: the whole pitch-drop schedule is expressed as
 * multiples of f0, so moving f0 retunes body, sweep depth and tail
 * together -- the one knob that turns this from an 808 sub into a
 * short acoustic thump. */
static void kick_set_other(void *state_v, float other01)
{
	RollerKickState *st = (RollerKickState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->tune_hz = KICK_TUNE_MIN_HZ + other01 * (KICK_TUNE_MAX_HZ - KICK_TUNE_MIN_HZ);
	kick_rebuild(st);
}

static void kick_init(void *state_v)
{
	RollerKickState *st = (RollerKickState *)state_v;
	memset(st, 0, sizeof(*st));

	drum_shared_filter_init(&st->filt);
	roller_noise_seed(&st->rng, 0x1F3A5C7Du);
	roller_biquad_set(&st->air_hp, ROLLER_BQ_HIGHPASS, 2600.0f, 0.7f);

	st->decay       = 0.34f;
	st->tune_hz     = 49.0f;
	st->pitch_ratio = 1.0f;
	kick_rebuild(st);
}

const DrumVoiceOps drum_voice_roller_kick = {
	.init       = kick_init,
	.trigger    = kick_trigger,
	.render     = kick_render,
	.set_filter = kick_set_filter,
	.set_decay  = kick_set_decay,
	.set_other  = kick_set_other,
	.state_size = sizeof(RollerKickState),
};
