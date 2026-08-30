/*
 * drum_voice_roller_rimshot.c
 *
 * Port of synth_rimshot() from
 * drum_prototype/render_roller_style_drums.py: a very narrow (Q=7)
 * bandpassed noise burst at 1.75kHz plus an 860Hz square click.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_roller_common.h"
#include "drum_shared_filter.h"
#include "drum_voice.h"

#define RIM_DECAY_MIN 0.008f
#define RIM_DECAY_MAX 0.100f

#define RIM_NZ_HZ    1750.0f
#define RIM_NZ_Q     7.0f
#define RIM_NZ_LVL   0.8f
#define RIM_CLICK_HZ 860.0f
#define RIM_CLICK_LVL 0.3f
#define RIM_CLICK_DEC 0.008f
#define RIM_CLICK_GATE 0.02f

#define RIM_TUNE_MIN 0.5f
#define RIM_TUNE_MAX 2.0f

/* Stands in for the python reference's peak normalisation. */
#define RIM_GAIN 2.20f

typedef struct {
	DrumSharedFilter filt;
	RollerNoise rng;
	RollerBiquad nz_bp;

	RollerEnv nz_env;
	RollerEnv click_env;

	float decay;
	float tune;
	float pitch_ratio;

	float click_phase;
	float click_inc;

	int n_samples;
	int sample_idx;
} RollerRimState;

static void rim_rebuild(RollerRimState *st)
{
	float scale = st->tune * st->pitch_ratio;

	roller_biquad_set(&st->nz_bp, ROLLER_BQ_BANDPASS, RIM_NZ_HZ * scale, RIM_NZ_Q);
	roller_env_set(&st->nz_env, 0.0f, RIM_NZ_LVL, st->decay, 0.0008f);
	roller_env_set(&st->click_env, 0.0f, RIM_CLICK_LVL, RIM_CLICK_DEC, 0.0008f);

	st->click_inc = RIM_CLICK_HZ * scale / DRUM_VOICE_SAMPLE_RATE;
}

static void rim_trigger(void *state_v, float pitch)
{
	RollerRimState *st = (RollerRimState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	rim_rebuild(st);

	roller_biquad_reset(&st->nz_bp);
	st->click_phase = 0.0f;
	st->n_samples   = (int)((st->decay + 0.03f) * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx  = 0;
}

static void rim_render(void *state_v, float *out, int n)
{
	RollerRimState *st = (RollerRimState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	const float nz_gate = st->decay + 0.008f;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;

		float nz = roller_biquad_tick(&st->nz_bp, roller_noise_next(&st->rng));
		nz = (t <= nz_gate) ? nz * roller_env_at(&st->nz_env, t) : 0.0f;

		st->click_phase += st->click_inc;
		if (st->click_phase >= 1.0f) st->click_phase -= 1.0f;

		float click = 0.0f;
		if (t <= RIM_CLICK_GATE)
			click = ((st->click_phase < 0.5f) ? 1.0f : -1.0f) * roller_env_at(&st->click_env, t);

		out[i] = (nz + click) * RIM_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void rim_set_filter(void *state_v, float cutoff01)
{
	RollerRimState *st = (RollerRimState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void rim_set_decay(void *state_v, float decay01)
{
	RollerRimState *st = (RollerRimState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = RIM_DECAY_MIN + decay01 * (RIM_DECAY_MAX - RIM_DECAY_MIN);
	rim_rebuild(st);
}

/* "other" -> tune: with a Q=7 bandpass the noise burst is effectively a
 * pitched ping, so its centre frequency (moved together with the click
 * square) is the only thing that changes the drum's identity -- woodblock
 * up top, cross-stick down low. */
static void rim_set_other(void *state_v, float other01)
{
	RollerRimState *st = (RollerRimState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->tune = RIM_TUNE_MIN + other01 * (RIM_TUNE_MAX - RIM_TUNE_MIN);
	rim_rebuild(st);
}

static void rim_init(void *state_v)
{
	RollerRimState *st = (RollerRimState *)state_v;
	memset(st, 0, sizeof(*st));

	drum_shared_filter_init(&st->filt);
	roller_noise_seed(&st->rng, 0xA37E5C21u);

	st->decay       = 0.022f;
	st->tune        = 1.0f;
	st->pitch_ratio = 1.0f;
	rim_rebuild(st);
}

const DrumVoiceOps drum_voice_roller_rimshot = {
	.init       = rim_init,
	.trigger    = rim_trigger,
	.render     = rim_render,
	.set_filter = rim_set_filter,
	.set_decay  = rim_set_decay,
	.set_other  = rim_set_other,
	.state_size = sizeof(RollerRimState),
};
