/*
 * drum_voice_roller_open_hat.c
 *
 * Port of synth_open_hat() from
 * drum_prototype/render_roller_style_drums.py: the shared six-square
 * metal bank (see drum_roller_metal.h) voiced slightly lower and darker
 * than the closed hat, with a long ringing decay.
 *
 * -----------------------------------------------------------------------------
 */

#include <string.h>

#include "drum_roller_metal.h"
#include "drum_shared_filter.h"
#include "drum_voice.h"

#define OHAT_DECAY_MIN 0.08f
#define OHAT_DECAY_MAX 0.90f

#define OHAT_LEVEL     0.6f
#define OHAT_BP_HZ     8200.0f
#define OHAT_HP_HZ     5800.0f
#define OHAT_RATIO     2.1f
#define OHAT_NOISE_MIX 0.5f

#define OHAT_BRIGHT_MIN 0.60f
#define OHAT_BRIGHT_MAX 1.60f

/* Stands in for the python reference's peak normalisation. */
#define OHAT_GAIN 0.85f

typedef struct {
	DrumSharedFilter filt;
	DrumRollerMetal metal;

	float decay;
	float bright;
	float pitch_ratio;

	int n_samples;
	int sample_idx;
} RollerOhatState;

static void ohat_rebuild(RollerOhatState *st)
{
	float scale = st->bright * st->pitch_ratio;
	drum_roller_metal_config(&st->metal, OHAT_BP_HZ * scale, OHAT_HP_HZ,
	                         OHAT_RATIO * scale, OHAT_NOISE_MIX);
	drum_roller_metal_set_env(&st->metal, OHAT_LEVEL, st->decay);
}

static void ohat_trigger(void *state_v, float pitch)
{
	RollerOhatState *st = (RollerOhatState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	ohat_rebuild(st);
	drum_roller_metal_reset(&st->metal);

	st->n_samples  = (int)((st->decay + 0.06f) * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void ohat_render(void *state_v, float *out, int n)
{
	RollerOhatState *st = (RollerOhatState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;
		out[i] = drum_roller_metal_tick(&st->metal, t) * OHAT_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void ohat_set_filter(void *state_v, float cutoff01)
{
	RollerOhatState *st = (RollerOhatState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void ohat_set_decay(void *state_v, float decay01)
{
	RollerOhatState *st = (RollerOhatState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = OHAT_DECAY_MIN + decay01 * (OHAT_DECAY_MAX - OHAT_DECAY_MIN);
	ohat_rebuild(st);
}

/* "other" -> brightness: same reasoning as the closed hat -- transposing
 * the square bank with its bandpass is what moves this between a thin
 * sizzle and a wide open crash-adjacent wash. */
static void ohat_set_other(void *state_v, float other01)
{
	RollerOhatState *st = (RollerOhatState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->bright = OHAT_BRIGHT_MIN + other01 * (OHAT_BRIGHT_MAX - OHAT_BRIGHT_MIN);
	ohat_rebuild(st);
}

static void ohat_init(void *state_v)
{
	RollerOhatState *st = (RollerOhatState *)state_v;
	memset(st, 0, sizeof(*st));

	drum_shared_filter_init(&st->filt);
	drum_roller_metal_init(&st->metal, 0x3D9F2A61u);

	st->decay       = 0.26f;
	st->bright      = 1.0f;
	st->pitch_ratio = 1.0f;
	ohat_rebuild(st);
}

const DrumVoiceOps drum_voice_roller_open_hat = {
	.init       = ohat_init,
	.trigger    = ohat_trigger,
	.render     = ohat_render,
	.set_filter = ohat_set_filter,
	.set_decay  = ohat_set_decay,
	.set_other  = ohat_set_other,
	.state_size = sizeof(RollerOhatState),
};
