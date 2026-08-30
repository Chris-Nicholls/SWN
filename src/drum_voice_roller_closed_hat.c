/*
 * drum_voice_roller_closed_hat.c
 *
 * Port of synth_closed_hat() from
 * drum_prototype/render_roller_style_drums.py: the shared six-square
 * metal bank (see drum_roller_metal.h) tuned bright and tight, blended
 * with highpassed noise.
 *
 * -----------------------------------------------------------------------------
 */

#include <string.h>

#include "drum_roller_metal.h"
#include "drum_shared_filter.h"
#include "drum_voice.h"

#define CHAT_DECAY_MIN 0.012f
#define CHAT_DECAY_MAX 0.120f

#define CHAT_LEVEL     0.7f
#define CHAT_BP_HZ     9000.0f
#define CHAT_HP_HZ     6400.0f
#define CHAT_RATIO     2.15f
#define CHAT_NOISE_MIX 0.6f

#define CHAT_BRIGHT_MIN 0.60f
#define CHAT_BRIGHT_MAX 1.60f

/* Stands in for the python reference's peak normalisation. */
#define CHAT_GAIN 0.85f

typedef struct {
	DrumSharedFilter filt;
	DrumRollerMetal metal;

	float decay;
	float bright;
	float pitch_ratio;

	int n_samples;
	int sample_idx;
} RollerChatState;

static void chat_rebuild(RollerChatState *st)
{
	float scale = st->bright * st->pitch_ratio;
	drum_roller_metal_config(&st->metal, CHAT_BP_HZ * scale, CHAT_HP_HZ,
	                         CHAT_RATIO * scale, CHAT_NOISE_MIX);
	drum_roller_metal_set_env(&st->metal, CHAT_LEVEL, st->decay);
}

static void chat_trigger(void *state_v, float pitch)
{
	RollerChatState *st = (RollerChatState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);
	chat_rebuild(st);
	drum_roller_metal_reset(&st->metal);

	st->n_samples  = (int)((st->decay + 0.06f) * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void chat_render(void *state_v, float *out, int n)
{
	RollerChatState *st = (RollerChatState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;
		out[i] = drum_roller_metal_tick(&st->metal, t) * CHAT_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void chat_set_filter(void *state_v, float cutoff01)
{
	RollerChatState *st = (RollerChatState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void chat_set_decay(void *state_v, float decay01)
{
	RollerChatState *st = (RollerChatState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = CHAT_DECAY_MIN + decay01 * (CHAT_DECAY_MAX - CHAT_DECAY_MIN);
	chat_rebuild(st);
}

/* "other" -> brightness: a hat is characterised almost entirely by where
 * the square bank's inharmonic comb sits against the bandpass, so moving
 * both together is what separates a small tight 909 hat from a big
 * splashy one. Decay is already on its own knob. */
static void chat_set_other(void *state_v, float other01)
{
	RollerChatState *st = (RollerChatState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->bright = CHAT_BRIGHT_MIN + other01 * (CHAT_BRIGHT_MAX - CHAT_BRIGHT_MIN);
	chat_rebuild(st);
}

static void chat_init(void *state_v)
{
	RollerChatState *st = (RollerChatState *)state_v;
	memset(st, 0, sizeof(*st));

	drum_shared_filter_init(&st->filt);
	drum_roller_metal_init(&st->metal, 0x5A3C11B7u);

	st->decay       = 0.0345f;
	st->bright      = 1.0f;
	st->pitch_ratio = 1.0f;
	chat_rebuild(st);
}

const DrumVoiceOps drum_voice_roller_closed_hat = {
	.init       = chat_init,
	.trigger    = chat_trigger,
	.render     = chat_render,
	.set_filter = chat_set_filter,
	.set_decay  = chat_set_decay,
	.set_other  = chat_set_other,
	.state_size = sizeof(RollerChatState),
};
