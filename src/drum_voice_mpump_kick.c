/*
 * drum_voice_mpump_kick.c
 *
 * Port of synth_kick() from drum_prototype/render_mpump_style_drums.py:
 * a frequency-swept sine body (closed-form integrated phase, so the
 * sweep stays continuous across render() calls) plus a fixed sub
 * partial and a two-tone exponential click transient.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

typedef struct {
	DrumSharedFilter filt;

	float pitch_ratio;   /* r = 2^(tune/12) */
	float decay;         /* python's "decay" parameter, seconds-ish scale */
	float click_amt;

	int   n_samples;     /* active length, min(0.6*decay, 2.0) * SR, set at trigger */
	int   sample_idx;

	float base_f;
	float sweep_f;
	float sweep_depth;
	float s_rate;
} KickState;

#define KICK_DECAY_MIN 0.2f
#define KICK_DECAY_MAX 2.5f

static void kick_trigger(void *state_v, float pitch)
{
	KickState *st = (KickState *)state_v;

	st->pitch_ratio = powf(2.0f, pitch / 12.0f);

	float decay = st->decay;
	float r = st->pitch_ratio;
	const float sweep_depth = 0.5f;   /* not exposed via set_other; python default */
	const float sweep_rate  = 0.5f;   /* not exposed via set_other; python default */

	st->base_f  = 45.0f * r;
	st->sweep_f = (80.0f + 170.0f * sweep_depth) * r;
	st->s_rate  = (20.0f + 70.0f * sweep_rate) / fmaxf(decay, 0.5f);
	st->sweep_depth = st->sweep_f / st->s_rate;

	float n_seconds = 0.6f * decay;
	if (n_seconds > 2.0f) n_seconds = 2.0f;
	st->n_samples  = (int)(n_seconds * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void kick_render(void *state_v, float *out, int n)
{
	KickState *st = (KickState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	const float click_f1 = 2000.0f;   /* click_tune fixed at python default (0 semitones) */
	const float click_f2 = 5000.0f;
	const float inv_decay = 1.0f / st->decay;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;

		float turns = st->base_f * t +
			st->sweep_depth * (1.0f - drum_fast_expf(-t * st->s_rate));
		float body_attack = drum_fast_expf(-t * 200.0f);
		float body_tail   = drum_fast_expf(-t * (5.0f * inv_decay));
		float body = drum_fast_sin_turns(turns) * (body_attack * 0.55f + body_tail * 0.12f) * 0.95f;

		float sub = drum_fast_sin_turns(50.0f * st->pitch_ratio * t) * body_tail * 0.4f;

		float click = (drum_fast_sin_turns(click_f1 * t) + drum_fast_sin_turns(click_f2 * t)) *
			0.5f * drum_fast_expf(-t * 2000.0f) * st->click_amt;

		out[i] = body + sub + click;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void kick_set_filter(void *state_v, float cutoff01)
{
	KickState *st = (KickState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void kick_set_decay(void *state_v, float decay01)
{
	KickState *st = (KickState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = KICK_DECAY_MIN + decay01 * (KICK_DECAY_MAX - KICK_DECAY_MIN);
}

/* "other" -> click_amt: the click transient is the most audibly
 * characterful per-hit knob in synth_kick() (sweep_depth/sweep_rate
 * mostly reshape the body tone, click_amt toggles a whole extra
 * transient layer in and out). Passed through 0..1 directly since
 * that's already click_amt's native range in the python original. */
static void kick_set_other(void *state_v, float other01)
{
	KickState *st = (KickState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->click_amt = other01;
}

static void kick_init(void *state_v)
{
	KickState *st = (KickState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->decay     = 1.0f;
	st->click_amt = 0.15f;
	st->n_samples = 0;
	st->sample_idx = 0;
}

const DrumVoiceOps drum_voice_mpump_kick = {
	.init       = kick_init,
	.trigger    = kick_trigger,
	.render     = kick_render,
	.set_filter = kick_set_filter,
	.set_decay  = kick_set_decay,
	.set_other  = kick_set_other,
	.state_size = sizeof(KickState),
};
