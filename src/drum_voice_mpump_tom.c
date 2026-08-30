/*
 * drum_voice_mpump_tom.c
 *
 * Port of synth_tom() from drum_prototype/render_mpump_style_drums.py:
 * a pitch-swept sine body (closed-form integrated phase, so the sweep
 * survives being rendered in blocks) plus a fixed high click transient.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

#define TOM_N_SECONDS   0.25f
#define TOM_MAX_SECONDS 2.0f

#define TOM_DECAY_MIN 0.3f
#define TOM_DECAY_MAX 3.0f

#define TOM_TUNE_RANGE_SEMI 12.0f

typedef struct {
	DrumSharedFilter filt;

	float pitch;
	float tune;
	float decay;
	float pitch_ratio;

	float base_f;
	float sweep_f;
	float sweep_depth;
	float s_rate;

	int n_samples;
	int sample_idx;
} TomState;

static void tom_update_ratio(TomState *st)
{
	st->pitch_ratio = powf(2.0f, (st->pitch + st->tune) / 12.0f);
}

static void tom_trigger(void *state_v, float pitch)
{
	TomState *st = (TomState *)state_v;

	st->pitch = pitch;
	tom_update_ratio(st);

	float r = st->pitch_ratio;
	st->base_f  = 200.0f * r;
	st->sweep_f = 80.0f * r;
	st->s_rate  = 25.0f / st->decay;
	st->sweep_depth = st->sweep_f / st->s_rate;

	float n_seconds = TOM_N_SECONDS * st->decay;
	if (n_seconds > TOM_MAX_SECONDS) n_seconds = TOM_MAX_SECONDS;
	st->n_samples  = (int)(n_seconds * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static void tom_render(void *state_v, float *out, int n)
{
	TomState *st = (TomState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	const float inv_decay = 1.0f / st->decay;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;

		float turns = st->base_f * t +
			st->sweep_depth * (1.0f - drum_fast_expf(-t * st->s_rate));
		float body = drum_fast_sin_turns(turns) * drum_fast_expf(-t * (12.0f * inv_decay)) * 0.7f;
		float click = drum_fast_sin_turns(5000.0f * t) * drum_fast_expf(-t * 2500.0f) * 0.08f;

		out[i] = body + click;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void tom_set_filter(void *state_v, float cutoff01)
{
	TomState *st = (TomState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void tom_set_decay(void *state_v, float decay01)
{
	TomState *st = (TomState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = TOM_DECAY_MIN + decay01 * (TOM_DECAY_MAX - TOM_DECAY_MIN);
}

/* "other" -> tune: a tom is only ever "which drum of the kit is this",
 * and tune is the sole timbral parameter synth_tom() takes. 0..1 maps
 * to +/-1 octave around the nominal 200Hz shell, i.e. floor tom to
 * high rack tom. */
static void tom_set_other(void *state_v, float other01)
{
	TomState *st = (TomState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->tune = (other01 * 2.0f - 1.0f) * TOM_TUNE_RANGE_SEMI;
	tom_update_ratio(st);
}

static void tom_init(void *state_v)
{
	TomState *st = (TomState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->decay = 1.0f;
	st->tune  = 0.0f;
	tom_update_ratio(st);
}

const DrumVoiceOps drum_voice_mpump_tom = {
	.init       = tom_init,
	.trigger    = tom_trigger,
	.render     = tom_render,
	.set_filter = tom_set_filter,
	.set_decay  = tom_set_decay,
	.set_other  = tom_set_other,
	.state_size = sizeof(TomState),
};
