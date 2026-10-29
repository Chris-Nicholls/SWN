/*
 * drum_voice_mpump_cowbell.c
 *
 * Port of synth_cowbell() from
 * drum_prototype/render_mpump_style_drums.py: the classic pair of
 * detuned square oscillators (545/815Hz) under one exponential, mixed
 * with a resonant bandpass copy of themselves to round off the edges.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

#define COWBELL_N_SECONDS   0.15f
#define COWBELL_MAX_SECONDS 1.0f

#define COWBELL_DECAY_MIN 0.3f
#define COWBELL_DECAY_MAX 3.0f

#define COWBELL_TUNE_RANGE_SEMI 12.0f

/* Fixed internal shaping, per the port's convention that only the
 * shared post-voice SVF is user-controllable. */
#define COWBELL_BP_FREQ 800.0f
#define COWBELL_BP_Q    4.0f

typedef struct {
	float b0, a0, a1, a2;
	float x1, x2, y1, y2;
} Biquad;

static void biquad_set_bandpass(Biquad *bq, float freq, float q, float sr)
{
	float w0 = 2.0f * (float)M_PI * freq / sr;
	float alpha = sinf(w0) / (2.0f * q);
	bq->b0 = alpha;
	bq->a0 = 1.0f + alpha;
	bq->a1 = -2.0f * cosf(w0);
	bq->a2 = 1.0f - alpha;
}

static float biquad_process1(Biquad *bq, float x)
{
	float y = (bq->b0 * x - bq->b0 * bq->x2 - bq->a1 * bq->y1 - bq->a2 * bq->y2) / bq->a0;
	bq->x2 = bq->x1; bq->x1 = x;
	bq->y2 = bq->y1; bq->y1 = y;
	return y;
}

typedef struct {
	DrumSharedFilter filt;
	Biquad body_bp;

	float pitch;
	float tune;
	float decay;
	float pitch_ratio;

	int n_samples;
	int sample_idx;
} CowbellState;

static void cowbell_update_ratio(CowbellState *st)
{
	st->pitch_ratio = powf(2.0f, (st->pitch + st->tune) / 12.0f);
	biquad_set_bandpass(&st->body_bp, COWBELL_BP_FREQ * st->pitch_ratio,
	                    COWBELL_BP_Q, DRUM_VOICE_SAMPLE_RATE);
}

static void cowbell_trigger(void *state_v, float pitch)
{
	CowbellState *st = (CowbellState *)state_v;

	st->pitch = pitch;
	cowbell_update_ratio(st);
	st->body_bp.x1 = st->body_bp.x2 = st->body_bp.y1 = st->body_bp.y2 = 0.0f;

	float n_seconds = COWBELL_N_SECONDS * st->decay;
	if (n_seconds > COWBELL_MAX_SECONDS) n_seconds = COWBELL_MAX_SECONDS;
	st->n_samples  = (int)(n_seconds * DRUM_VOICE_SAMPLE_RATE);
	st->sample_idx = 0;
}

static float square_sign(float turns)
{
	turns -= (float)(int)turns;
	if (turns < 0.0f) turns += 1.0f;
	return (turns < 0.5f) ? 1.0f : -1.0f;
}

static void cowbell_render(void *state_v, float *out, int n)
{
	CowbellState *st = (CowbellState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;
	const float inv_decay = 1.0f / st->decay;
	float r = st->pitch_ratio;
	float f1 = 545.0f * r;
	float f2 = 815.0f * r;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float t = (float)st->sample_idx * inv_sr;
		float env = drum_fast_expf(-t * (20.0f * inv_decay));

		float raw = (square_sign(f1 * t) * 0.22f +
		             square_sign(f2 * t) * 0.22f) * env;
		float shaped = biquad_process1(&st->body_bp, raw);

		out[i] = raw * 0.6f + shaped * 0.4f;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void cowbell_set_filter(void *state_v, float cutoff01)
{
	CowbellState *st = (CowbellState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void cowbell_set_decay(void *state_v, float decay01)
{
	CowbellState *st = (CowbellState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = COWBELL_DECAY_MIN + decay01 * (COWBELL_DECAY_MAX - COWBELL_DECAY_MIN);
}

/* "other" -> tune: tune is synth_cowbell()'s only character parameter,
 * and it drags the bandpass with it, so the whole bell scales in size
 * rather than just shifting pitch. 0..1 maps to +/-1 octave. */
static void cowbell_set_other(void *state_v, float other01)
{
	CowbellState *st = (CowbellState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->tune = (other01 * 2.0f - 1.0f) * COWBELL_TUNE_RANGE_SEMI;
	cowbell_update_ratio(st);
}

static void cowbell_init(void *state_v)
{
	CowbellState *st = (CowbellState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->decay = 1.0f;
	st->tune  = 0.0f;
	cowbell_update_ratio(st);
}

const DrumVoiceOps drum_voice_mpump_cowbell = {
	.init       = cowbell_init,
	.trigger    = cowbell_trigger,
	.render     = cowbell_render,
	.set_filter = cowbell_set_filter,
	.set_decay  = cowbell_set_decay,
	.set_other  = cowbell_set_other,
	.state_size = sizeof(CowbellState),
};
