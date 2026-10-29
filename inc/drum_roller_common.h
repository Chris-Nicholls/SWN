/*
 * drum_roller_common.h
 *
 * Shared building blocks for the ROLLER-style voice family (ported from
 * drum_prototype/render_roller_style_drums.py): the RBJ-cookbook biquads
 * and the Web Audio AudioParam automation curves
 * (setValueAtTime/linearRampToValueAtTime/exponentialRampToValueAtTime)
 * that every ROLLER voice is assembled from.
 *
 * Header-only: the ramp/biquad evaluators sit in each voice's per-sample
 * loop, so keeping them inline avoids a call per sample per layer.
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

#include <math.h>
#include <stdint.h>

#include "drum_fast_math.h"
#include "drum_voice.h"

/* ---- noise source ------------------------------------------------------- */

typedef struct RollerNoise {
	uint32_t s;
} RollerNoise;

static inline void roller_noise_seed(RollerNoise *n, uint32_t seed)
{
	n->s = seed ? seed : 1u;
}

static inline float roller_noise_next(RollerNoise *n)
{
	uint32_t x = n->s;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	n->s = x;
	return ((float)(x & 0xFFFFFFu) / (float)0x800000u) - 1.0f;
}

/* ---- RBJ / Web-Audio-spec biquad ---------------------------------------- */

typedef enum RollerBiquadKind {
	ROLLER_BQ_LOWPASS,
	ROLLER_BQ_HIGHPASS,
	ROLLER_BQ_BANDPASS,
} RollerBiquadKind;

typedef struct RollerBiquad {
	float b0, b1, b2, a1, a2;
	float x1, x2, y1, y2;
} RollerBiquad;

static inline void roller_biquad_reset(RollerBiquad *bq)
{
	bq->x1 = bq->x2 = bq->y1 = bq->y2 = 0.0f;
}

static inline void roller_biquad_set(RollerBiquad *bq, RollerBiquadKind kind, float freq, float q)
{
	const float nyq = 0.49f * DRUM_VOICE_SAMPLE_RATE;
	if (freq < 10.0f)  freq = 10.0f;
	if (freq > nyq)    freq = nyq;
	if (q < 0.05f)     q = 0.05f;

	float turns = freq * (1.0f / DRUM_VOICE_SAMPLE_RATE);
	float cosw0 = drum_fast_sin_turns(turns + 0.25f);
	float alpha = drum_fast_sin_turns(turns) / (2.0f * q);

	float b0, b1, b2;
	switch (kind) {
	case ROLLER_BQ_LOWPASS:
		b0 = (1.0f - cosw0) * 0.5f;
		b1 = 1.0f - cosw0;
		b2 = b0;
		break;
	case ROLLER_BQ_HIGHPASS:
		b0 = (1.0f + cosw0) * 0.5f;
		b1 = -(1.0f + cosw0);
		b2 = b0;
		break;
	default:
		b0 = alpha;
		b1 = 0.0f;
		b2 = -alpha;
		break;
	}

	float a0_inv = 1.0f / (1.0f + alpha);
	bq->b0 = b0 * a0_inv;
	bq->b1 = b1 * a0_inv;
	bq->b2 = b2 * a0_inv;
	bq->a1 = (-2.0f * cosw0) * a0_inv;
	bq->a2 = (1.0f - alpha) * a0_inv;
}

static inline float roller_biquad_tick(RollerBiquad *bq, float x)
{
	float y = bq->b0 * x + bq->b1 * bq->x1 + bq->b2 * bq->x2 - bq->a1 * bq->y1 - bq->a2 * bq->y2;
	bq->x2 = bq->x1; bq->x1 = x;
	bq->y2 = bq->y1; bq->y1 = y;
	return y;
}

/* ---- amplitude envelopes ------------------------------------------------ */

/* linearRamp(0 -> v_peak, t_attack) then exponentialRamp(v_peak -> v_floor,
 * t_end), holding v_floor afterwards. t_attack == 0 degenerates to the
 * setValueAtTime(v_peak) + exponentialRamp form. */
typedef struct RollerEnv {
	float t_attack;
	float v_peak;
	float t_end;
	float v_floor;
	float rate;      /* ln(v_floor/v_peak) / (t_end - t_attack) */
	float atk_inv;
} RollerEnv;

static inline void roller_env_set(RollerEnv *e, float t_attack, float v_peak, float t_end, float v_floor)
{
	if (v_peak < 1e-9f) v_peak = 1e-9f;
	if (v_floor < 1e-9f) v_floor = 1e-9f;
	if (t_end < t_attack + 1e-6f) t_end = t_attack + 1e-6f;

	e->t_attack = t_attack;
	e->v_peak   = v_peak;
	e->t_end    = t_end;
	e->v_floor  = v_floor;
	e->rate     = logf(v_floor / v_peak) / (t_end - t_attack);
	e->atk_inv  = (t_attack > 0.0f) ? (1.0f / t_attack) : 0.0f;
}

static inline float roller_env_at(const RollerEnv *e, float t)
{
	if (t <= e->t_attack)
		return (e->atk_inv > 0.0f) ? e->v_peak * t * e->atk_inv : e->v_peak;
	if (t <= e->t_end)
		return e->v_peak * drum_fast_expf(e->rate * (t - e->t_attack));
	return e->v_floor;
}

/* ---- exponential parameter ramps ---------------------------------------- */

typedef struct RollerExpSeg {
	float t0, t1;
	float v0, v1;
	float rate;   /* ln(v1/v0) / (t1 - t0) */
} RollerExpSeg;

static inline void roller_exp_seg_set(RollerExpSeg *s, float t0, float t1, float v0, float v1)
{
	if (v0 < 1e-9f) v0 = 1e-9f;
	if (v1 < 1e-9f) v1 = 1e-9f;
	if (t1 < t0 + 1e-6f) t1 = t0 + 1e-6f;

	s->t0   = t0;
	s->t1   = t1;
	s->v0   = v0;
	s->v1   = v1;
	s->rate = logf(v1 / v0) / (t1 - t0);
}

static inline float roller_exp_segs_at(const RollerExpSeg *segs, int n, float t)
{
	if (t <= segs[0].t0)
		return segs[0].v0;
	if (t >= segs[n - 1].t1)
		return segs[n - 1].v1;

	for (int i = n - 1; i >= 0; i--) {
		if (t >= segs[i].t0)
			return (t <= segs[i].t1) ? segs[i].v0 * drum_fast_expf(segs[i].rate * (t - segs[i].t0))
			                         : segs[i].v1;
	}
	return segs[0].v0;
}

/* ---- waveshaper --------------------------------------------------------- */

static inline float roller_tanh_shape(float x, float k, float norm_inv)
{
	return drum_fast_tanhf(x * k) * norm_inv;
}
