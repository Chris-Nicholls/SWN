/*
 * drum_voice_chip_lfsr.h
 *
 * Shared building blocks for the chiptune (NES 2A03 APU) drum family:
 * the 15-bit Galois LFSR noise channel, the documented fixed noise-period
 * table, and the APU's genuine 16-step linear staircase envelope.
 *
 * These live in a header rather than being copy-pasted per voice (as the
 * mpump family does with its 6-line xorshift) because all six chip voices
 * need bit-identical LFSR/envelope behaviour -- a divergent copy would
 * silently change the family's whole character.
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>

#include "drum_voice.h"

#define CHIP_APU_CLOCK    894886.0f   /* NTSC 2A03 APU clock (CPU clock / 2) */
#define CHIP_FRAME_SEQ_HZ 240.0f      /* quarter-frame envelope clock, NTSC 4-step mode */

/* The hardware's 16 selectable noise periods, in APU clock ticks. This is a
 * lookup table on the real chip, not a formula, and the audible "steppiness"
 * of the noise pitch across the range is part of the character. */
static inline float chip_noise_period(int index)
{
	static const uint16_t table[16] = {
		4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
	};
	if (index < 0)  index = 0;
	if (index > 15) index = 15;
	return (float)table[index];
}

/* 0..1 -> one of the 16 discrete table entries. */
static inline int chip_noise_period_index(float x01)
{
	int idx = (int)(x01 * 16.0f);
	if (idx < 0)  idx = 0;
	if (idx > 15) idx = 15;
	return idx;
}

/* ---- noise channel -------------------------------------------------------- */

typedef struct {
	uint16_t shift;              /* 15-bit LFSR state */
	uint8_t  metallic;           /* 0: bit1 feedback tap (32767-step), 1: bit6 tap (93-step) */
	float    acc;
	float    samples_per_update;
	float    level;
} ChipLfsr;

static inline void chip_lfsr_set_period(ChipLfsr *l, float period_ticks)
{
	if (period_ticks < 1.0f) period_ticks = 1.0f;
	l->samples_per_update = DRUM_VOICE_SAMPLE_RATE * period_ticks / CHIP_APU_CLOCK;
	/* Never let the shift rate outrun the sample rate to the point where
	 * the catch-up loop below cannot terminate. */
	if (l->samples_per_update < 1.0e-3f) l->samples_per_update = 1.0e-3f;
}

static inline void chip_lfsr_reset(ChipLfsr *l, uint16_t seed)
{
	l->shift = (uint16_t)(seed & 0x7FFFu);
	if (l->shift == 0u) l->shift = 1u;
	l->acc   = 0.0f;
	l->level = -1.0f;
}

static inline void chip_lfsr_init(ChipLfsr *l, uint16_t seed, uint8_t metallic, float period_ticks)
{
	l->metallic = metallic;
	chip_lfsr_set_period(l, period_ticks);
	chip_lfsr_reset(l, seed);
}

static inline float chip_lfsr_next(ChipLfsr *l)
{
	l->acc += 1.0f;
	while (l->acc >= l->samples_per_update) {
		l->acc -= l->samples_per_update;
		uint16_t bit0 = (uint16_t)(l->shift & 1u);
		uint16_t tap  = l->metallic ? (uint16_t)((l->shift >> 6) & 1u)
		                            : (uint16_t)((l->shift >> 1) & 1u);
		uint16_t fb   = (uint16_t)(bit0 ^ tap);
		l->shift = (uint16_t)((l->shift >> 1) | (uint16_t)(fb << 14));
		l->level = (l->shift & 1u) ? -1.0f : 1.0f;
	}
	return l->level;
}

/* ---- envelope generator --------------------------------------------------- */

/* Linear 16-step staircase, level 15 down to 0, one step every
 * (period + 1) quarter-frame clocks. Deliberately not interpolated:
 * the quantised amplitude steps are the point of this family. */
typedef struct {
	float acc;
	float samples_per_step;
	int   step;
} ChipEnv;

static inline void chip_env_set(ChipEnv *e, float period, float time_scale)
{
	if (time_scale < 0.01f) time_scale = 0.01f;
	float steps_per_sec = CHIP_FRAME_SEQ_HZ / (period + 1.0f) / time_scale;
	e->samples_per_step = DRUM_VOICE_SAMPLE_RATE / steps_per_sec;
	if (e->samples_per_step < 1.0f) e->samples_per_step = 1.0f;
}

static inline void chip_env_reset(ChipEnv *e)
{
	e->acc  = 0.0f;
	e->step = 0;
}

static inline float chip_env_next(ChipEnv *e)
{
	float v = (e->step >= 15) ? 0.0f : (float)(15 - e->step) * (1.0f / 15.0f);
	e->acc += 1.0f;
	while (e->acc >= e->samples_per_step) {
		e->acc -= e->samples_per_step;
		if (e->step < 15) e->step++;
	}
	return v;
}

/* ---- duty-cycle pulse ----------------------------------------------------- */

/* The four duty settings the pulse channels can actually select. */
static inline float chip_duty(float x01)
{
	static const float table[4] = { 0.125f, 0.25f, 0.5f, 0.75f };
	int idx = (int)(x01 * 4.0f);
	if (idx < 0) idx = 0;
	if (idx > 3) idx = 3;
	return table[idx];
}

static inline float chip_pulse_next(float *phase, float freq_hz, float duty)
{
	*phase += freq_hz * (1.0f / DRUM_VOICE_SAMPLE_RATE);
	while (*phase >= 1.0f) *phase -= 1.0f;
	return (*phase < duty) ? 1.0f : -1.0f;
}
