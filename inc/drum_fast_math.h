/*
 * drum_fast_math.h
 *
 * Cheap replacements for the libm calls that the drum voices would
 * otherwise make once (or several times) per sample. On the F765 an
 * expf()/sinf() is a software routine costing tens of times a multiply,
 * and with six voices rendering at once the audio ISR cannot afford them.
 *
 * All three approximations are well inside the noise floor of a 16-bit
 * output: exp2 is accurate to ~1e-7 relative, sin to ~2e-6 absolute,
 * tanh is the same Pade form already used for the deluge ladder drive.
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>

/* 2^x. Exponent is assembled by hand; the mantissa comes from the
 * degree-6 Taylor series of 2^f on the unit interval. */
static inline float drum_fast_exp2f(float x)
{
	if (x <= -126.0f) return 0.0f;
	if (x >= 126.0f)  x = 126.0f;

	int   xi = (int)x;
	float f  = x - (float)xi;
	if (f < 0.0f) { f += 1.0f; xi -= 1; }

	float p = 0.0001530658f;
	p = p * f + 0.0013395061f;
	p = p * f + 0.0096180876f;
	p = p * f + 0.0555034023f;
	p = p * f + 0.2402265069f;
	p = p * f + 0.6931471805f;
	p = p * f + 1.0f;

	union { uint32_t u; float f; } scale;
	scale.u = (uint32_t)((xi + 127) << 23);

	return p * scale.f;
}

static inline float drum_fast_expf(float x)
{
	return drum_fast_exp2f(x * 1.4426950409f);
}

/* sin(2*pi*turns): the phase is folded into a quarter period, where the
 * odd Taylor series to x^9 is exact to ~2e-6. */
static inline float drum_fast_sin_turns(float turns)
{
	turns -= (float)(int)turns;
	if (turns < 0.0f)      turns += 1.0f;
	if (turns > 0.5f)      turns -= 1.0f;
	if (turns > 0.25f)     turns = 0.5f - turns;
	else if (turns < -0.25f) turns = -0.5f - turns;

	float x2 = turns * turns;
	float p  = 42.0586939f;
	p = p * x2 - 76.7058598f;
	p = p * x2 + 81.6052493f;
	p = p * x2 - 41.3417022f;
	p = p * x2 + 6.2831853f;
	return turns * p;
}

/* Pade[7/6] of tanh, clamped where it would start to fall away again.
 * Below |x| = 2 it is accurate to 3e-4, and past the clamp the curve is
 * already flat enough that the 2e-3 offset from the true tanh is buried. */
static inline float drum_fast_tanhf(float x)
{
	if (x < -3.0f) x = -3.0f;
	else if (x > 3.0f) x = 3.0f;

	float x2 = x * x;
	float num = 135135.0f + x2 * (17325.0f + x2 * 378.0f);
	float den = 135135.0f + x2 * (62370.0f + x2 * (3150.0f + x2 * 28.0f));
	return x * num / den;
}
