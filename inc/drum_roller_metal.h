/*
 * drum_roller_metal.h
 *
 * The _metal() generator shared by the ROLLER hats, ride and crash
 * (drum_prototype/render_roller_style_drums.py): six detuned squares
 * through a bandpass + highpass pair, optionally blended with highpassed
 * noise, under one linear-attack/exponential-decay envelope.
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

#include "drum_roller_common.h"

#define DRUM_ROLLER_METAL_PARTIALS 6

typedef struct DrumRollerMetal {
	float phase[DRUM_ROLLER_METAL_PARTIALS];
	float inc[DRUM_ROLLER_METAL_PARTIALS];

	RollerBiquad bp;
	RollerBiquad hp;
	RollerBiquad noise_hp;
	RollerNoise  rng;

	RollerEnv env;
	float noise_mix;
	float dec;
} DrumRollerMetal;

void drum_roller_metal_init(DrumRollerMetal *m, uint32_t seed);

/* Voicing: square-bank transposition plus the two fixed filter corners.
 * Safe to call from set_other/trigger; leaves filter memory intact. */
void drum_roller_metal_config(DrumRollerMetal *m, float bp_freq, float hp_freq,
                              float ratio, float noise_mix);

/* Amplitude contour for the next hit; dec also sets the hard gate. */
void drum_roller_metal_set_env(DrumRollerMetal *m, float v, float dec);

void drum_roller_metal_reset(DrumRollerMetal *m);

float drum_roller_metal_tick(DrumRollerMetal *m, float t);
