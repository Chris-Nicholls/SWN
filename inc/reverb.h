/*
 * reverb.h
 *
 * C API for the MI Clouds plate reverb.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Reverb_Init(void);

// Process a block of audio in-place.
// left/right are float arrays of length `size`.
// With amount=1.0 (default), the output is pure wet reverb.
void Reverb_Process(float *left, float *right, size_t size);

// Update global reverb parameters (called each audio block or on param change).
// time:      decay length  [0, 1]  (0.35 = short room, 0.85 = long/infinite)
// diffusion: allpass smear [0, 1]  (0.625 = Clouds default)
// lp:        HF damping    [0, 1]  (0.7 = Clouds default; lower = darker)
void Reverb_SetParams(float time, float diffusion, float lp);

// Set the fixed internal tank drive (called once at init; not a user param).
void Reverb_SetInputGain(float gain);

#ifdef __cplusplus
}
#endif
