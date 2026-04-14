/*
 * reverb.cc
 *
 * C++ shim wrapping the MI Clouds plate reverb (adapted from
 * clouds/dsp/fx/reverb.h) for use from C code in the SWN firmware.
 *
 * Pattern follows src/plaits_shim.cpp.
 */

#include "reverb.h"
#include "plaits/dsp/fx/reverb.h"

// SRAM1 placement — same pattern as plaits_shim.cpp
#ifndef SRAM1_DATA
#define SRAM1_DATA __attribute__((section(".sram1data")))
#endif

// FxEngine<16384, FORMAT_12_BIT> needs a 16384-entry uint16_t buffer = 32 KB.
// Placed in SRAM1 (384 KB available, ~96 KB used by Plaits voices).
SRAM1_DATA alignas(4) static uint16_t reverb_buffer[16384];

static plaits::Reverb reverb;

extern "C" {

void Reverb_Init(void) {
    reverb.Init(reverb_buffer);
    // amount = 1.0: we feed pre-scaled send signals, output is pure wet
    reverb.set_amount(1.0f);
    // input_gain: drive into the tank. With correct normalisation the
    // send bus is ±1.0 at max. 0.5 gives ±1.0 peak in the tank (the
    // ×2 tap extraction makes the wet output ≈ unity relative to dry).
    reverb.set_input_gain(0.5f);
    // Parameters — overridden by preset once loaded
    reverb.set_time(0.6f);
    reverb.set_diffusion(0.625f);
    reverb.set_lp(0.7f);
}

void Reverb_Process(float *left, float *right, size_t size) {
    reverb.Process(left, right, size);
}

void Reverb_SetParams(float time, float diffusion, float lp) {
    reverb.set_time(time);
    reverb.set_diffusion(diffusion);
    reverb.set_lp(lp);
}

void Reverb_SetInputGain(float gain) {
    reverb.set_input_gain(gain);
}

} // extern "C"
