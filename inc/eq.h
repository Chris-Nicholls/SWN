/*
 * eq.h - 6-Band EQ using biquad filters
 *
 * Author: Chris Nicholls
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * See http://creativecommons.org/licenses/MIT/ for more information.
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

#include <stm32f7xx.h>
#include "globals.h"

// Number of EQ bands (one per slider)
#define NUM_EQ_BANDS 6

// Number of biquad stages: 2 for HPF + 2 for LPF + 4 for shelves/peaks = 8
#define NUM_EQ_BIQUADS 8

// EQ frequency ranges
#define EQ_HPF_FREQ_MIN     20.0f      // HPF minimum frequency (Hz)
#define EQ_HPF_FREQ_MAX     500.0f     // HPF maximum frequency (Hz)
#define EQ_BASS_BOOST_FREQ_MIN  60.0f  // Bass boost minimum frequency (Hz)
#define EQ_BASS_BOOST_FREQ_MAX  200.0f // Bass boost maximum frequency (Hz)

#define EQ_LPF_FREQ_MIN     1000.0f    // LPF minimum frequency (Hz)
#define EQ_LPF_FREQ_MAX     20000.0f   // LPF maximum frequency (Hz)
#define EQ_TREBLE_BOOST_FREQ_MIN 4000.0f  // Treble boost minimum frequency (Hz)
#define EQ_TREBLE_BOOST_FREQ_MAX 12000.0f // Treble boost maximum frequency (Hz)

// Fixed center frequencies for parametric bands
#define EQ_BASS_FREQ        120.0f     // Bass shelf frequency (Hz)
#define EQ_LOW_MID_FREQ     500.0f     // Low-mid center frequency (Hz)
#define EQ_HIGH_MID_FREQ    2000.0f    // High-mid center frequency (Hz)
#define EQ_TREBLE_FREQ      8000.0f    // Treble shelf frequency (Hz)

// EQ gain ranges
#define EQ_MAX_GAIN_DB      12.0f      // Maximum gain for standard bands (dB)
#define EQ_BOOST_GAIN_DB    6.0f       // Maximum gain for boost modes (dB)

// Q factor for peaking filters
#define EQ_PEAK_Q           1.0f

// Slider midpoint (50% = 2048 on 12-bit ADC)
#define EQ_SLIDER_MID       2048
#define EQ_SLIDER_MAX       4095

// Biquad coefficient structure (5 coefficients per stage)
// Transfer function: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
typedef struct {
    float b0, b1, b2;  // Numerator coefficients
    float a1, a2;      // Denominator coefficients (a0 = 1, normalized)
} BiquadCoeffs;

// Biquad filter state (2 delay elements per stage for DF2T)
typedef struct {
    float z1, z2;      // Delay elements
} BiquadState;

// EQ parameters derived from sliders
typedef struct {
    // Slider 1: HPF or bass boost
    uint8_t  hpf_enabled;
    float    hpf_freq;
    uint8_t  bass_boost_enabled;
    float    bass_boost_freq;
    float    bass_boost_gain_db;
    
    // Slider 2: Bass shelf
    float    bass_gain_db;
    
    // Slider 3: Low-mid peaking
    float    low_mid_gain_db;
    
    // Slider 4: High-mid peaking
    float    high_mid_gain_db;
    
    // Slider 5: Treble shelf
    float    treble_gain_db;
    
    // Slider 6: LPF or treble boost
    uint8_t  lpf_enabled;
    float    lpf_freq;
    uint8_t  treble_boost_enabled;
    float    treble_boost_freq;
    float    treble_boost_gain_db;
} EQParams;

// Complete EQ state for stereo processing
typedef struct {
    // Coefficients for all biquad stages
    // [0-1]: HPF (2 stages for 24dB/oct)
    // [2]:   Bass boost/shelf
    // [3]:   Low-mid peak
    // [4]:   High-mid peak
    // [5]:   Treble shelf
    // [6-7]: LPF (2 stages for 24dB/oct) or treble boost (1 stage)
    BiquadCoeffs coeffs[NUM_EQ_BIQUADS];
    
    // Filter states for left and right channels
    BiquadState state_L[NUM_EQ_BIQUADS];
    BiquadState state_R[NUM_EQ_BIQUADS];
    
    // Current parameters
    EQParams params;
    
    // Flags for which stages are active
    uint8_t stage_active[NUM_EQ_BIQUADS];
} EQState;

// Initialize EQ state to flat response
void eq_init(void);

// Update EQ coefficients based on slider values (0-4095)
// Called when sliders change in EQ mode
void eq_update_from_sliders(uint16_t slider_values[NUM_EQ_BANDS]);

// Process audio through EQ (in-place, stereo)
// buffer_L and buffer_R are arrays of MONO_BUFSZ samples
void eq_process(float *buffer_L, float *buffer_R, uint16_t num_samples);


// Get current EQ parameters (for LED display)
const EQParams* eq_get_params(void);

// Reset filter states (call when enabling EQ to avoid clicks)
void eq_reset_states(void);
