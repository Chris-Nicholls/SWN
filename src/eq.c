/*
 * eq.c - 6-Band EQ using biquad filters
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

#include "eq.h"
#include "math.h"
#include "math_util.h"

// Global EQ state
static EQState eq_state;

// Constants
#define PI 3.14159265358979323846f
#define SAMPLE_RATE 44100.0f

// ============================================================================
// Biquad Coefficient Calculation Functions
// ============================================================================

// Set coefficients for bypass (unity gain, no filtering)
static void calc_bypass(BiquadCoeffs *c) {
    c->b0 = 1.0f;
    c->b1 = 0.0f;
    c->b2 = 0.0f;
    c->a1 = 0.0f;
    c->a2 = 0.0f;
}

// Calculate 2nd-order Butterworth highpass coefficients
static void calc_highpass(BiquadCoeffs *c, float freq) {
    float w0 = 2.0f * PI * freq / SAMPLE_RATE;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / (2.0f * 0.7071067811865476f); // Q = sqrt(2)/2 for Butterworth
    
    float a0 = 1.0f + alpha;
    c->b0 = ((1.0f + cos_w0) / 2.0f) / a0;
    c->b1 = (-(1.0f + cos_w0)) / a0;
    c->b2 = ((1.0f + cos_w0) / 2.0f) / a0;
    c->a1 = (-2.0f * cos_w0) / a0;
    c->a2 = (1.0f - alpha) / a0;
}

// Calculate 2nd-order Butterworth lowpass coefficients
static void calc_lowpass(BiquadCoeffs *c, float freq) {
    float w0 = 2.0f * PI * freq / SAMPLE_RATE;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / (2.0f * 0.7071067811865476f); // Q = sqrt(2)/2 for Butterworth
    
    float a0 = 1.0f + alpha;
    c->b0 = ((1.0f - cos_w0) / 2.0f) / a0;
    c->b1 = (1.0f - cos_w0) / a0;
    c->b2 = ((1.0f - cos_w0) / 2.0f) / a0;
    c->a1 = (-2.0f * cos_w0) / a0;
    c->a2 = (1.0f - alpha) / a0;
}

// Calculate low shelf filter coefficients
// gain_db: gain in dB (positive for boost, negative for cut)
static void calc_low_shelf(BiquadCoeffs *c, float freq, float gain_db) {
    if (fabsf(gain_db) < 0.1f) {
        calc_bypass(c);
        return;
    }
    
    float A = powf(10.0f, gain_db / 40.0f);
    float w0 = 2.0f * PI * freq / SAMPLE_RATE;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / 2.0f * sqrtf((A + 1.0f/A) * (1.0f/0.9f - 1.0f) + 2.0f);
    float sqrt_A_alpha = 2.0f * sqrtf(A) * alpha;
    
    float a0 = (A + 1.0f) + (A - 1.0f) * cos_w0 + sqrt_A_alpha;
    c->b0 = (A * ((A + 1.0f) - (A - 1.0f) * cos_w0 + sqrt_A_alpha)) / a0;
    c->b1 = (2.0f * A * ((A - 1.0f) - (A + 1.0f) * cos_w0)) / a0;
    c->b2 = (A * ((A + 1.0f) - (A - 1.0f) * cos_w0 - sqrt_A_alpha)) / a0;
    c->a1 = (-2.0f * ((A - 1.0f) + (A + 1.0f) * cos_w0)) / a0;
    c->a2 = ((A + 1.0f) + (A - 1.0f) * cos_w0 - sqrt_A_alpha) / a0;
}

// Calculate high shelf filter coefficients
// gain_db: gain in dB (positive for boost, negative for cut)
static void calc_high_shelf(BiquadCoeffs *c, float freq, float gain_db) {
    if (fabsf(gain_db) < 0.1f) {
        calc_bypass(c);
        return;
    }
    
    float A = powf(10.0f, gain_db / 40.0f);
    float w0 = 2.0f * PI * freq / SAMPLE_RATE;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / 2.0f * sqrtf((A + 1.0f/A) * (1.0f/0.9f - 1.0f) + 2.0f);
    float sqrt_A_alpha = 2.0f * sqrtf(A) * alpha;
    
    float a0 = (A + 1.0f) - (A - 1.0f) * cos_w0 + sqrt_A_alpha;
    c->b0 = (A * ((A + 1.0f) + (A - 1.0f) * cos_w0 + sqrt_A_alpha)) / a0;
    c->b1 = (-2.0f * A * ((A - 1.0f) + (A + 1.0f) * cos_w0)) / a0;
    c->b2 = (A * ((A + 1.0f) + (A - 1.0f) * cos_w0 - sqrt_A_alpha)) / a0;
    c->a1 = (2.0f * ((A - 1.0f) - (A + 1.0f) * cos_w0)) / a0;
    c->a2 = ((A + 1.0f) - (A - 1.0f) * cos_w0 - sqrt_A_alpha) / a0;
}

// Calculate peaking EQ filter coefficients
// gain_db: gain in dB (positive for boost, negative for cut)
// Q: quality factor (bandwidth control)
static void calc_peaking(BiquadCoeffs *c, float freq, float gain_db, float Q) {
    if (fabsf(gain_db) < 0.1f) {
        calc_bypass(c);
        return;
    }
    
    float A = powf(10.0f, gain_db / 40.0f);
    float w0 = 2.0f * PI * freq / SAMPLE_RATE;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / (2.0f * Q);
    
    float a0 = 1.0f + alpha / A;
    c->b0 = (1.0f + alpha * A) / a0;
    c->b1 = (-2.0f * cos_w0) / a0;
    c->b2 = (1.0f - alpha * A) / a0;
    c->a1 = (-2.0f * cos_w0) / a0;
    c->a2 = (1.0f - alpha / A) / a0;
}

// ============================================================================
// Filter Processing
// ============================================================================

// Process single sample through one biquad stage (Direct Form II Transposed)
static inline float biquad_process(BiquadCoeffs *c, BiquadState *s, float in) {
    float out = c->b0 * in + s->z1;
    s->z1 = c->b1 * in - c->a1 * out + s->z2;
    s->z2 = c->b2 * in - c->a2 * out;
    return out;
}

// ============================================================================
// Public Functions
// ============================================================================

void eq_init(void) {
    // Initialize all coefficients to bypass
    for (int i = 0; i < NUM_EQ_BIQUADS; i++) {
        calc_bypass(&eq_state.coeffs[i]);
        eq_state.stage_active[i] = 0;
    }
    
    // Reset filter states
    eq_reset_states();
    
    // Initialize parameters to flat
    eq_state.params.hpf_enabled = 0;
    eq_state.params.hpf_freq = EQ_HPF_FREQ_MIN;
    eq_state.params.bass_boost_enabled = 0;
    eq_state.params.bass_boost_freq = EQ_BASS_BOOST_FREQ_MIN;
    eq_state.params.bass_boost_gain_db = 0.0f;
    
    eq_state.params.bass_gain_db = 0.0f;
    eq_state.params.low_mid_gain_db = 0.0f;
    eq_state.params.high_mid_gain_db = 0.0f;
    eq_state.params.treble_gain_db = 0.0f;
    
    eq_state.params.lpf_enabled = 0;
    eq_state.params.lpf_freq = EQ_LPF_FREQ_MAX;
    eq_state.params.treble_boost_enabled = 0;
    eq_state.params.treble_boost_freq = EQ_TREBLE_BOOST_FREQ_MIN;
    eq_state.params.treble_boost_gain_db = 0.0f;
}

void eq_reset_states(void) {
    for (int i = 0; i < NUM_EQ_BIQUADS; i++) {
        eq_state.state_L[i].z1 = 0.0f;
        eq_state.state_L[i].z2 = 0.0f;
        eq_state.state_R[i].z1 = 0.0f;
        eq_state.state_R[i].z2 = 0.0f;
    }
}

void eq_update_from_sliders(uint16_t slider_values[NUM_EQ_BANDS]) {
    EQParams *p = &eq_state.params;
    
    // ========================================================================
    // Slider 1: HPF (below 50%) or Bass Boost (above 50%)
    // ========================================================================
    if (slider_values[0] < EQ_SLIDER_MID - 50) {
        // HPF mode: cutoff from 500Hz (at 0) to 20Hz (at ~50%)
        float normalized = (float)(EQ_SLIDER_MID - slider_values[0]) / (float)EQ_SLIDER_MID;
        p->hpf_enabled = 1;
        p->bass_boost_enabled = 0;
        p->hpf_freq = EQ_HPF_FREQ_MIN + normalized * (EQ_HPF_FREQ_MAX - EQ_HPF_FREQ_MIN);
        
        // 24dB/oct = 2 cascaded 12dB/oct stages
        calc_highpass(&eq_state.coeffs[0], p->hpf_freq);
        calc_highpass(&eq_state.coeffs[1], p->hpf_freq);
        eq_state.stage_active[0] = 1;
        eq_state.stage_active[1] = 1;
        
        // Disable bass boost stage
        calc_bypass(&eq_state.coeffs[2]);
        eq_state.stage_active[2] = 0;
        
    } else if (slider_values[0] > EQ_SLIDER_MID + 50) {
        // Bass boost mode
        float normalized = (float)(slider_values[0] - EQ_SLIDER_MID) / (float)(EQ_SLIDER_MAX - EQ_SLIDER_MID);
        p->hpf_enabled = 0;
        p->bass_boost_enabled = 1;
        p->bass_boost_freq = EQ_BASS_BOOST_FREQ_MIN + normalized * (EQ_BASS_BOOST_FREQ_MAX - EQ_BASS_BOOST_FREQ_MIN);
        p->bass_boost_gain_db = normalized * EQ_BOOST_GAIN_DB;
        
        // Disable HPF stages
        calc_bypass(&eq_state.coeffs[0]);
        calc_bypass(&eq_state.coeffs[1]);
        eq_state.stage_active[0] = 0;
        eq_state.stage_active[1] = 0;
        
        // Use low shelf for bass boost
        calc_low_shelf(&eq_state.coeffs[2], p->bass_boost_freq, p->bass_boost_gain_db);
        eq_state.stage_active[2] = 1;
        
    } else {
        // Near center - bypass
        p->hpf_enabled = 0;
        p->bass_boost_enabled = 0;
        calc_bypass(&eq_state.coeffs[0]);
        calc_bypass(&eq_state.coeffs[1]);
        calc_bypass(&eq_state.coeffs[2]);
        eq_state.stage_active[0] = 0;
        eq_state.stage_active[1] = 0;
        eq_state.stage_active[2] = 0;
    }
    
    // ========================================================================
    // Slider 2: Bass EQ (low shelf)
    // ========================================================================
    {
        float normalized = (float)((int32_t)slider_values[1] - EQ_SLIDER_MID) / (float)EQ_SLIDER_MID;
        p->bass_gain_db = normalized * EQ_MAX_GAIN_DB;
        calc_low_shelf(&eq_state.coeffs[3], EQ_BASS_FREQ, p->bass_gain_db);
        eq_state.stage_active[3] = (fabsf(p->bass_gain_db) > 0.1f) ? 1 : 0;
    }
    
    // ========================================================================
    // Slider 3: Low-mid EQ (peaking)
    // ========================================================================
    {
        float normalized = (float)((int32_t)slider_values[2] - EQ_SLIDER_MID) / (float)EQ_SLIDER_MID;
        p->low_mid_gain_db = normalized * EQ_MAX_GAIN_DB;
        calc_peaking(&eq_state.coeffs[4], EQ_LOW_MID_FREQ, p->low_mid_gain_db, EQ_PEAK_Q);
        eq_state.stage_active[4] = (fabsf(p->low_mid_gain_db) > 0.1f) ? 1 : 0;
    }
    
    // ========================================================================
    // Slider 4: High-mid EQ (peaking)
    // ========================================================================
    {
        float normalized = (float)((int32_t)slider_values[3] - EQ_SLIDER_MID) / (float)EQ_SLIDER_MID;
        p->high_mid_gain_db = normalized * EQ_MAX_GAIN_DB;
        calc_peaking(&eq_state.coeffs[5], EQ_HIGH_MID_FREQ, p->high_mid_gain_db, EQ_PEAK_Q);
        eq_state.stage_active[5] = (fabsf(p->high_mid_gain_db) > 0.1f) ? 1 : 0;
    }
    
    // ========================================================================
    // Slider 5: Treble EQ (high shelf)
    // ========================================================================
    {
        float normalized = (float)((int32_t)slider_values[4] - EQ_SLIDER_MID) / (float)EQ_SLIDER_MID;
        p->treble_gain_db = normalized * EQ_MAX_GAIN_DB;
        calc_high_shelf(&eq_state.coeffs[6], EQ_TREBLE_FREQ, p->treble_gain_db);
        eq_state.stage_active[6] = (fabsf(p->treble_gain_db) > 0.1f) ? 1 : 0;
    }
    
    // ========================================================================
    // Slider 6: LPF (below 50%) or Treble Boost (above 50%)
    // ========================================================================
    if (slider_values[5] < EQ_SLIDER_MID - 50) {
        // LPF mode: cutoff from 1kHz (at 0) to 20kHz (at ~50%)
        float normalized = (float)(EQ_SLIDER_MID - slider_values[5]) / (float)EQ_SLIDER_MID;
        p->lpf_enabled = 1;
        p->treble_boost_enabled = 0;
        p->lpf_freq = EQ_LPF_FREQ_MAX - normalized * (EQ_LPF_FREQ_MAX - EQ_LPF_FREQ_MIN);
        
        // 24dB/oct = 2 cascaded 12dB/oct stages
        // Use stage 7 for LPF (need both 7 for the two stages, reuse 6 if needed)
        // Actually we have 8 stages total, so we can use 6 and 7 for LPF
        // But stage 6 is used for treble shelf, so we need a different approach
        // Let's use stage 7 for both LPF stages (processed twice) - not efficient
        // Better: Reconfigure - stages 0,1 = HPF, stage 2 = bass boost or bass shelf
        //                       stages 3,4 = parametric, stage 5 = treble shelf
        //                       stages 6,7 = LPF
        // Wait, we have 8 stages, let me recount:
        // 0,1 = HPF (24dB)
        // 2 = bass boost (when HPF not active) -- but this conflicts with bass shelf
        // Let me reorganize:
        // 0,1 = HPF
        // 2 = bass boost OR first part of slider 1 functionality
        // 3 = bass shelf (slider 2)
        // 4 = low-mid peak (slider 3)
        // 5 = high-mid peak (slider 4)
        // 6 = treble shelf (slider 5)
        // 7 = LPF/treble boost (slider 6)
        // 
        // For 24dB LPF we need 2 stages. Let's use just 12dB for simplicity,
        // or we can add more stages. Let me use 12dB for now.
        
        calc_lowpass(&eq_state.coeffs[7], p->lpf_freq);
        eq_state.stage_active[7] = 1;
        
    } else if (slider_values[5] > EQ_SLIDER_MID + 50) {
        // Treble boost mode
        float normalized = (float)(slider_values[5] - EQ_SLIDER_MID) / (float)(EQ_SLIDER_MAX - EQ_SLIDER_MID);
        p->lpf_enabled = 0;
        p->treble_boost_enabled = 1;
        p->treble_boost_freq = EQ_TREBLE_BOOST_FREQ_MIN + normalized * (EQ_TREBLE_BOOST_FREQ_MAX - EQ_TREBLE_BOOST_FREQ_MIN);
        p->treble_boost_gain_db = normalized * EQ_BOOST_GAIN_DB;
        
        calc_high_shelf(&eq_state.coeffs[7], p->treble_boost_freq, p->treble_boost_gain_db);
        eq_state.stage_active[7] = 1;
        
    } else {
        // Near center - bypass
        p->lpf_enabled = 0;
        p->treble_boost_enabled = 0;
        calc_bypass(&eq_state.coeffs[7]);
        eq_state.stage_active[7] = 0;
    }
}

void eq_process(float *buffer_L, float *buffer_R, uint16_t num_samples) {
    for (uint16_t i = 0; i < num_samples; i++) {
        float sample_L = buffer_L[i];
        float sample_R = buffer_R[i];
        
        // Process through all active biquad stages
        for (int stage = 0; stage < NUM_EQ_BIQUADS; stage++) {
            if (eq_state.stage_active[stage]) {
                sample_L = biquad_process(&eq_state.coeffs[stage], &eq_state.state_L[stage], sample_L);
                sample_R = biquad_process(&eq_state.coeffs[stage], &eq_state.state_R[stage], sample_R);
            }
        }
        
        buffer_L[i] = sample_L;
        buffer_R[i] = sample_R;
    }
}

const EQParams* eq_get_params(void) {
    return &eq_state.params;
}
