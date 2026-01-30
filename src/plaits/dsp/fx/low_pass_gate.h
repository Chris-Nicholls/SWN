// Copyright 2014 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// 
// See http://creativecommons.org/licenses/MIT/ for more information.
//
// -----------------------------------------------------------------------------
//
// Approximative low pass gate.

#ifndef PLAITS_DSP_FX_LOW_PASS_GATE_H_
#define PLAITS_DSP_FX_LOW_PASS_GATE_H_

#include <algorithm>

#include "arm_math.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/parameter_interpolator.h"

namespace plaits {
  
class LowPassGate {
 public:
  LowPassGate() { }
  ~LowPassGate() { }
  
  void Init() {
    previous_gain_ = 0.0f;
    previous_hf_bleed_ = 0.0f;
    std::fill(&state_[0], &state_[4], 0.0f);
    iir_.numStages = 1;
    iir_.pState = &state_[0];
    iir_.pCoeffs = &coeffs_[0];
  }

  // Maps frequency and Q to Biquad coefficients matching the ZDF SVF response
  void ComputeCoefficients(float frequency, float resonance) {
    float g = stmlib::OnePole::tan<stmlib::FREQUENCY_DIRTY>(frequency);
    float r = 1.0f / resonance;
    float h = 1.0f / (1.0f + r * g + g * g);
    float g2 = g * g;
    
    // Low-pass Biquad coefficients (BLT of ZDF SVF)
    // CMSIS-DSP uses: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] + a1*y[n-1] + a2*y[n-2]
    coeffs_[0] = g2 * h;                   // b0
    coeffs_[1] = 2.0f * coeffs_[0];        // b1
    coeffs_[2] = coeffs_[0];               // b2
    coeffs_[3] = 2.0f * (1.0f - g2) * h;   // a1 (negated std a1)
    coeffs_[4] = -(1.0f - r * g + g2) * h; // a2 (negated std a2)
  }
  
  void Process(
      float gain,
      float frequency,
      float hf_bleed,
      float* in_out,
      size_t size) {
    if (size == 0) return;

    float32_t temp_orig[48]; // Stack buffer, max block size is 48
    size_t block_size = std::min(size, (size_t)48);

    stmlib::ParameterInterpolator gain_modulation(&previous_gain_, gain, block_size);
    stmlib::ParameterInterpolator hf_modulation(&previous_hf_bleed_, hf_bleed, block_size);
    ComputeCoefficients(frequency, 0.4f);

    // 1. Apply Gain and store original for HF bleed
    for (size_t i = 0; i < block_size; ++i) {
      temp_orig[i] = in_out[i] * gain_modulation.Next();
    }

    // 2. Optimized Biquad Filter
    arm_biquad_cascade_df1_f32(&iir_, temp_orig, in_out, block_size);

    // 3. Apply HF bleed: out = lp + (s - lp) * hf_bleed (interpolated)
    for (size_t i = 0; i < block_size; ++i) {
      float hf = hf_modulation.Next();
      in_out[i] = in_out[i] + (temp_orig[i] - in_out[i]) * hf;
    }
  }
  
  void Process(
      float gain,
      float frequency,
      float hf_bleed,
      float* in,
      short* out,
      size_t size,
      size_t stride) {
    if (size == 0) return;

    float32_t temp_f[48];
    float32_t temp_orig[48];
    size_t block_size = std::min(size, (size_t)48);

    stmlib::ParameterInterpolator gain_modulation(&previous_gain_, gain, block_size);
    stmlib::ParameterInterpolator hf_modulation(&previous_hf_bleed_, hf_bleed, block_size);
    ComputeCoefficients(frequency, 0.4f);

    for (size_t i = 0; i < block_size; ++i) {
      temp_orig[i] = in[i] * gain_modulation.Next();
    }

    arm_biquad_cascade_df1_f32(&iir_, temp_orig, temp_f, block_size);

    for (size_t i = 0; i < block_size; ++i) {
      float lp = temp_f[i];
      float s = temp_orig[i];
      float hf = hf_modulation.Next();
      *out = stmlib::Clip16(1 + static_cast<int32_t>(lp + (s - lp) * hf));
      out += stride;
    }
  }
  
 private:
  float previous_gain_;
  float previous_hf_bleed_;
  arm_biquad_casd_df1_inst_f32 iir_;
  float32_t state_[4];
  float32_t coeffs_[5];
  
  DISALLOW_COPY_AND_ASSIGN(LowPassGate);
};

}  // namespace plaits

#endif  // PLAITS_DSP_FX_LOW_PASS_GATE_H_
