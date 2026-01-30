// Copyright 2021 Emilie Gillet.
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
// Virtual analog with VCF (Optimized version using SvfBlock).

#include "plaits/dsp/engine2/virtual_analog_vcf_engine_optimised.h"
#include "stmlib/dsp/parameter_interpolator.h"

namespace plaits {

using namespace stmlib;

void VirtualAnalogVCFEngineOptimised::Init(BufferAllocator* allocator) {
  oscillator_.Init();
  sub_oscillator_.Init();
  vcf_[0].Init();
  vcf_[1].Init();
  
  previous_sub_gain_ = 0.0f;
  previous_cutoff_ = 0.0f;
  previous_stage2_gain_ = 0.0f;
  previous_q_ = 0.0f;
  previous_gain_ = 0.0f;
}

void VirtualAnalogVCFEngineOptimised::Reset() {
  vcf_[0].Reset();
  vcf_[1].Reset();
}

void VirtualAnalogVCFEngineOptimised::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  const float f0 = NoteToFrequency(parameters.note);

  float shape = (parameters.morph - 0.25f) * 2.0f + 0.5f;
  CONSTRAIN(shape, 0.5f, 1.0f);

  float pw = (parameters.morph - 0.5f) * 2.0f + 0.5f;
  if (parameters.morph > 0.75f) {
    pw = 2.5f - parameters.morph * 2.0f;
  }
  CONSTRAIN(pw, 0.5f, 0.98f);
  
  float sub_gain_target = std::max(fabsf(parameters.morph - 0.5f) - 0.3f, 0.0f) * 5.0f;

  oscillator_.Render(f0, pw, shape, out, size);
  sub_oscillator_.Render(f0 * 0.501f, 0.5f, 1.0f, aux, size);
  
  const float cutoff_target = f0 * SemitonesToRatio((parameters.timbre - 0.2f) * 120.0f);
  float stage2_gain_target = 1.0f - (parameters.harmonics - 0.4f) * 4.0f;
  CONSTRAIN(stage2_gain_target, 0.0f, 1.0f);
  const float resonance = 2.667f * std::max(fabsf(parameters.harmonics - 0.5f) - 0.125f, 0.0f);
  const float resonance_sqr = resonance * resonance;
  const float q_target = resonance_sqr * resonance_sqr * 48.0f;
  float gain_target = (parameters.harmonics - 0.7f) + 0.85f;
  CONSTRAIN(gain_target, 0.7f - resonance_sqr * 0.3f, 1.0f);

  ParameterInterpolator sub_gain_modulation(&previous_sub_gain_, sub_gain_target, size);
  ParameterInterpolator cutoff_modulation(&previous_cutoff_, cutoff_target, size);
  ParameterInterpolator stage2_gain_modulation(&previous_stage2_gain_, stage2_gain_target, size);
  ParameterInterpolator q_modulation(&previous_q_, q_target, size);
  ParameterInterpolator gain_modulation(&previous_gain_, gain_target, size);
  
  // Use sub-blocks of 2 samples for high accuracy
  const size_t sub_block_size = 2;
  for (size_t i = 0; i < size; i += sub_block_size) {
    size_t current_size = std::min(sub_block_size, size - i);
    
    float cutoff_s = 0.0f, q_s = 0.0f, stage2_gain_s = 0.0f, gain_s = 0.0f;
    for (size_t j = 0; j < current_size; ++j) {
        cutoff_s = std::min(cutoff_modulation.Next(), 0.25f);
        q_s = q_modulation.Next();
        stage2_gain_s = stage2_gain_modulation.Next();
        gain_s = gain_modulation.Next();
        
        // Input preparation (must be done per sample)
        out[i + j] = SoftClip((out[i + j] + aux[i + j] * sub_gain_modulation.Next()) * gain_s);
    }
    
    vcf_[0].set_f_q<FREQUENCY_FAST>(cutoff_s, 0.5f + q_s);
    vcf_[1].set_f_q<FREQUENCY_FAST>(cutoff_s, 0.5f + 0.025f * q_s);
    
    float lp0[sub_block_size];
    float hp0[sub_block_size];
    vcf_[0].Process<FILTER_MODE_LOW_PASS, FILTER_MODE_HIGH_PASS>(out + i, lp0, hp0, current_size);
    
    float lp1[sub_block_size];
    for (size_t j = 0; j < current_size; ++j) {
      lp0[j] = SoftClip(lp0[j] * gain_s);
    }
    
    vcf_[1].Process<FILTER_MODE_LOW_PASS>(lp0, lp1, current_size);
    
    for (size_t j = 0; j < current_size; ++j) {
      out[i + j] = lp0[j] + stage2_gain_s * (SoftClip(lp1[j]) - lp0[j]);
      aux[i + j] = SoftClip(hp0[j] * gain_s);
    }
  }
}

}  // namespace plaits
