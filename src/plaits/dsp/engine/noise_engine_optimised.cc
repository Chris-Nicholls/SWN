// Copyright 2016 Emilie Gillet.
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
// Clocked noise processed by a multimode filter (Optimized version).

#include "plaits/dsp/engine/noise_engine_optimised.h"
#include "stmlib/dsp/parameter_interpolator.h"

namespace plaits {

using namespace std;
using namespace stmlib;

void NoiseEngineOptimised::Init(BufferAllocator* allocator) {
  clocked_noise_[0].Init();
  clocked_noise_[1].Init();
  lp_hp_filter_.Init();
  bp_filter_[0].Init();
  bp_filter_[1].Init();

  previous_f0_ = 0.0f;
  previous_f1_ = 0.0f;
  previous_q_ = 0.0f;
  previous_mode_ = 0.0f;

  temp_buffer_ = allocator->Allocate<float>(kMaxBlockSize);
}

void NoiseEngineOptimised::Reset() {
  lp_hp_filter_.Reset();
  bp_filter_[0].Reset();
  bp_filter_[1].Reset();
}

void NoiseEngineOptimised::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  const float f0 = NoteToFrequency(parameters.note);
  const float f1 = NoteToFrequency(
      parameters.note + parameters.harmonics * 48.0f - 24.0f);
  const float clock_lowest_note = parameters.trigger & TRIGGER_UNPATCHED
      ? 0.0f
      : -24.0f;
  const float clock_f = NoteToFrequency(
      parameters.timbre * (128.0f - clock_lowest_note) + clock_lowest_note);
  const float q = 0.5f * SemitonesToRatio(parameters.morph * 120.0f);
  const bool sync = parameters.trigger & TRIGGER_RISING_EDGE;
  
  clocked_noise_[0].Render(sync, clock_f, aux, size);
  clocked_noise_[1].Render(sync, clock_f * f1 / f0, temp_buffer_, size);
  
  ParameterInterpolator f0_modulation(&previous_f0_, f0, size);
  ParameterInterpolator f1_modulation(&previous_f1_, f1, size);
  ParameterInterpolator q_modulation(&previous_q_, q, size);
  ParameterInterpolator mode_modulation(&previous_mode_, parameters.harmonics, size);
  
  // Use sub-blocks of 2 samples (24 updates per 48-sample block) for ultra accuracy.
  const size_t sub_block_size = 2;
  for (size_t i = 0; i < size; i += sub_block_size) {
    size_t current_size = std::min(sub_block_size, size - i);
    
    float f0_s = 0.0f, f1_s = 0.0f, q_s = 0.0f, mode_s = 0.0f;
    for (size_t j = 0; j < current_size; ++j) {
        f0_s = f0_modulation.Next();
        f1_s = f1_modulation.Next();
        q_s = q_modulation.Next();
        mode_s = mode_modulation.Next();
        
        float gain = 1.0f / Sqrt((0.5f + q_s) * 40.0f * f0_s);
        aux[i + j] *= gain;
        temp_buffer_[i + j] *= gain;
    }
    
    lp_hp_filter_.set_f_q<FREQUENCY_DIRTY>(f0_s, q_s);
    bp_filter_[0].set_f_q<FREQUENCY_DIRTY>(f0_s, q_s);
    bp_filter_[1].set_f_q<FREQUENCY_DIRTY>(f1_s, q_s);
    
    lp_hp_filter_.ProcessMultimodeLPtoHP(aux + i, out + i, current_size, mode_s);
    
    float bp0[sub_block_size];
    float bp1[sub_block_size];
    bp_filter_[0].Process<FILTER_MODE_BAND_PASS>(aux + i, bp0, current_size);
    bp_filter_[1].Process<FILTER_MODE_BAND_PASS>(temp_buffer_ + i, bp1, current_size);
    
    for (size_t j = 0; j < current_size; ++j) {
      aux[i + j] = bp0[j] + bp1[j];
    }
  }
}

}  // namespace plaits
