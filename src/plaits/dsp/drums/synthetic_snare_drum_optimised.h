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
// Naive snare drum model (two modulated oscillators + filtered noise). Optimized version.

#ifndef PLAITS_DSP_DRUMS_SYNTHETIC_SNARE_DRUM_OPTIMISED_H_
#define PLAITS_DSP_DRUMS_SYNTHETIC_SNARE_DRUM_OPTIMISED_H_

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

#include "plaits/dsp/dsp.h"
#include "plaits/dsp/fx/biquad.h"

namespace plaits {

class SyntheticSnareDrumOptimised {
 public:
  SyntheticSnareDrumOptimised() { }
  ~SyntheticSnareDrumOptimised() { }

  void Init() {
    phase_[0] = 0.0f;
    phase_[1] = 0.0f;
    drum_amplitude_ = 0.0f;
    snare_amplitude_ = 0.0f;
    fm_ = 0.0f;
    hold_counter_ = 0;
    sustain_gain_ = 0.0f;

    drum_lp_.Init();
    snare_hp_.Init();
    snare_lp_.Init();
  }

  void Reset() {
    snare_lp_.Reset();
  }
  
  inline float DistortedSine(float phase) {
    float triangle = (phase < 0.5f ? phase : 1.0f - phase) * 4.0f - 1.3f;
    return 2.0f * triangle / (1.0f + fabsf(triangle));
  }
  
  void Render(
      bool sustain,
      bool trigger,
      float accent,
      float f0,
      float fm_amount,
      float decay,
      float snappy,
      float* out,
      size_t size) {
    const float decay_xt = decay * (1.0f + decay * (decay - 1.0f));
    fm_amount *= fm_amount;
    const float drum_decay = 1.0f - 1.0f / (0.015f * kSampleRate) * \
        stmlib::SemitonesToRatio(
           -decay_xt * 72.0f - fm_amount * 12.0f + snappy * 7.0f);
    const float snare_decay = 1.0f - 1.0f / (0.01f * kSampleRate) * \
        stmlib::SemitonesToRatio(-decay * 60.0f - snappy * 7.0f);
    const float fm_decay = 1.0f - 1.0f / (0.007f * kSampleRate);
    
    snappy = snappy * 1.1f - 0.05f;
    CONSTRAIN(snappy, 0.0f, 1.0f);
    
    const float drum_level = stmlib::Sqrt(1.0f - snappy);
    const float snare_level = stmlib::Sqrt(snappy);
    
    const float snare_f_min = std::min(10.0f * f0, 0.5f);
    const float snare_f_max = std::min(35.0f * f0, 0.5f);

    snare_hp_.set_f<stmlib::FREQUENCY_FAST>(snare_f_min);
    snare_lp_.set_f_q<stmlib::FREQUENCY_FAST>(snare_f_max, 0.5f + 2.0f * snappy);
    drum_lp_.set_f<stmlib::FREQUENCY_FAST>(3.0f * f0);
    
    if (trigger) {
      snare_amplitude_ = drum_amplitude_ = 0.3f + 0.7f * accent;
      fm_ = 1.0f;
      phase_[0] = phase_[1] = 0.0f;
      hold_counter_ = static_cast<int>((0.04f + decay * 0.03f) * kSampleRate);
    }
    
    stmlib::ParameterInterpolator sustain_gain_interp(&sustain_gain_, accent * decay, size);
    
    float noise_buffer[kMaxBlockSize];
    float snare_amplitude_buffer[kMaxBlockSize];
    float fm_buffer[kMaxBlockSize];

    for (size_t i = 0; i < size; ++i) {
      if (sustain) {
        snare_amplitude_ = sustain_gain_interp.Next();
        drum_amplitude_ = snare_amplitude_;
        fm_ = 0.0f;
      } else {
        size_t remaining = size - i - 1;
        drum_amplitude_ *= (drum_amplitude_ > 0.03f || !(remaining & 1)) ? drum_decay : 1.0f;
        if (hold_counter_) {
          --hold_counter_;
        } else {
          snare_amplitude_ *= snare_decay;
        }
        fm_ *= fm_decay;
      }
      snare_amplitude_buffer[i] = snare_amplitude_;
      fm_buffer[i] = fm_;

      float reset_noise = 0.0f;
      float reset_noise_amount = (0.125f - f0) * 8.0f;
      CONSTRAIN(reset_noise_amount, 0.0f, 1.0f);
      reset_noise_amount *= reset_noise_amount;
      reset_noise_amount *= fm_amount;
      reset_noise += phase_[0] > 0.5f ? -1.0f : 1.0f;
      reset_noise += phase_[1] > 0.5f ? -1.0f : 1.0f;
      reset_noise *= reset_noise_amount * 0.025f;

      float f = f0 * (1.0f + fm_amount * (4.0f * fm_));
      phase_[0] += f;
      phase_[1] += f * 1.47f;
      if (reset_noise_amount > 0.1f) {
        if (phase_[0] >= 1.0f + reset_noise) phase_[0] = 1.0f - phase_[0];
        if (phase_[1] >= 1.0f + reset_noise) phase_[1] = 1.0f - phase_[1];
      } else {
        if (phase_[0] >= 1.0f) phase_[0] -= 1.0f;
        if (phase_[1] >= 1.0f) phase_[1] -= 1.0f;
      }
      
      float drum = -0.1f;
      drum += DistortedSine(phase_[0]) * 0.60f;
      drum += DistortedSine(phase_[1]) * 0.25f;
      drum *= drum_amplitude_ * drum_level;
      out[i] = drum_lp_.Process<stmlib::FILTER_MODE_LOW_PASS>(drum);
      
      noise_buffer[i] = stmlib::Random::GetFloat();
    }

    // Process snare noise in blocks of 2 for consistency
    const size_t sub_block_size = 2;
    for (size_t i = 0; i < size; i += sub_block_size) {
        size_t current_size = std::min(sub_block_size, size - i);
        snare_lp_.Process<stmlib::FILTER_MODE_LOW_PASS>(noise_buffer + i, noise_buffer + i, current_size);
        for (size_t j = 0; j < current_size; ++j) {
            float snare = snare_hp_.Process<stmlib::FILTER_MODE_HIGH_PASS>(noise_buffer[i + j]);
            snare = (snare + 0.1f) * (snare_amplitude_buffer[i + j] + fm_buffer[i + j]) * snare_level;
            out[i + j] += snare;
        }
    }
  }

 private:
  float phase_[2];
  float drum_amplitude_;
  float snare_amplitude_;
  float fm_;
  float sustain_gain_;
  int hold_counter_;
  
  stmlib::OnePole drum_lp_;
  stmlib::OnePole snare_hp_;
  SvfBlock snare_lp_;
  
  DISALLOW_COPY_AND_ASSIGN(SyntheticSnareDrumOptimised);
};

}  // namespace plaits

#endif  // PLAITS_DSP_DRUMS_SYNTHETIC_SNARE_DRUM_OPTIMISED_H_
