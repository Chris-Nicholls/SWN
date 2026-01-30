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
// 808 snare drum model, revisited. Optimized with block-processed filters.

#ifndef PLAITS_DSP_DRUMS_ANALOG_SNARE_DRUM_OPTIMISED_H_
#define PLAITS_DSP_DRUMS_ANALOG_SNARE_DRUM_OPTIMISED_H_

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

#include "plaits/dsp/dsp.h"
#include "plaits/dsp/oscillator/sine_oscillator.h"
#include "plaits/dsp/fx/biquad.h"

namespace plaits {

class AnalogSnareDrumOptimised {
 public:
  AnalogSnareDrumOptimised() { }
  ~AnalogSnareDrumOptimised() { }

  static const int kNumModes = 5;

  void Init() {
    pulse_remaining_samples_ = 0;
    pulse_ = 0.0f;
    pulse_height_ = 0.0f;
    pulse_lp_ = 0.0f;
    noise_envelope_ = 0.0f;
    sustain_gain_ = 0.0f;

    for (int i = 0; i < kNumModes; ++i) {
      resonator_[i].Init();
      oscillator_[i].Init();
    }
    noise_filter_.Init();
  }

  void Reset() {
    for (int i = 0; i < kNumModes; ++i) {
      resonator_[i].Reset();
    }
    noise_filter_.Reset();
  }
  
  void Render(
      bool sustain,
      bool trigger,
      float accent,
      float f0,
      float tone,
      float decay,
      float snappy,
      float* out,
      size_t size) {
    const float decay_xt = decay * (1.0f + decay * (decay - 1.0f));
    const int kTriggerPulseDuration = 1.0e-3f * kSampleRate;
    const float kPulseDecayTime = 0.1e-3f * kSampleRate;
    const float q = 2000.0f * stmlib::SemitonesToRatio(decay_xt * 84.0f);
    const float noise_envelope_decay = 1.0f - 0.0017f * \
        stmlib::SemitonesToRatio(-decay * (50.0f + snappy * 10.0f));
    const float exciter_leak = snappy * (2.0f - snappy) * 0.1f;
    
    snappy = snappy * 1.1f - 0.05f;
    CONSTRAIN(snappy, 0.0f, 1.0f);
    
    if (trigger) {
      pulse_remaining_samples_ = kTriggerPulseDuration;
      pulse_height_ = 3.0f + 7.0f * accent;
      noise_envelope_ = 2.0f;
    }
    
    static const float kModeFrequencies[kNumModes] = {
        1.00f,
        2.00f,
        3.18f,
        4.16f,
        5.62f};
    
    float f[kNumModes];
    float gain[kNumModes];
    
    for (int i = 0; i < kNumModes; ++i) {
      f[i] = std::min(f0 * kModeFrequencies[i], 0.499f);
      resonator_[i].set_f_q<stmlib::FREQUENCY_FAST>(
          f[i],
          1.0f + f[i] * (i == 0 ? q : q * 0.25f));
    }
    
    if (tone < 0.666667f) {
      // 808-style (2 modes)
      tone *= 1.5f;
      gain[0] = 1.5f + (1.0f - tone) * (1.0f - tone) * 4.5f;
      gain[1] = 2.0f * tone + 0.15f;
      std::fill(&gain[2], &gain[kNumModes], 0.0f);
    } else {
      // What the 808 could have been if there were extra modes!
      tone = (tone - 0.666667f) * 3.0f;
      gain[0] = 1.5f - tone * 0.5f;
      gain[1] = 2.15f - tone * 0.7f;
      for (int i = 2; i < kNumModes; ++i) {
        gain[i] = tone;
        tone *= tone;
      }
    }

    float f_noise = f0 * 16.0f;
    CONSTRAIN(f_noise, 0.0f, 0.499f);
    noise_filter_.set_f_q<stmlib::FREQUENCY_FAST>(
        f_noise, 1.0f + f_noise * 1.5f);
        
    stmlib::ParameterInterpolator sustain_gain_interp(&sustain_gain_, accent * decay, size);

    float excitation_0[kMaxBlockSize];
    float excitation_others[kMaxBlockSize];
    float noise_buffer[kMaxBlockSize];
    float sustain_gain_values[kMaxBlockSize];

    for (size_t i = 0; i < size; ++i) {
      float pulse = 0.0f;
      if (pulse_remaining_samples_) {
        --pulse_remaining_samples_;
        pulse = pulse_remaining_samples_ ? pulse_height_ : pulse_height_ - 1.0f;
        pulse_ = pulse;
      } else {
        pulse_ *= 1.0f - 1.0f / kPulseDecayTime;
        pulse = pulse_;
      }
      ONE_POLE(pulse_lp_, pulse, 0.75f);
      
      excitation_0[i] = (pulse - pulse_lp_) + 0.006f * pulse;
      excitation_others[i] = 0.026f * pulse;

      float sustain_gain_value = sustain_gain_interp.Next();
      sustain_gain_values[i] = sustain_gain_value;
      
      float noise = 2.0f * stmlib::Random::GetFloat() - 1.0f;
      if (noise < 0.0f) noise = 0.0f;
      noise_envelope_ *= noise_envelope_decay;
      
      noise_buffer[i] = noise * (sustain ? sustain_gain_value : noise_envelope_) * snappy * 2.0f;
    }

    float shell[kMaxBlockSize];
    std::fill(&shell[0], &shell[size], 0.0f);

    if (sustain) {
      for (int i = 0; i < kNumModes; ++i) {
        if (gain[i] > 0.0f) {
           for (size_t j = 0; j < size; ++j) {
             shell[j] += gain[i] * oscillator_[i].Next(f[i]) * sustain_gain_values[j] * 0.25f;
           }
        }
      }
    } else {
      float mode_out[kMaxBlockSize];
      for (int i = 0; i < kNumModes; ++i) {
        if (gain[i] > 0.0f) {
          resonator_[i].Process<stmlib::FILTER_MODE_BAND_PASS>(
              i == 0 ? excitation_0 : excitation_others, mode_out, size);
          for (size_t j = 0; j < size; ++j) {
            shell[j] += gain[i] * (mode_out[j] + (i == 0 ? excitation_0[j] : excitation_others[j]) * exciter_leak);
          }
        }
      }
    }

    noise_filter_.Process<stmlib::FILTER_MODE_BAND_PASS>(noise_buffer, noise_buffer, size);

    for (size_t i = 0; i < size; ++i) {
      float s = stmlib::SoftClip(shell[i]);
      out[i] = noise_buffer[i] + s * (1.0f - snappy);
    }
  }

 private:
  int pulse_remaining_samples_;
  float pulse_;
  float pulse_height_;
  float pulse_lp_;
  float noise_envelope_;
  float sustain_gain_;
  
  SvfBlock resonator_[kNumModes];
  SvfBlock noise_filter_;

  // Replace the resonators in "free running" (sustain) mode.
  SineOscillator oscillator_[kNumModes];
  
  DISALLOW_COPY_AND_ASSIGN(AnalogSnareDrumOptimised);
};
  
}  // namespace plaits

#endif  // PLAITS_DSP_DRUMS_ANALOG_SNARE_DRUM_OPTIMISED_H_
