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
// 808 HH (Optimized version).

#ifndef PLAITS_DSP_DRUMS_HI_HAT_OPTIMISED_H_
#define PLAITS_DSP_DRUMS_HI_HAT_OPTIMISED_H_

#include <algorithm>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

#include "plaits/dsp/dsp.h"
#include "plaits/dsp/oscillator/oscillator.h"
#include "plaits/dsp/drums/hi_hat.h"
#include "plaits/dsp/fx/biquad.h"

namespace plaits {

template<
    typename MetallicNoiseSource,
    typename VCA,
    bool resonance,
    bool two_stage_envelope>
class HiHatOptimised {
 public:
  HiHatOptimised() { }
  ~HiHatOptimised() { }

  void Init() {
    envelope_ = 0.0f;
    noise_clock_ = 0.0f;
    noise_sample_ = 0.0f;
    sustain_gain_ = 0.0f;

    metallic_noise_.Init();
    noise_coloration_svf_.Init();
    hpf_.Init();
  }
  
  void Reset() {
    noise_coloration_svf_.Reset();
    hpf_.Reset();
  }
  
  void Render(
      bool sustain,
      bool trigger,
      float accent,
      float f0,
      float tone,
      float decay,
      float noisiness,
      float* temp_1,
      float* temp_2,
      float* out,
      size_t size) {
    const float envelope_decay = 1.0f - 0.003f * stmlib::SemitonesToRatio(
        -decay * 84.0f);
    const float cut_decay = 1.0f - 0.0025f * stmlib::SemitonesToRatio(
        -decay * 36.0f);
    
    if (trigger) {
      envelope_ = (1.5f + 0.5f * (1.0f - decay)) * (0.3f + 0.7f * accent);
    }

    // Render the metallic noise.
    metallic_noise_.Render(2.0f * f0, temp_1, temp_2, out, size);

    // Apply BPF on the metallic noise.
    float cutoff = 150.0f / kSampleRate * stmlib::SemitonesToRatio(
        tone * 72.0f);
    CONSTRAIN(cutoff, 0.0f, 16000.0f / kSampleRate);
    noise_coloration_svf_.set_f_q<stmlib::FREQUENCY_ACCURATE>(
        cutoff, resonance ? 3.0f + 3.0f * tone : 1.0f);
    noise_coloration_svf_.Process<stmlib::FILTER_MODE_BAND_PASS>(
        out, out, size);
    
    noisiness *= noisiness;
    float noise_f = f0 * (16.0f + 16.0f * (1.0f - noisiness));
    CONSTRAIN(noise_f, 0.0f, 0.5f);
    
    // Partially vectorized noise mix
    float clock = noise_clock_;
    float sample = noise_sample_;
    for (size_t i = 0; i < size; ++i) {
      clock += noise_f;
      if (clock >= 1.0f) {
        clock -= 1.0f;
        sample = stmlib::Random::GetFloat() - 0.5f;
      }
      out[i] += noisiness * (sample - out[i]);
    }
    noise_clock_ = clock;
    noise_sample_ = sample;

    // Apply VCA.
    stmlib::ParameterInterpolator sustain_gain_interp(
        &sustain_gain_,
        accent * decay,
        size);
        
    VCA vca;
    float env = envelope_;
    if (sustain) {
        for (size_t i = 0; i < size; ++i) {
            env *= env > 0.5f || !two_stage_envelope ? envelope_decay : cut_decay;
            out[i] = vca(out[i], sustain_gain_interp.Next());
        }
    } else {
        for (size_t i = 0; i < size; ++i) {
            env *= env > 0.5f || !two_stage_envelope ? envelope_decay : cut_decay;
            out[i] = vca(out[i], env);
        }
    }
    envelope_ = env;
    
    hpf_.set_f_q<stmlib::FREQUENCY_ACCURATE>(cutoff, 0.5f);
    hpf_.Process<stmlib::FILTER_MODE_HIGH_PASS>(out, out, size);
  }

 private:
  float envelope_;
  float noise_clock_;
  float noise_sample_;
  float sustain_gain_;

  MetallicNoiseSource metallic_noise_;
  SvfBlock noise_coloration_svf_;
  SvfBlock hpf_;
  
  DISALLOW_COPY_AND_ASSIGN(HiHatOptimised);
};

}  // namespace plaits

#endif  // PLAITS_DSP_DRUMS_HI_HAT_OPTIMISED_H_
