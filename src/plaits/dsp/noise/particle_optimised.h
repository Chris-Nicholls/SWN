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
// Random impulse train processed by a resonant filter.

#ifndef PLAITS_DSP_NOISE_PARTICLE_OPTIMISED_H_
#define PLAITS_DSP_NOISE_PARTICLE_OPTIMISED_H_

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/filter.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"
#include "plaits/dsp/dsp.h"
#include "plaits/dsp/fx/biquad.h"

namespace plaits {

class ParticleOptimised {
 public:
  ParticleOptimised() { }
  ~ParticleOptimised() { }

  inline void Init() {
    pre_gain_ = 0.0f;
    filter_.Init();
  }
  
  inline void Render(
      bool sync,
      float density,
      float gain,
      float frequency,
      float spread,
      float q,
      float* out,
      float* aux,
      size_t size) {
    float u = stmlib::Random::GetFloat();
    if (sync) {
      u = density;
    }
    
    float excitation[kMaxBlockSize];
    int first_pulse = -1;
    bool can_randomize = true;
    float f_new = 0.0f;
    
    for (size_t i = 0; i < size; ++i) {
      float s = 0.0f;
      if (u <= density) {
        s = u * gain;
        if (can_randomize) {
          first_pulse = i;
          const float u_rand = 2.0f * stmlib::Random::GetFloat() - 1.0f;
          f_new = std::min(stmlib::SemitonesToRatio(spread * u_rand) * frequency, 0.25f);
          pre_gain_ = 0.5f / stmlib::Sqrt(q * f_new * stmlib::Sqrt(density));
          can_randomize = false;
        }
      }
      excitation[i] = s;
      aux[i] += s;
      u = stmlib::Random::GetFloat();
    }

    if (first_pulse == -1) {
      // Input is all 0, scaling doesn't matter, but we must process for decay.
      filter_.ProcessAdd<stmlib::FILTER_MODE_BAND_PASS>(excitation, out, size, 1.0f);
    } else {
      // Samples before first pulse are 0, scaling doesn't matter.
      if (first_pulse > 0) {
        filter_.ProcessAdd<stmlib::FILTER_MODE_BAND_PASS>(excitation, out, first_pulse, 1.0f);
      }
      
      // Update filter with new coefficients
      filter_.set_f_q<stmlib::FREQUENCY_DIRTY>(f_new, q);
      
      // Scale the rest of the excitation (from first_pulse onwards) by the NEW pre_gain_
      for (size_t i = first_pulse; i < size; ++i) {
        excitation[i] *= pre_gain_;
      }
      
      // Process the rest with gain 1.0 (since scaling is already applied)
      filter_.ProcessAdd<stmlib::FILTER_MODE_BAND_PASS>(
          &excitation[first_pulse], 
          &out[first_pulse], 
          size - first_pulse, 
          1.0f);
    }
  }
  
 private:
  float pre_gain_;
  SvfBlock filter_;
  
  DISALLOW_COPY_AND_ASSIGN(ParticleOptimised);
};

}  // namespace plaits

#endif  // PLAITS_DSP_NOISE_PARTICLE_OPTIMISED_H_
