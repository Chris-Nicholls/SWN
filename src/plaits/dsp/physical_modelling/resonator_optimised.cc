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
// Resonator, taken from Rings' code but with fixed position.

#include "plaits/dsp/physical_modelling/resonator_optimised.h"

#include <algorithm>

#include "stmlib/dsp/cosine_oscillator.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/units.h"

#include "plaits/resources.h"

namespace plaits {

using namespace std;
using namespace stmlib;

void ResonatorOptimised::Init(float position, int resolution) {
  resolution_ = min(resolution, kMaxNumModesOptimised);
  
  CosineOscillator amplitudes;
  amplitudes.Init<COSINE_OSCILLATOR_APPROXIMATE>(position);
  
  for (int i = 0; i < resolution_; ++i) {
    mode_amplitude_[i] = amplitudes.Next() * 0.25f;
  }
  
  for (int i = 0; i < kMaxNumModesOptimised / kModeBatchSizeOptimised; ++i) {
    mode_filters_[i].Init();
  }
}

inline float NthHarmonicCompensationOptimised(int n, float s) {
    if (n <= 1) return 1.0f;

    // The sign of s never changes, so we pick the ratio once
    const float r = (s < 0.0f) ? 0.93f : 0.98f;
    
    // Geometric series sum: sf = 1 + s * (1 - r^(n-1)) / (1 - r)
    float pow_r = std::powf(r, static_cast<float>(n - 1));
    float sf = 1.0f + s * (1.0f - pow_r) / (1.0f - r);

    return 1.0f / sf;
}

void ResonatorOptimised::Process(
    float f0,
    float structure,
    float brightness,
    float damping,
    const float* in,
    float* out,
    size_t size) {
  float stiffness = Interpolate(lut_stiffness, structure, 64.0f);
  f0 *= NthHarmonicCompensationOptimised(3, stiffness);
  
  float harmonic = f0;
  float stretch_factor = 1.0f;
  float q_sqrt = SemitonesToRatio(damping * 79.7f);
  float q = 500.0f * q_sqrt * q_sqrt;
  brightness *= 1.0f - structure * 0.3f;
  brightness *= 1.0f - damping * 0.3f;
  float q_loss = brightness * (2.0f - brightness) * 0.85f + 0.15f;
  
  float mode_q[kModeBatchSizeOptimised];
  float mode_f[kModeBatchSizeOptimised];
  float mode_a[kModeBatchSizeOptimised];
  int batch_counter = 0;
  
  ResonatorSvfOptimised<kModeBatchSizeOptimised>* batch_processor = &mode_filters_[0];
  
  for (int i = 0; i < resolution_; ++i) {
    float mode_frequency = harmonic * stretch_factor;
    if (mode_frequency >= 0.499f) {
      mode_frequency = 0.499f;
    }
    const float mode_attenuation = 1.0f - mode_frequency * 2.0f;
    
    mode_f[batch_counter] = mode_frequency;
    mode_q[batch_counter] = 1.0f + mode_frequency * q;
    mode_a[batch_counter] = mode_amplitude_[i] * mode_attenuation;
    ++batch_counter;
    
    if (batch_counter == kModeBatchSizeOptimised) {
      batch_counter = 0;
      batch_processor->Process<FILTER_MODE_BAND_PASS, true>(
          mode_f,
          mode_q,
          mode_a,
          in,
          out,
          size);
      ++batch_processor;
    }
    
    stretch_factor += stiffness;
    if (stiffness < 0.0f) {
      // Make sure that the partials do not fold back into negative frequencies.
      stiffness *= 0.93f;
    } else {
      // This helps adding a few extra partials in the highest frequencies.
      stiffness *= 0.98f;
    }
    harmonic += f0;
    q *= q_loss;
  }
}

}  // namespace plaits
