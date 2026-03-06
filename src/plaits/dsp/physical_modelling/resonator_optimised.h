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

#ifndef PLAITS_DSP_PHYSICAL_MODELLING_RESONATOR_OPTIMISED_H_
#define PLAITS_DSP_PHYSICAL_MODELLING_RESONATOR_OPTIMISED_H_

#include "plaits/dsp/fx/biquad.h"

namespace plaits {

const int kMaxNumModesOptimised = 16;
const int kModeBatchSizeOptimised = 8;

// We render 4 modes simultaneously since there are enough registers to hold
// all state variables.
template<int batch_size>
class ResonatorSvfOptimised {
 public:
  ResonatorSvfOptimised() { }
  ~ResonatorSvfOptimised() { }
  
  void Init() {
    for (int i = 0; i < batch_size; ++i) {
      biquad_[i].Init();
    }
  }
  
  template<stmlib::FilterMode mode, bool add>
  void Process(
      const float* f,
      const float* q,
      const float* gain,
      const float* in,
      float* out,
      size_t size) {
    if (!add) {
      std::fill(&out[0], &out[size], 0.0f);
    }

    for (int i = 0; i < batch_size; ++i) {
      biquad_[i].template set_f_q<stmlib::FREQUENCY_FAST>(f[i], q[i], mode);
      biquad_[i].template ProcessAdd<mode>(in, out, size, gain[i]);
    }
  }
  
 private:
  Biquad biquad_[batch_size];
  
  DISALLOW_COPY_AND_ASSIGN(ResonatorSvfOptimised);
};

class ResonatorOptimised {
 public:
  ResonatorOptimised() { }
  ~ResonatorOptimised() { }
  
  void Init(float position, int resolution);
  void Process(
      float f0,
      float structure,
      float brightness,
      float damping,
      const float* in,
      float* out,
      size_t size);
  
 private:
  int resolution_;
  
  float mode_amplitude_[kMaxNumModesOptimised];
  ResonatorSvfOptimised<kModeBatchSizeOptimised> mode_filters_[kMaxNumModesOptimised / kModeBatchSizeOptimised];
  
  DISALLOW_COPY_AND_ASSIGN(ResonatorOptimised);
};

}  // namespace plaits

#endif  // PLAITS_DSP_PHYSICAL_MODELLING_RESONATOR_OPTIMISED_H_
