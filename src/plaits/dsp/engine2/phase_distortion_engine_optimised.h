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
// Phase distortion and phase modulation with an asymmetric triangle as the
// modulator.

#ifndef PLAITS_DSP_ENGINE2_PHASE_DISTORTION_ENGINE_OPTIMISED_H_
#define PLAITS_DSP_ENGINE2_PHASE_DISTORTION_ENGINE_OPTIMISED_H_

#include "plaits/dsp/engine/engine.h"

namespace plaits {

class PhaseDistortionEngineOptimised : public Engine {
 public:
  PhaseDistortionEngineOptimised() { }
  ~PhaseDistortionEngineOptimised() { }

  virtual void Init(stmlib::BufferAllocator* allocator);
  virtual void Reset();
  virtual void LoadUserData(const uint8_t* user_data) { }
  virtual void Render(const EngineParameters& parameters,
      float* out,
      float* aux,
      size_t size,
      bool* already_enveloped);

 private:
  float master_phase_;
  float slave_phase_;
  float slave_phase_2_;

  float previous_f0_;
  float previous_modulator_f_;
  float previous_pw_;
  float previous_amount_;

  float next_sample_;
  float next_sample_2_;
  
  DISALLOW_COPY_AND_ASSIGN(PhaseDistortionEngineOptimised);
};

}  // namespace plaits

#endif  // PLAITS_DSP_ENGINE2_PHASE_DISTORTION_ENGINE_OPTIMISED_H_
