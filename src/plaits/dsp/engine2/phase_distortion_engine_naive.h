#ifndef PLAITS_DSP_ENGINE_PHASE_DISTORTION_ENGINE_NAIVE_H_
#define PLAITS_DSP_ENGINE_PHASE_DISTORTION_ENGINE_NAIVE_H_

#include "plaits/dsp/engine/engine.h"

namespace plaits {
  
class PhaseDistortionEngineNaive : public Engine {
 public:
  PhaseDistortionEngineNaive() { }
  ~PhaseDistortionEngineNaive() { }
  
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

  DISALLOW_COPY_AND_ASSIGN(PhaseDistortionEngineNaive);
};

}  // namespace plaits

#endif  // PLAITS_DSP_ENGINE_PHASE_DISTORTION_ENGINE_NAIVE_H_
