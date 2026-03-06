#ifndef PLAITS_DSP_ENGINE_STRING_MACHINE_ENGINE_OPTIMISED_H_
#define PLAITS_DSP_ENGINE_STRING_MACHINE_ENGINE_OPTIMISED_H_

#include "plaits/dsp/chords/chord_bank.h"
#include "plaits/dsp/engine/chord_engine.h"
#include "plaits/dsp/fx/biquad.h"
#include "plaits/dsp/fx/ensemble_optimised.h"

namespace plaits {

class StringMachineEngineOptimised : public Engine {
 public:
  StringMachineEngineOptimised() { }
  ~StringMachineEngineOptimised() { }
  
  virtual void Init(stmlib::BufferAllocator* allocator);
  virtual void Reset();
  virtual void LoadUserData(const uint8_t* user_data) { }
  virtual void Render(const EngineParameters& parameters,
      float* out,
      float* aux,
      size_t size,
      bool* already_enveloped);

 private:
  void ComputeRegistration(float registration, float* amplitudes);
  
  ChordBank chords_;
  
  EnsembleOptimised ensemble_;
  StringSynthOscillator divide_down_voice_[kChordNumNotes];
  stmlib::NaiveSvf svf_[2];
  
  float morph_lp_;
  float timbre_lp_;
  
  DISALLOW_COPY_AND_ASSIGN(StringMachineEngineOptimised);
};

}  // namespace plaits

#endif  // PLAITS_DSP_ENGINE_STRING_MACHINE_ENGINE_OPTIMISED_H_
