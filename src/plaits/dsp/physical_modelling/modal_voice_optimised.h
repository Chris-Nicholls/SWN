#ifndef PLAITS_DSP_PHYSICAL_MODELLING_MODAL_VOICE_OPTIMISED_H_
#define PLAITS_DSP_PHYSICAL_MODELLING_MODAL_VOICE_OPTIMISED_H_

#include "plaits/dsp/physical_modelling/resonator_optimised.h"

namespace plaits {

class ModalVoiceOptimised {
 public:
  ModalVoiceOptimised() { }
  ~ModalVoiceOptimised() { }
  
  void Init();
  void Render(
      bool sustain,
      bool trigger,
      float accent,
      float f0,
      float structure,
      float brightness,
      float damping,
      float* temp,
      float* out,
      float* aux,
      size_t size);
  
 private:
  ResonatorSvfOptimised<1> excitation_filter_;
  ResonatorOptimised resonator_;
  
  DISALLOW_COPY_AND_ASSIGN(ModalVoiceOptimised);
};

}  // namespace plaits

#endif  // PLAITS_DSP_PHYSICAL_MODELLING_MODAL_VOICE_OPTIMISED_H_
