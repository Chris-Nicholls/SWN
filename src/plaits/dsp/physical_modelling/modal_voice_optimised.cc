#include "plaits/dsp/physical_modelling/modal_voice_optimised.h"

#include <algorithm>

#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

#include "plaits/dsp/noise/dust.h"

namespace plaits {

using namespace std;
using namespace stmlib;

void ModalVoiceOptimised::Init() {
  excitation_filter_.Init();
  resonator_.Init(0.015f, kMaxNumModesOptimised);
}

void ModalVoiceOptimised::Render(
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
    size_t size) {
  const float density = brightness * brightness;
  
  brightness += 0.25f * accent * (1.0f - brightness);
  damping += 0.25f * accent * (1.0f - damping);
  
  const float range = sustain ? 36.0f : 60.0f;
  const float f = sustain ? 4.0f * f0 : 2.0f * f0;
  const float cutoff = min(
      f * SemitonesToRatio((brightness * (2.0f - brightness) - 0.5f) * range),
      0.499f);
  const float q = sustain ? 0.7f : 1.5f;
  
  // Synthesize excitation signal.
  if (sustain) {
    const float dust_f = 0.00005f + 0.99995f * density * density;
    for (size_t i = 0; i < size; ++i) {
      temp[i] = Dust(dust_f) * (4.0f - dust_f * 3.0f) * accent;
    }
  } else {
    fill(&temp[0], &temp[size], 0.0f);
    if (trigger) {
      const float attenuation = 1.0f - damping * 0.5f;
      const float amplitude = (0.12f + 0.08f * accent) * attenuation;
      temp[0] = amplitude * SemitonesToRatio(cutoff * cutoff * 24.0f) / cutoff;
    }
  }
  const float one = 1.0f;
  excitation_filter_.Process<FILTER_MODE_LOW_PASS, false>(
      &cutoff, &q, &one, temp, aux, size);
  
  resonator_.Process(f0, structure, brightness, damping, aux, out, size);
}

}  // namespace plaits
