#include "plaits/dsp/engine2/string_machine_engine_optimised.h"

#include <algorithm>

#include "plaits/resources.h"

namespace plaits {

using namespace std;
using namespace stmlib;

void StringMachineEngineOptimised::Init(BufferAllocator* allocator) {
  for (int i = 0; i < kChordNumNotes; ++i) {
    divide_down_voice_[i].Init();
  }
  chords_.Init(allocator);
  morph_lp_ = 0.0f;
  timbre_lp_ = 0.0f;
  svf_[0].Init();
  svf_[1].Init();
  ensemble_.Init(allocator->Allocate<float>(1024));
}

void StringMachineEngineOptimised::Reset() {
  chords_.Reset();
  ensemble_.Reset();
  svf_[0].Reset();
  svf_[1].Reset();
}

extern const float registrations[11][kChordNumHarmonics * 2];

void StringMachineEngineOptimised::ComputeRegistration(
    float registration,
    float* amplitudes) {
  registration *= (11 - 1.001f);
  MAKE_INTEGRAL_FRACTIONAL(registration);
  
  for (int i = 0; i < kChordNumHarmonics * 2; ++i) {
    float a = registrations[registration_integral][i];
    float b = registrations[registration_integral + 1][i];
    amplitudes[i] = a + (b - a) * registration_fractional;
  }
}

void StringMachineEngineOptimised::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  ONE_POLE(morph_lp_, parameters.morph, 0.1f);
  ONE_POLE(timbre_lp_, parameters.timbre, 0.1f);

  chords_.set_chord(parameters.harmonics);

  float harmonics[kChordNumHarmonics * 2 + 2];
  float registration = max(morph_lp_, 0.0f);
  ComputeRegistration(registration, harmonics);
  harmonics[kChordNumHarmonics * 2] = 0.0f;

  // Render string/organ sound.
  fill(&out[0], &out[size], 0.0f);
  fill(&aux[0], &aux[size], 0.0f);
  const float f0 = NoteToFrequency(parameters.note) * 0.998f;
  for (int note = 0; note < kChordNumNotes; ++note) {
    const float note_f0 = f0 * chords_.ratio(note);
    float divide_down_gain = 4.0f - note_f0 * 32.0f;
    CONSTRAIN(divide_down_gain, 0.0f, 1.0f);
    divide_down_voice_[note].Render(
        note_f0,
        harmonics,
        0.25f * divide_down_gain,
        note & 1 ? aux : out,
        size);
  }
  
  // Pass through VCF.
  const float cutoff = 2.2f * f0 * SemitonesToRatio(120.0f * parameters.timbre);
  svf_[0].set_f_q<FREQUENCY_DIRTY>(cutoff, 1.0f);
  svf_[1].set_f_q<FREQUENCY_DIRTY>(cutoff * 1.5f, 1.0f);

  // Mixdown with optimized filtering.
  svf_[0].Process<FILTER_MODE_LOW_PASS>(out, out, size);
  svf_[1].Process<FILTER_MODE_LOW_PASS>(aux, aux, size);
  
  for (size_t i = 0; i < size; ++i) {
    const float l = out[i];
    const float r = aux[i];
    out[i] = 0.66f * l + 0.33f * r;
    aux[i] = 0.66f * r + 0.33f * l;
  }

  // Ensemble FX.
  const float amount = fabsf(parameters.timbre - 0.5f) * 2.0f;
  const float depth = 0.35f + 0.65f * parameters.timbre;
  ensemble_.set_amount(amount);
  ensemble_.set_depth(depth);
  ensemble_.Process(out, aux, size);
}

}  // namespace plaits
