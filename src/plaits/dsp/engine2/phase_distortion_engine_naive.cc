#include "plaits/dsp/engine2/phase_distortion_engine_naive.h"

#include <algorithm>
#include <cmath>

#include "stmlib/dsp/parameter_interpolator.h"
#include "plaits/dsp/oscillator/sine_oscillator.h"
#include "plaits/resources.h"

namespace plaits {

using namespace std;
using namespace stmlib;

void PhaseDistortionEngineNaive::Init(BufferAllocator* allocator) {
  master_phase_ = 0.0f;
  slave_phase_ = 0.0f;
  slave_phase_2_ = 0.0f;
  
  previous_f0_ = 0.0f;
  previous_modulator_f_ = 0.01f;
  previous_pw_ = 0.5f;
  previous_amount_ = 0.0f;
  
  next_sample_ = 0.0f;
  next_sample_2_ = 0.0f;
}

void PhaseDistortionEngineNaive::Reset() {
  master_phase_ = 0.0f;
  slave_phase_ = 0.0f;
  slave_phase_2_ = 0.0f;
  next_sample_ = 0.0f;
  next_sample_2_ = 0.0f;
  
  previous_f0_ = 0.0f;
  previous_modulator_f_ = 0.01f;
  previous_pw_ = 0.5f;
  previous_amount_ = 0.0f;
}

void PhaseDistortionEngineNaive::Render(
    const EngineParameters& parameters,
    float* out,
    float* aux,
    size_t size,
    bool* already_enveloped) {
  const float f0 = 0.5f * NoteToFrequency(parameters.note);
  const float modulator_f = min(0.25f, f0 * SemitonesToRatio(Interpolate(
      lut_fm_frequency_quantizer,
      parameters.harmonics,
      128.0f)));
  const float pw = 0.5f + parameters.morph * 0.49f;
  const float amount = 8.0f * parameters.timbre * parameters.timbre * \
      (1.0f - modulator_f * 3.8f);

  ParameterInterpolator f0_mod(&previous_f0_, f0, 2 * size);
  ParameterInterpolator modulator_f_mod(&previous_modulator_f_, modulator_f, 2 * size);
  ParameterInterpolator pw_mod(&previous_pw_, pw, 2 * size);
  ParameterInterpolator amount_mod(&previous_amount_, amount, 2 * size);

  for (size_t i = 0; i < size; ++i) {
    float out_sum = 0.0f;
    float aux_sum = 0.0f;
    
    // 2x Oversampling
    for (int j = 0; j < 2; ++j) {
      const float cf0 = f0_mod.Next();
      const float cmod_f = modulator_f_mod.Next();
      const float cpw = pw_mod.Next();
      const float camount = amount_mod.Next();
      const float skew = 2.0f * fabsf(cpw - 0.5f);

      float reset_time = 0.0f;
      bool reset = false;

      // 1. Master Phase Increment and Reset Detection (Sync)
      master_phase_ += cf0;
      if (master_phase_ >= 1.0f) {
        master_phase_ -= 1.0f;
        reset = true;
        reset_time = master_phase_ / cf0;
      }

      // 2. Slave Phase Increment and Reset
      slave_phase_ += cmod_f;
      if (reset) {
        slave_phase_ = reset_time * cmod_f;
      }
      if (slave_phase_ >= 1.0f) {
        slave_phase_ -= 1.0f;
      }

      slave_phase_2_ += cmod_f;
      if (slave_phase_2_ >= 1.0f) {
        slave_phase_2_ -= 1.0f;
      }

      // 3. Compute Modulator Samples (Naive Triangle)
      auto get_tri = [&](float p, float pw) {
        return p < pw ? (p / pw) : (1.0f - (p - pw) / (1.0f - pw));
      };
      
      float tri = next_sample_;
      float tri2 = next_sample_2_;
      
      next_sample_ = get_tri(slave_phase_, cpw);
      next_sample_2_ = get_tri(slave_phase_2_, cpw);

      // 4. Apply window and asymmetry
      float p_synced = master_phase_;
      const float w = 4.0f * (1.0f - p_synced) * p_synced;
      const float window = w * (2.0f - w);
      tri *= window;
      
      const float p2 = p_synced * p_synced;
      p_synced += (p2 * p2 - p_synced) * skew;
      
      float shaper_phase = p_synced + camount * tri;
      out_sum += Sine(shaper_phase + 0.25f);
      
      // 5. Free-running (Aux)
      float aux_phase = master_phase_ + camount * tri2;
      aux_sum += Sine(aux_phase + 0.25f);
    }
    
    out[i] = 0.5f * out_sum;
    aux[i] = 0.5f * aux_sum;
  }
}

}  // namespace plaits
