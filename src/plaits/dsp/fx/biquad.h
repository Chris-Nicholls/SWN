#ifndef PLAITS_DSP_FX_BIQUAD_H_
#define PLAITS_DSP_FX_BIQUAD_H_

#include "arm_math.h"
#include "stmlib/dsp/filter.h"
#include <algorithm>

namespace plaits {

class Biquad {
 public:
  Biquad() { }
  ~Biquad() { }

  void Init() {
    std::fill(&state_[0], &state_[4], 0.0f);
    iir_.numStages = 1;
    iir_.pState = &state_[0];
    iir_.pCoeffs = &coeffs_[0];
  }

  template<stmlib::FilterMode mode>
  void Process(const float* in, float* out, size_t size) {
    if (size == 0) return;
    arm_biquad_cascade_df1_f32(&iir_, (float32_t*)in, out, (uint32_t)size);
  }

  template<stmlib::FilterMode mode>
  void ProcessAdd(const float* in, float* out, size_t size, float gain) {
    if (size == 0) return;
    float32_t temp[48];
    uint32_t block = std::min((size_t)48, size);
    arm_biquad_cascade_df1_f32(&iir_, (float32_t*)in, temp, block);
    for (size_t i = 0; i < block; ++i) {
      out[i] += temp[i] * gain;
    }
  }

  template<stmlib::FrequencyApproximation approximation>
  void set_f_q(float f, float q, stmlib::FilterMode mode = stmlib::FILTER_MODE_LOW_PASS) {
    float g = stmlib::OnePole::tan<approximation>(f);
    float r = 1.0f / q;
    float h = 1.0f / (1.0f + r * g + g * g);
    
    if (mode == stmlib::FILTER_MODE_LOW_PASS) {
      float g2 = g * g;
      coeffs_[0] = g2 * h;
      coeffs_[1] = 2.0f * coeffs_[0];
      coeffs_[2] = coeffs_[0];
    } else if (mode == stmlib::FILTER_MODE_BAND_PASS || mode == stmlib::FILTER_MODE_BAND_PASS_NORMALIZED) {
      coeffs_[0] = g * h;
      coeffs_[1] = 0.0f;
      coeffs_[2] = -coeffs_[0];
      if (mode == stmlib::FILTER_MODE_BAND_PASS_NORMALIZED) {
          coeffs_[0] *= r;
          coeffs_[2] *= r;
      }
    } else if (mode == stmlib::FILTER_MODE_HIGH_PASS) {
      coeffs_[0] = h;
      coeffs_[1] = -2.0f * h;
      coeffs_[2] = h;
    }
    coeffs_[3] = 2.0f * (1.0f - g * g) * h;
    coeffs_[4] = -(1.0f - r * g + g * g) * h;
  }

 private:
  arm_biquad_casd_df1_inst_f32 iir_;
  float32_t state_[4];
  float32_t coeffs_[5];
};

class SvfBlock {
 public:
  SvfBlock() { }
  ~SvfBlock() { }

  void Init() {
    s1_ = s2_ = 0.0f;
    set_f_q<stmlib::FREQUENCY_DIRTY>(0.01f, 100.0f);
  }

  void Reset() {
    s1_ = s2_ = 0.0f;
  }

  template<stmlib::FilterMode mode>
  void Process(const float* in, float* out, size_t size) {
    float s1 = s1_;
    float s2 = s2_;
    float g = g_;
    float r = r_;
    float h = h_;
    
    while (size--) {
      float hp = (*in++ - (r + g) * s1 - s2) * h;
      float bp = g * hp + s1;
      s1 = g * hp + bp;
      float lp = g * bp + s2;
      s2 = g * bp + lp;
      
      if (mode == stmlib::FILTER_MODE_LOW_PASS) {
        *out++ = lp;
      } else if (mode == stmlib::FILTER_MODE_BAND_PASS) {
        *out++ = bp;
      } else if (mode == stmlib::FILTER_MODE_BAND_PASS_NORMALIZED) {
        *out++ = bp * r;
      } else if (mode == stmlib::FILTER_MODE_HIGH_PASS) {
        *out++ = hp;
      }
    }
    s1_ = s1;
    s2_ = s2;
  }

  template<stmlib::FilterMode mode>
  void ProcessAdd(const float* in, float* out, size_t size, float gain) {
    float s1 = s1_;
    float s2 = s2_;
    float g = g_;
    float r = r_;
    float h = h_;
    
    while (size--) {
      float hp = (*in++ - (r + g) * s1 - s2) * h;
      float bp = g * hp + s1;
      s1 = g * hp + bp;
      float lp = g * bp + s2;
      s2 = g * bp + lp;
      
      float val = 0.0f;
      if (mode == stmlib::FILTER_MODE_LOW_PASS) {
        val = lp;
      } else if (mode == stmlib::FILTER_MODE_BAND_PASS) {
        val = bp;
      } else if (mode == stmlib::FILTER_MODE_BAND_PASS_NORMALIZED) {
        val = bp * r;
      } else if (mode == stmlib::FILTER_MODE_HIGH_PASS) {
        val = hp;
      }
      *out++ += val * gain;
    }
    s1_ = s1;
    s2_ = s2;
  }

  void ProcessMulti(const float* in, float* lp_out, float* bp_out, float* hp_out, size_t size) {
    float s1 = s1_;
    float s2 = s2_;
    float g = g_;
    float r = r_;
    float h = h_;
    
    while (size--) {
      float hp = (*in++ - (r + g) * s1 - s2) * h;
      float bp = g * hp + s1;
      s1 = g * hp + bp;
      float lp = g * bp + s2;
      s2 = g * bp + lp;
      
      if (lp_out) *lp_out++ = lp;
      if (bp_out) *bp_out++ = bp;
      if (hp_out) *hp_out++ = hp;
    }
    s1_ = s1;
    s2_ = s2;
  }

  void ProcessMultimodeLPtoHP(const float* in, float* out, size_t size, float mode) {
    float s1 = s1_;
    float s2 = s2_;
    float g = g_;
    float r = r_;
    float h = h_;
    float hp_gain = std::min(-mode * 2.0f + 1.0f, 0.0f);
    float bp_gain = 1.0f - 2.0f * fabsf(mode - 0.5f);
    float lp_gain = std::max(1.0f - mode * 2.0f, 0.0f);
    
    while (size--) {
      float hp = (*in++ - (r + g) * s1 - s2) * h;
      float bp = g * hp + s1;
      s1 = g * hp + bp;
      float lp = g * bp + s2;
      s2 = g * bp + lp;
      *out++ = hp_gain * hp + bp_gain * bp + lp_gain * lp;
    }
    s1_ = s1;
    s2_ = s2;
  }
  
  template<stmlib::FilterMode mode_1, stmlib::FilterMode mode_2>
  void Process(const float* in, float* out_1, float* out_2, size_t size) {
    float s1 = s1_;
    float s2 = s2_;
    float g = g_;
    float r = r_;
    float h = h_;
    
    while (size--) {
      float hp = (*in++ - (r + g) * s1 - s2) * h;
      float bp = g * hp + s1;
      s1 = g * hp + bp;
      float lp = g * bp + s2;
      s2 = g * bp + lp;
      
      if (mode_1 == stmlib::FILTER_MODE_LOW_PASS) *out_1++ = lp;
      else if (mode_1 == stmlib::FILTER_MODE_BAND_PASS) *out_1++ = bp;
      else if (mode_1 == stmlib::FILTER_MODE_HIGH_PASS) *out_1++ = hp;
      
      if (mode_2 == stmlib::FILTER_MODE_LOW_PASS) *out_2++ = lp;
      else if (mode_2 == stmlib::FILTER_MODE_BAND_PASS) *out_2++ = bp;
      else if (mode_2 == stmlib::FILTER_MODE_HIGH_PASS) *out_2++ = hp;
    }
    s1_ = s1;
    s2_ = s2;
  }

  template<stmlib::FrequencyApproximation approximation>
  void set_f_q(float f, float q) {
    g_ = stmlib::OnePole::tan<approximation>(f);
    r_ = 1.0f / q;
    h_ = 1.0f / (1.0f + r_ * g_ + g_ * g_);
  }

 private:
  float s1_, s2_;
  float g_, r_, h_;
  
  DISALLOW_COPY_AND_ASSIGN(SvfBlock);
};

}  // namespace plaits

#endif  // PLAITS_DSP_FX_BIQUAD_H_
