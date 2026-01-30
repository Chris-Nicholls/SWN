#ifndef PLAITS_DSP_FX_ENSEMBLE_OPTIMISED_H_
#define PLAITS_DSP_FX_ENSEMBLE_OPTIMISED_H_

#include <algorithm>
#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "plaits/dsp/oscillator/sine_oscillator.h"

namespace plaits {

class EnsembleOptimised {
 public:
  EnsembleOptimised() { }
  ~EnsembleOptimised() { }
  
  void Init(float* buffer) {
    buffer_ = buffer;
    phase_1_ = 0;
    phase_2_ = 0;
    write_ptr_ = 0;
    std::fill(&buffer_[0], &buffer_[1024], 0.0f);
  }
  
  void Reset() {
    std::fill(&buffer_[0], &buffer_[1024], 0.0f);
    write_ptr_ = 0;
  }
  
  void Process(float* left, float* right, size_t size) {
    const float dry_amount = 1.0f - amount_ * 0.5f;
    const float a = depth_ * 160.0f;
    const float b = depth_ * 16.0f;
    const uint32_t one_third = 1417339207UL;
    const uint32_t two_third = 2834678415UL;
    
    int32_t ptr = write_ptr_;
    
    for (size_t i = 0; i < size; ++i) {
      ptr = (ptr - 1) & 1023;
      buffer_[ptr] = left[i];
      buffer_[(ptr + 512) & 1023] = right[i];
      
      phase_1_ += 67289;
      phase_2_ += 589980;
      
      const float mod_1 = SineRaw(phase_1_) * a + SineRaw(phase_2_) * b + 192.0f;
      const float mod_2 = SineRaw(phase_1_ + one_third) * a + SineRaw(phase_2_ + one_third) * b + 192.0f;
      const float mod_3 = SineRaw(phase_1_ + two_third) * a + SineRaw(phase_2_ + two_third) * b + 192.0f;

      // Left output
      float l_wet = 0.0f;
      l_wet += Interpolate(buffer_, ptr, mod_1, 0) * 0.33f;
      l_wet += Interpolate(buffer_, ptr, mod_2, 0) * 0.33f;
      l_wet += Interpolate(buffer_, ptr, mod_3, 512) * 0.33f;
      
      // Right output
      float r_wet = 0.0f;
      r_wet += Interpolate(buffer_, ptr, mod_1, 512) * 0.33f;
      r_wet += Interpolate(buffer_, ptr, mod_2, 512) * 0.33f;
      r_wet += Interpolate(buffer_, ptr, mod_3, 0) * 0.33f;

      left[i] = l_wet * amount_ + left[i] * dry_amount;
      right[i] = r_wet * amount_ + right[i] * dry_amount;
    }
    write_ptr_ = ptr;
  }
  
  inline void set_amount(float amount) {
    amount_ = amount;
  }
  
  inline void set_depth(float depth) {
    depth_ = depth;
  }
  
 private:
  inline float Interpolate(const float* buffer, int32_t ptr, float offset, int32_t base) {
    MAKE_INTEGRAL_FRACTIONAL(offset);
    const int32_t i0 = (ptr + offset_integral + base) & 1023;
    const int32_t i1 = (i0 + 1) & 1023;
    const float a = buffer[i0];
    const float b = buffer[i1];
    return a + (b - a) * offset_fractional;
  }

  float* buffer_;
  float amount_;
  float depth_;
  int32_t write_ptr_;
  uint32_t phase_1_;
  uint32_t phase_2_;
  
  DISALLOW_COPY_AND_ASSIGN(EnsembleOptimised);
};

}  // namespace plaits

#endif  // PLAITS_DSP_FX_ENSEMBLE_OPTIMISED_H_
