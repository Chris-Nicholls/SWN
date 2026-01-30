// Granular diffuser (ARM optimized block-based version).

#ifndef PLAITS_DSP_FX_DIFFUSER_OPTIMISED_H_
#define PLAITS_DSP_FX_DIFFUSER_OPTIMISED_H_

#include "stmlib/stmlib.h"
#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/cosine_oscillator.h"
#include <algorithm>

namespace plaits {

class DiffuserOptimised {
 public:
  DiffuserOptimised() { }
  ~DiffuserOptimised() { }
  
  void Init(uint16_t* buffer) {
    buffer_ = buffer;
    std::fill(&buffer_[0], &buffer_[8192], 0);
    write_ptr_ = 0;
    lfo_.Init<stmlib::COSINE_OSCILLATOR_APPROXIMATE>(0.3f / 48000.0f * 32.0f);
    lp_decay_ = 0.0f;
    lfo_last_ = 1.0f; // Cosine at phase 0
  }
  
  void Reset() {
    std::fill(&buffer_[0], &buffer_[8192], 0);
    write_ptr_ = 0;
  }
  
  void Process(float amount, float rt, float* in_out, size_t size) {
    const float kap = 0.625f;
    const float klp = 0.75f;
    
    // Delay lengths from original Diffuser
    const int d1 = 126;
    const int d2 = 180;
    const int d3 = 269;
    const int d4 = 444;
    const int d5 = 1653;
    const int d6 = 2010;
    const int d7 = 3411;
    
    // Base addresses from FxEngine::Reserve logic
    const int b1 = 0;
    const int b2 = b1 + d1 + 1;
    const int b3 = b2 + d2 + 1;
    const int b4 = b3 + d3 + 1;
    const int b5 = b4 + d4 + 1;
    const int b6 = b5 + d5 + 1;
    const int b7 = b6 + d6 + 1;
    
    const int mask = 8191;
    
    float lp = lp_decay_;
    uint16_t* buf = buffer_;
    int write_ptr = write_ptr_;
    
    for (size_t i = 0; i < size; ++i) {
      write_ptr = (write_ptr - 1) & mask;
      
      float acc = in_out[i];
      float lfo_val;
      if ((write_ptr & 31) == 0) {
        lfo_val = lfo_.Next();
        lfo_last_ = lfo_val;
      } else {
        lfo_val = lfo_last_;
      }
      
      // AP 1
      float read_val = Decompress(buf[(write_ptr + b1 + d1 - 1) & mask]);
      float ap_acc = acc + kap * read_val;
      buf[(write_ptr + b1) & mask] = Compress(ap_acc);
      acc = read_val - kap * ap_acc;
      
      // AP 2
      read_val = Decompress(buf[(write_ptr + b2 + d2 - 1) & mask]);
      ap_acc = acc + kap * read_val;
      buf[(write_ptr + b2) & mask] = Compress(ap_acc);
      acc = read_val - kap * ap_acc;
      
      // AP 3
      read_val = Decompress(buf[(write_ptr + b3 + d3 - 1) & mask]);
      ap_acc = acc + kap * read_val;
      buf[(write_ptr + b3) & mask] = Compress(ap_acc);
      acc = read_val - kap * ap_acc;
      
      // AP 4 (Modulated)
      float offset4 = 400.0f + 43.0f * lfo_val;
      MAKE_INTEGRAL_FRACTIONAL(offset4);
      float a = Decompress(buf[(write_ptr + offset4_integral + b4) & mask]);
      float b = Decompress(buf[(write_ptr + offset4_integral + b4 + 1) & mask]);
      read_val = a + (b - a) * offset4_fractional;
      ap_acc = acc + kap * read_val;
      buf[(write_ptr + b4) & mask] = Compress(ap_acc);
      acc = read_val - kap * ap_acc;
      
      // Delay + LP
      float offset7 = 3070.0f + 340.0f * lfo_val;
      MAKE_INTEGRAL_FRACTIONAL(offset7);
      a = Decompress(buf[(write_ptr + offset7_integral + b7) & mask]);
      b = Decompress(buf[(write_ptr + offset7_integral + b7 + 1) & mask]);
      read_val = a + (b - a) * offset7_fractional;
      acc += rt * read_val;
      
      lp += klp * (acc - lp);
      acc = lp;
      
      // AP 5
      read_val = Decompress(buf[(write_ptr + b5 + d5 - 1) & mask]);
      ap_acc = acc - kap * read_val;
      buf[(write_ptr + b5) & mask] = Compress(ap_acc);
      acc = read_val + kap * ap_acc;
      
      // AP 6
      read_val = Decompress(buf[(write_ptr + b6 + d6 - 1) & mask]);
      ap_acc = acc + kap * read_val;
      buf[(write_ptr + b6) & mask] = Compress(ap_acc);
      acc = read_val - kap * ap_acc;
      
      // Write to delay line head
      buf[(write_ptr + b7) & mask] = Compress(acc);
      
      // FxEngine writes accumulator * 2.0 to wet output
      float wet = acc * 2.0f;
      in_out[i] += amount * (wet - in_out[i]);
    }
    
    write_ptr_ = write_ptr;
    lp_decay_ = lp;
  }
  
 private:
  static inline float Decompress(uint16_t value) {
    return static_cast<float>(static_cast<int16_t>(value)) / 4096.0f;
  }
  
  static inline uint16_t Compress(float value) {
    return static_cast<uint16_t>(
        stmlib::Clip16(static_cast<int32_t>(value * 4096.0f)));
  }

  uint16_t* buffer_;
  int32_t write_ptr_;
  stmlib::CosineOscillator lfo_;
  float lfo_last_;
  float lp_decay_;
  
  DISALLOW_COPY_AND_ASSIGN(DiffuserOptimised);
};

}  // namespace plaits

#endif  // PLAITS_DSP_FX_DIFFUSER_OPTIMISED_H_
