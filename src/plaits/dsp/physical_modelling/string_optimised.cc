// Copyright 2016 Emilie Gillet.
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
// Comb filter / KS string. "Lite" version of the implementation used in Rings.

#include "plaits/dsp/physical_modelling/string_optimised.h"

#include <cmath>

#include "stmlib/dsp/dsp.h"
#include "stmlib/dsp/parameter_interpolator.h"
#include "stmlib/dsp/units.h"
#include "stmlib/utils/random.h"

#include "plaits/dsp/dsp.h"
#include "plaits/resources.h"

namespace plaits {
  
using namespace std;
using namespace stmlib;

void StringOptimised::Init(stmlib::BufferAllocator* allocator) {
  string_.Init(allocator->Allocate<float>(kDelayLineSizeOptimised));
  stretch_.Init(allocator->Allocate<float>(kDelayLineSizeOptimised / 4));
  Reset();
}

void StringOptimised::Reset() {
  string_.Reset();
  stretch_.Reset();
  iir_damping_filter_.Init();
  dc_blocker_.Init(0.999f);
  dispersion_noise_ = 0.0f;
  curved_bridge_ = 0.0f;
  src_phase_ = 0.0f;
  delay_ = 100.0f;
  out_sample_[0] = out_sample_[1] = 0.0f;
}

void StringOptimised::Process(
    float f0,
    float non_linearity_amount,
    float brightness,
    float damping,
    const float* in,
    float* out,
    size_t size) {
  if (non_linearity_amount <= 0.0f) {
    ProcessInternal<STRING_NON_LINEARITY_CURVED_BRIDGE_OPTIMISED>(
        f0, -non_linearity_amount, brightness, damping, in, out, size);
  } else {
    ProcessInternal<STRING_NON_LINEARITY_DISPERSION_OPTIMISED>(
        f0, non_linearity_amount, brightness, damping, in, out, size);
  }
}

template<StringNonLinearityOptimised non_linearity>
void StringOptimised::ProcessInternal(
    float f0,
    float non_linearity_amount,
    float brightness,
    float damping,
    const float* __restrict__ in,
    float* __restrict__ out,
    size_t size) {
  float delay = 1.0f / f0;
  CONSTRAIN(delay, 4.0f, kDelayLineSizeOptimised - 4.0f);

  float damping_cutoff = min(
      12.0f + damping * damping * 60.0f + brightness * 24.0f,
      84.0f);
  float damping_f = min(f0 * SemitonesToRatio(damping_cutoff), 0.499f);
  
  // Crossfade to infinite decay.
  if (damping >= 0.95f) {
    float to_infinite = 20.0f * (damping - 0.95f);
    brightness += to_infinite * (1.0f - brightness);
    damping_f += to_infinite * (0.4999f - damping_f);
    damping_cutoff += to_infinite * (128.0f - damping_cutoff);
  }
  
  iir_damping_filter_.set_f_q<FREQUENCY_FAST>(damping_f, 0.5f);
  
  float damping_compensation = Interpolate(lut_svf_shift, damping_cutoff, 1.0f);
  
  // Linearly interpolate delay time.
  ParameterInterpolator delay_modulation(
      &delay_, delay * damping_compensation, size);
  
  float stretch_point = non_linearity_amount * (2.0f - non_linearity_amount) * 0.225f;
  float stretch_correction = (160.0f / kSampleRate) * delay;
  CONSTRAIN(stretch_correction, 1.0f, 2.1f);
  
  float noise_amount_sqrt = non_linearity_amount > 0.75f
      ? 4.0f * (non_linearity_amount - 0.75f)
      : 0.0f;
  float noise_amount = noise_amount_sqrt * noise_amount_sqrt * 0.1f;
  float noise_filter = 0.06f + 0.94f * brightness * brightness;
  
  float bridge_curving_sqrt = non_linearity_amount;
  float bridge_curving = bridge_curving_sqrt * bridge_curving_sqrt * 0.01f;
  
  float ap_gain = -0.618f * non_linearity_amount / (0.15f + fabsf(non_linearity_amount));

  float svf_s1 = iir_damping_filter_.state_1();
  float svf_s2 = iir_damping_filter_.state_2();
  const float svf_g = iir_damping_filter_.g();
  const float svf_r = iir_damping_filter_.r();
  const float svf_h = iir_damping_filter_.h();
  
  float dc_x = dc_blocker_.x();
  float dc_y = dc_blocker_.y();
  const float dc_pole = dc_blocker_.pole();
  
  while (size--) {
    float delay = delay_modulation.Next();
    float s = 0.0f;
    
    if (non_linearity == STRING_NON_LINEARITY_DISPERSION_OPTIMISED) {
      float noise = Random::GetFloat() - 0.5f;
      ONE_POLE(dispersion_noise_, noise, noise_filter)
      delay *= 1.0f + dispersion_noise_ * noise_amount;
    } else {
      delay *= 1.0f - curved_bridge_ * bridge_curving;
    }
    
    if (non_linearity == STRING_NON_LINEARITY_DISPERSION_OPTIMISED) {
      float ap_delay = delay * stretch_point;
      float main_delay = delay - ap_delay * (0.408f - stretch_point * 0.308f) * stretch_correction;
      if (ap_delay >= 4.0f && main_delay >= 4.0f) {
        s = string_.Read(main_delay);
        s = stretch_.Allpass(s, ap_delay, ap_gain);
      } else {
        s = string_.ReadHermite(delay);
      }
    } else {
      s = string_.ReadHermite(delay);
    }
    
    if (non_linearity == STRING_NON_LINEARITY_CURVED_BRIDGE_OPTIMISED) {
      float value = fabsf(s) - 0.025f;
      float sign = s > 0.0f ? 1.0f : -1.5f;
      curved_bridge_ = (fabsf(value) + value) * sign;
    }
  
    s += *in++;
    if (s < -20.0f) s = -20.0f;
    else if (s > 20.0f) s = 20.0f;
    
    // DC Blocker
    float old_x = dc_x;
    dc_x = s;
    s = dc_y = dc_y * dc_pole + dc_x - old_x;
    
    // SVF
    float hp = (s - svf_r * svf_s1 - svf_g * svf_s1 - svf_s2) * svf_h;
    float bp = svf_g * hp + svf_s1;
    svf_s1 = svf_g * hp + bp;
    float lp = svf_g * bp + svf_s2;
    svf_s2 = svf_g * bp + lp;
    s = lp;
    
    string_.Write(s);
    *out++ += s;
  }
  
  iir_damping_filter_.set_states(svf_s1, svf_s2);
  dc_blocker_.set_states(dc_x, dc_y);
}

}  // namespace plaits
