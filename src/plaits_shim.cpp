/*
 * plaits_shim.cpp
 *
 * Adapter between SWN C codebase and Plaits C++ engine
 */

#include "plaits_shim.h"
#include <string.h>

// Plaits Includes
#include "plaits/dsp/voice.h"
#include "plaits/dsp/dsp.h"
#include "stmlib/utils/buffer_allocator.h"

namespace {
inline float Clamp(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}
}

// SRAM1 Placement
#ifndef SRAM1_DATA
#define SRAM1_DATA __attribute__((section(".sram1data")))
#endif

// Global Plaits State
// We have 6 voices.
// We place them in SRAM1 to avoid exhausting DTCM RAM.
// Each voice needs its own memory for engine state (delay lines, etc.)
// Original Plaits uses ~16KB. We try to fit 6 voices. 
// 12KB * 6 = 72KB.
// UPDATE: 12KB causes crashes on some engines. Increasing to 16KB.
// 16KB * 6 = 96KB. Fits in SRAM1 (384KB).
#define PLAITS_VOICE_BUFFER_SIZE 16384
SRAM1_DATA alignas(32) char voice_buffers[6][PLAITS_VOICE_BUFFER_SIZE];
alignas(32)  plaits::Voice voices[6];

extern "C" {

void Plaits_Init(void) {
    stmlib::BufferAllocator allocator;
    
    for (int i=0; i<6; i++) {
        // Initialize allocator with the specific buffer for this voice
        allocator.Init(voice_buffers[i], PLAITS_VOICE_BUFFER_SIZE);
        
        // Initialize voice (assigns pointers within the buffer)
        voices[i].Init(&allocator);
    }
}

void Plaits_Render(uint8_t channel, PlaitsParams* params, float* out_buffer, int32_t size) {
    if (channel >= 6) return;
    if (size > plaits::kMaxBlockSize) return;  // Safety: kMaxBlockSize = 48

    static float temp_buffer[plaits::kMaxBlockSize]; // Buffer for the unused output (Aux or Main)

    plaits::Patch patch;
    
    patch.note = params->note; 
    patch.harmonics = Clamp(params->harmonics, 0.0f, 1.0f);
    patch.timbre = Clamp(params->timbre, 0.0f, 1.0f);
    patch.morph = Clamp(params->morph, 0.0f, 1.0f);
    patch.frequency_modulation_amount = Clamp(params->mod_freq, -1.0f, 1.0f);
    patch.timbre_modulation_amount = Clamp(params->mod_timbre, -1.0f, 1.0f);
    patch.morph_modulation_amount = Clamp(params->mod_morph, -1.0f, 1.0f);

    patch.engine = params->engine;
    patch.decay = params->lpg_decay;
    patch.lpg_colour = params->lpg_color;

    plaits::Modulations modulations;
    modulations.engine = 0.0f;
    modulations.note = 0.0f;
    modulations.frequency = params->mod_freq; 
    modulations.harmonics = params->mod_harmonics;
    modulations.timbre = params->mod_timbre;
    modulations.morph = params->mod_morph;
    modulations.trigger = params->trigger; // 0.0 to 1.0 (schmitt trigger inside)
    modulations.level = 1.0f;

    modulations.frequency_patched = false; 
    modulations.timbre_patched = false;
    modulations.morph_patched = false;
    
    // Modulate params
    
    // Enable trigger_patched only when we want the internal LPG and envelopes to work.
    // If false, the internal LPG is bypassed and the sound is constant (for SWN VCA to handle).
    modulations.trigger_patched = (params->use_internal_lpg > 0); 
    modulations.level_patched = false;

    // Optimization: Write directly to out_buffer based on mode, use temp for unused channel.
    if (params->output_mode == 1) {
        // Aux Mode: Write Aux to out_buffer, Main to temp
        voices[channel].Render(patch, modulations, temp_buffer, out_buffer, size);
    } else {
        // Main Mode: Write Main to out_buffer, Aux to temp
        voices[channel].Render(patch, modulations, out_buffer, temp_buffer, size);
    }
}

void Plaits_SetParams(uint8_t channel, PlaitsParams* params) {
    // This might not be needed if we pass params every Render call.
    // Keeping for compatibility with plan.
}

}

// LPG Wrapper
#include "plaits/dsp/fx/low_pass_gate.h"
#include "plaits/dsp/envelope.h"

plaits::LowPassGate lpg[6];
plaits::LPGEnvelope lpg_env[6];

// Helper for Decay mapping (using stmlib's fast lookup)
#include "stmlib/dsp/units.h"


extern "C" void Shim_LPG_Init(void) {
    for(int i=0; i<6; i++) {
        lpg[i].Init();
        lpg_env[i].Init();
    }
}

extern "C" void Shim_LPG_Trigger(uint8_t chan) {
    if (chan < 6) lpg_env[chan].Trigger();
}

extern "C" void Shim_LPG_Process(uint8_t chan, float* in_out, size_t size, float decay, float color) {
    if (chan >= 6) return;

    // Map Decay (0..1) to Plaits Coefficients
    // Formula from Voice.cc:
    // const float short_decay = (200.0f * kBlockSize) / kSampleRate * SemitonesToRatio(-96.0f * patch.decay);
    // kBlockSize here is 'size' (usually 24). kSampleRate is 48000.
    
    // We compute per-block coefficients to save CPU.
    // ProcessPing advances state per sample.
    
    float decay_amount = Clamp(decay, 0.0f, 1.0f);
    float color_amount = Clamp(color, 0.0f, 1.0f);

    static float last_decay[6] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};
    static float cached_short[6];
    static float cached_tail[6];

    // Static State Cache for Downsampling
    static float lpg_env_gain_cache[6];
    static float lpg_env_freq_cache[6];
    static float lpg_env_hf_cache[6];
    
    // Recalculate only if decay changed
    if (decay_amount != last_decay[chan]) {
         const float kSampleRate = 48000.0f;
         float ratio = stmlib::SemitonesToRatio(-96.0f * decay_amount);
         
         // Formula derived from Plaits Voice::Render and Resources
         cached_short[chan] = (200.0f / kSampleRate) * ratio; 
         cached_tail[chan] = (20.0f / kSampleRate) * ratio;
         
         last_decay[chan] = decay_amount;
    }
    
    float short_decay = cached_short[chan];
    float decay_tail = cached_tail[chan];
    
    // HF Bleed based on color
    float hf = color_amount;
    
    float gain, frequency, hf_bleed;
    static uint8_t sample_counter[6] = {0}; // Persistent counter for correct downsampling across calls
    
    // CPU OPTIMIZATION: Update Vactrol State every 4 samples (~12kHz control rate)
    // Using persistent counter to support size=1 calls in oscillator.c
    for (size_t i = 0; i < size; ++i) {
        if ((sample_counter[chan] & 3) == 0) {
            lpg_env[chan].ProcessPing(0.1f, short_decay, decay_tail, hf);
            // Cache the results for the next 3 samples
            lpg_env_gain_cache[chan] = lpg_env[chan].gain();
            lpg_env_freq_cache[chan] = lpg_env[chan].frequency();
            lpg_env_hf_cache[chan] = lpg_env[chan].hf_bleed();
        }
        sample_counter[chan]++;
        
        gain = lpg_env_gain_cache[chan];
        frequency = lpg_env_freq_cache[chan];
        hf_bleed = lpg_env_hf_cache[chan];
        
        // Filter Process still runs per-sample for audio quality
        lpg[chan].Process(gain, frequency, hf_bleed, &in_out[i], 1);
    }
}

extern "C" float Shim_LPG_GetEnvelope(uint8_t chan) {
    if (chan >= 6) return 0.0f;
    return lpg_env[chan].gain();
}
