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
    if (size > 24) return;  // Safety: kMaxBlockSize = 24

    static plaits::Voice::Frame frames[24];  // Static buffer, not on stack
    // Frame is struct { short out; short aux; }

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

    // Render directly (kBlockSize is now 24)
    voices[channel].Render(patch, modulations, frames, size);

    // Copy to output buffer (converting int16 to float)
    // Copy to output buffer (converting int16 to float)
    if (params->output_mode == 1) {
        for (int i=0; i<size; i++) {
            out_buffer[i] = (float)frames[i].aux / 16384.0f;
        }
    } else {
        for (int i=0; i<size; i++) {
            out_buffer[i] = (float)frames[i].out / 16384.0f;
        }
    }
}

void Plaits_SetParams(uint8_t channel, PlaitsParams* params) {
    // This might not be needed if we pass params every Render call.
    // Keeping for compatibility with plan.
}

}
