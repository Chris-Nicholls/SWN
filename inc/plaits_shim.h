/*
 * plaits_shim.h
 *
 * C-compatible shim for Plaits engine
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float note;
    float harmonics;
    float timbre;
    float morph;
    int   engine; // Plaits engine index
    float lpg_decay;
    float lpg_color;
    // Modulation amounts (applied to base params)
    float mod_timbre;
    float mod_morph;
    float mod_harmonics;
    float mod_freq;
    
    float trigger; // 0.0 or 1.0 (or >0.5)
    uint8_t output_mode;
    uint8_t use_internal_lpg;
} PlaitsParams;

void Plaits_Init(void);
void Plaits_Render(uint8_t channel, PlaitsParams* params, float* out_buffer, int32_t size);

#ifdef __cplusplus
}
#endif
