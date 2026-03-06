#include "plaits_shim.h"
#include <stdio.h>
#include <math.h>

int main() {
    Plaits_Init();
    PlaitsParams params;
    params.engine = 16; // PHASE_DISTORTION
    params.note = 60.0f;
    params.harmonics = 0.5f;
    params.timbre = 0.5f;
    params.morph = 0.5f;
    params.lpg_decay = 0.5f;
    params.lpg_color = 0.5f;
    params.mod_timbre = 0.0f;
    params.mod_morph = 0.0f;
    params.mod_harmonics = 0.0f;
    params.mod_freq = 0.0f;
    params.trigger = 0.0f;
    params.use_internal_lpg = false;
    params.output_mode = 0;

    float out[48];
    float aux[48] = { 0.0f };

    printf("Rendering 5 blocks of Phase Distortion (48 samples each)...\n");
    for (int b = 0; b < 5; ++b) {
        Plaits_Render(0, &params, out, 48);
        printf("Block %d (first 8 samples):\n", b);
        for (int i = 0; i < 8; ++i) {
            printf("  [%d] out: %8.4f\n", i, out[i]);
        }
    }

    // Check for extreme values or NaNs
    for (int i = 0; i < 48; ++i) {
        if (isnan(out[i]) || fabsf(out[i]) > 2.0f) {
            printf("Error: Invalid output detected at sample %d: out=%f\n", i, out[i]);
            return 1;
        }
    }

    printf("Verification complete. Outputs are within normal ranges.\n");
    return 0;
}
