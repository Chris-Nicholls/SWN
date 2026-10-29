#include "plaits/dsp/engine2/phase_distortion_engine_optimised.h"
#include "plaits/dsp/engine2/phase_distortion_engine.h"
#include "stmlib/utils/buffer_allocator.h"
#include <stdio.h>
#include <math.h>

using namespace plaits;

int main() {
    char memory_opt[16384];
    char memory_orig[16384];
    stmlib::BufferAllocator allocator_opt(memory_opt, 16384);
    stmlib::BufferAllocator allocator_orig(memory_orig, 16384);

    PhaseDistortionEngineOptimised engine_opt;
    PhaseDistortionEngine engine_orig;

    engine_opt.Init(&allocator_opt);
    engine_orig.Init(&allocator_orig);

    EngineParameters params;
    params.trigger = 0;
    params.note = 60.0f; // N:60 failed in sweep
    params.harmonics = 0.5f;
    params.timbre = 0.5f;
    params.morph = 0.5f;

    const size_t block_size = 48;
    float out_opt[block_size];
    float aux_opt[block_size];
    float out_orig[block_size];
    float aux_orig[block_size];
    bool already_enveloped;

    printf("DEBUG: Comparison N:60 H:0.5 T:0.5 M:0.5\n");
    
    for (int b = 0; b < 10; ++b) {
        engine_opt.Render(params, out_opt, aux_opt, block_size, &already_enveloped);
        engine_orig.Render(params, out_orig, aux_orig, block_size, &already_enveloped);

        if (b == 0 || b == 9) {
            printf("--- Block %d ---\n", b);
            for (int i = 0; i < 5; ++i) {
                printf("Sample %d | Orig: %8.6f | Opt: %8.6f | Diff: %8.6f\n", b * block_size + i, out_orig[i], out_opt[i], out_opt[i] - out_orig[i]);
            }
        }
    }

    return 0;
}
