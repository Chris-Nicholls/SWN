#include "plaits/dsp/engine2/string_machine_engine.h"
#include "plaits/dsp/engine2/string_machine_engine_optimised.h"
#include "stmlib/utils/buffer_allocator.h"
#include <stdio.h>
#include <math.h>
#include <vector>

using namespace plaits;

int main() {
    uint8_t buffer_orig[8192];
    uint8_t buffer_opt[8192];
    stmlib::BufferAllocator allocator_orig(buffer_orig, 8192);
    stmlib::BufferAllocator allocator_opt(buffer_opt, 8192);

    StringMachineEngine engine_orig;
    StringMachineEngineOptimised engine_opt;

    engine_orig.Init(&allocator_orig);
    engine_opt.Init(&allocator_opt);

    const size_t block_size = 48;
    float out_orig[block_size];
    float aux_orig[block_size];
    float out_opt[block_size];
    float aux_opt[block_size];

    printf("Comparing Optimized vs Original String Machine Engine...\n");
    printf("%-30s | %-10s | %-10s | %-10s\n", "Parameters", "Max Sig", "Max Diff", "RMS Diff");
    printf("------------------------------------------------------------------------------------\n");

    float notes[] = { 36.0f, 60.0f, 84.0f };
    float timbres[] = { 0.1f, 0.5f, 0.9f };
    float morphs[] = { 0.1f, 0.5f, 0.9f };
    float harmonics[] = { 0.1f, 0.5f, 0.9f };

    double total_max_diff = 0.0;
    double total_sum_sq_diff = 0.0;
    size_t total_samples = 0;

    for (float note : notes) {
        for (float timbre : timbres) {
            for (float morph : morphs) {
                for (float harmonic : harmonics) {
                    EngineParameters params;
                    params.note = note;
                    params.timbre = timbre;
                    params.morph = morph;
                    params.harmonics = harmonic;
                    params.trigger = TRIGGER_UNPATCHED;

                    // Reset state for determinism
                    allocator_orig.Free();
                    allocator_opt.Free();
                    engine_orig.Init(&allocator_orig);
                    engine_opt.Init(&allocator_opt);
                    engine_orig.Reset();
                    engine_opt.Reset();

                    double sweep_max_diff = 0.0;
                    double sweep_sum_sq_diff = 0.0;
                    double sweep_max_sig = 0.0;

                    for (int n = 0; n < 100; ++n) {
                        bool already_env_orig = false;
                        bool already_env_opt = false;
                        
                        engine_orig.Render(params, out_orig, aux_orig, block_size, &already_env_orig);
                        engine_opt.Render(params, out_opt, aux_opt, block_size, &already_env_opt);

                        for (size_t i = 0; i < block_size; ++i) {
                            double sig = fmax(fabs(out_orig[i]), fabs(aux_orig[i]));
                            if (sig > sweep_max_sig) sweep_max_sig = sig;

                            double diff_out = fabs(out_orig[i] - out_opt[i]);
                            double diff_aux = fabs(aux_orig[i] - aux_opt[i]);
                            double diff = fmax(diff_out, diff_aux);

                            if (diff > sweep_max_diff) sweep_max_diff = diff;
                            sweep_sum_sq_diff += diff_out * diff_out + diff_aux * diff_aux;
                            
                            if (diff > total_max_diff) total_max_diff = diff;
                            total_sum_sq_diff += diff_out * diff_out + diff_aux * diff_aux;
                            total_samples += 2;
                        }
                    }
                    double sweep_rms = sqrt(sweep_sum_sq_diff / (100 * block_size * 2));
                    printf("N:%-.1f T:%-.2f M:%-.2f H:%-.2f | %-10.6f | %-10.6f | %-10.6f\n", 
                        note, timbre, morph, harmonic, sweep_max_sig, sweep_max_diff, sweep_rms);
                }
            }
        }
    }

    double total_rms_diff = sqrt(total_sum_sq_diff / total_samples);
    printf("----------------------------------------------------------------------\n");
    printf("Overall Max Difference: %10.6f\n", total_max_diff);
    printf("Overall RMS Difference: %10.6f\n", total_rms_diff);

    if (total_max_diff < 0.01f) {
        printf("Verification SUCCESS: Optimized output matches original baseline.\n");
    } else {
        printf("Verification FAILED: Significant difference detected.\n");
        return 1;
    }

    return 0;
}
