#include "plaits/dsp/physical_modelling/resonator_optimised.h"
#include "plaits/dsp/physical_modelling/resonator.h"
#include "equivalence_utils.h"
#include <stdio.h>
#include <math.h>
#include <vector>
#include <iostream>

using namespace plaits;
using namespace plaits::tests;

int main() {
    ResonatorOptimised resonator_opt;
    Resonator resonator_orig;

    resonator_opt.Init(0.5f, kMaxNumModesOptimised);
    resonator_orig.Init(0.5f, kMaxNumModes);

    const size_t block_size = 48;
    float in[block_size];
    float out_opt[block_size];
    float out_orig[block_size];

    for (size_t i = 0; i < block_size; ++i) {
        in[i] = (i % 64 == 0) ? 1.0f : 0.0f;
    }

    printf("Comparing Optimized vs Original Modal Resonator...\n");
    printf("%-30s | %-10s | %-10s | %-10s\n", "Parameters", "Max Sig", "Max Diff", "RMS Diff");
    printf("------------------------------------------------------------------------------------\n");

    float f0s[] = { 0.01f, 0.1f, 0.4f };
    float structures[] = { 0.1f, 0.5f, 0.9f };
    float brightnesses[] = { 0.1f, 0.5f, 0.9f };
    float dampings[] = { 0.1f, 0.5f, 0.9f };

    TestStats stats;

    for (float f0 : f0s) {
        for (float s : structures) {
            for (float b : brightnesses) {
                for (float d : dampings) {
                    resonator_opt.Init(0.5f, kMaxNumModesOptimised);
                    resonator_orig.Init(0.5f, kMaxNumModes);

                    TestStats local_stats;

                    for (int n = 0; n < 200; ++n) {
                        for (size_t i = 0; i < block_size; ++i) {
                            out_opt[i] = 0.0f;
                            out_orig[i] = 0.0f;
                        }

                        resonator_opt.Process(f0, s, b, d, in, out_opt, block_size);
                        resonator_orig.Process(f0, s, b, d, in, out_orig, block_size);

                        for (size_t i = 0; i < block_size; ++i) {
                            local_stats.Update(out_orig[i], out_opt[i]);
                            stats.Update(out_orig[i], out_opt[i]);
                        }
                    }
                    
                    if (local_stats.max_diff > 1e-4) {
                        printf("F:%-.2f S:%-.2f B:%-.2f D:%-.2f | %-10.6f | %-10.6f | %-10.6f\n", 
                               f0, s, b, d, local_stats.max_sig_orig, local_stats.max_diff, local_stats.RMS());
                    }
                }
            }
        }
    }

    stats.PrintSummary();
    return stats.Success(5e-3) ? 0 : 1;
}
