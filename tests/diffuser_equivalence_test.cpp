#include "plaits/dsp/fx/diffuser_optimised.h"
#include "plaits/dsp/fx/diffuser.h"
#include "stmlib/utils/random.h"
#include <stdio.h>
#include <math.h>
#include <vector>

using namespace plaits;

struct TestStats {
    double max_diff;
    double sum_sq_diff;
    double max_sig;
    size_t count;

    void Reset() {
        max_diff = 0;
        sum_sq_diff = 0;
        max_sig = 0;
        count = 0;
    }

    void Update(float orig, float opt) {
        double diff = fabs(orig - opt);
        if (diff > max_diff) max_diff = diff;
        sum_sq_diff += diff * diff;
        if (fabs(orig) > max_sig) max_sig = fabs(orig);
        count++;
    }

    double RMS() { return sqrt(sum_sq_diff / count); }
};

int main() {
    uint16_t buffer_opt[8192];
    uint16_t buffer_orig[8192];
    
    DiffuserOptimised diffuser_opt;
    Diffuser diffuser_orig;

    std::fill(buffer_opt, buffer_opt + 8192, 0);
    std::fill(buffer_orig, buffer_orig + 8192, 0);

    diffuser_opt.Init(buffer_opt);
    diffuser_orig.Init(buffer_orig);

    const size_t block_size = 48;
    float in_out_opt[block_size];
    float in_out_orig[block_size];

    printf("Comparing Optimized vs Original Diffuser...\n");
    printf("%-20s | %-10s | %-10s | %-10s\n", "Parameters", "Max Sig", "Max Diff", "RMS Diff");
    printf("----------------------------------------------------------------------\n");

    float amounts[] = { 0.1f, 0.5f, 0.9f };
    float rts[] = { 0.1f, 0.5f, 0.9f };

    TestStats stats;
    stats.Reset();

    for (float amount : amounts) {
        for (float rt : rts) {
            // Reset state
            std::fill(buffer_opt, buffer_opt + 8192, 0);
            std::fill(buffer_orig, buffer_orig + 8192, 0);
            
            diffuser_opt.Init(buffer_opt);
            diffuser_orig.Init(buffer_orig);
            
            TestStats sweep_stats;
            sweep_stats.Reset();

            for (int n = 0; n < 200; ++n) {
                // Generate identical random input
                for (size_t i = 0; i < block_size; ++i) {
                    float noise = 2.0f * stmlib::Random::GetFloat() - 1.0f;
                    in_out_opt[i] = noise;
                    in_out_orig[i] = noise;
                }

                // Process Original
                diffuser_orig.Process(amount, rt, in_out_orig, block_size);

                // Process Optimized
                diffuser_opt.Process(amount, rt, in_out_opt, block_size);

                for (size_t i = 0; i < block_size; ++i) {
                    sweep_stats.Update(in_out_orig[i], in_out_opt[i]);
                    stats.Update(in_out_orig[i], in_out_opt[i]);
                }
            }
            printf("A:%-.2f RT:%-.2f    | %-10.6f | %-10.6f | %-10.6f\n", 
                amount, rt, sweep_stats.max_sig, sweep_stats.max_diff, sweep_stats.RMS());
        }
    }

    printf("----------------------------------------------------------------------\n");
    printf("Overall Max Difference: %10.6f\n", stats.max_diff);
    printf("Overall RMS Difference: %10.6f\n", stats.RMS());

    if (stats.max_diff < 0.05f) {
        printf("Verification SUCCESS: Optimized diffuser results match original.\n");
    } else {
        printf("Verification FAILED: Significant difference detected.\n");
        return 1;
    }

    return 0;
}
