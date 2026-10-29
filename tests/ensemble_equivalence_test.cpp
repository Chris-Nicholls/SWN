#include "plaits/dsp/fx/ensemble_optimised.h"
#include "plaits/dsp/fx/ensemble.h"
#include "stmlib/utils/random.h"
#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>

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

    double RMS() { return count > 0 ? sqrt(sum_sq_diff / count) : 0; }
};

int main() {
    float memory_opt[1024];
    Ensemble::E::T memory_orig[1024];
    
    EnsembleOptimised ensemble_opt;
    Ensemble ensemble_orig;

    std::fill(memory_opt, memory_opt + 1024, 0.0f);
    std::fill(memory_orig, memory_orig + 1024, 0);

    ensemble_opt.Init(memory_opt);
    ensemble_orig.Init(memory_orig);

    const size_t block_size = 48;
    float l_opt[block_size];
    float r_opt[block_size];
    float l_orig[block_size];
    float r_orig[block_size];

    printf("Comparing Optimized vs Original Ensemble FX...\n");
    printf("%-20s | %-10s | %-10s | %-10s\n", "Parameters", "Max Sig", "Max Diff", "RMS Diff");
    printf("----------------------------------------------------------------------\n");

    float amounts[] = { 0.1f, 0.5f, 0.9f };
    float depths[] = { 0.1f, 0.5f, 0.9f };

    TestStats stats;
    stats.Reset();

    for (float amount : amounts) {
        for (float depth : depths) {
            // Reset state for determinism
            std::fill(memory_opt, memory_opt + 1024, 0.0f);
            std::fill(memory_orig, memory_orig + 1024, 0);
            
            ensemble_opt.Init(memory_opt);
            ensemble_orig.Init(memory_orig);
            
            ensemble_opt.set_amount(amount);
            ensemble_opt.set_depth(depth);
            ensemble_orig.set_amount(amount);
            ensemble_orig.set_depth(depth);
            
            TestStats sweep_stats;
            sweep_stats.Reset();

            stmlib::Random::Seed(0x12345678);

            for (int n = 0; n < 200; ++n) {
                // Generate identical random input
                for (size_t i = 0; i < block_size; ++i) {
                    l_opt[i] = l_orig[i] = 2.0f * stmlib::Random::GetFloat() - 1.0f;
                    r_opt[i] = r_orig[i] = 2.0f * stmlib::Random::GetFloat() - 1.0f;
                }

                // Process Original
                ensemble_orig.Process(l_orig, r_orig, block_size);

                // Process Optimized
                ensemble_opt.Process(l_opt, r_opt, block_size);

                for (size_t i = 0; i < block_size; ++i) {
                    sweep_stats.Update(l_orig[i], l_opt[i]);
                    stats.Update(l_orig[i], l_opt[i]);
                    sweep_stats.Update(r_orig[i], r_opt[i]);
                    stats.Update(r_orig[i], r_opt[i]);
                }
            }
            printf("A:%-.2f D:%-.2f      | %-10.6f | %-10.6f | %-10.6f\n", 
                amount, depth, sweep_stats.max_sig, sweep_stats.max_diff, sweep_stats.RMS());
        }
    }

    printf("----------------------------------------------------------------------\n");
    printf("Overall Max Difference: %10.6f\n", stats.max_diff);
    printf("Overall RMS Difference: %10.6f\n", stats.RMS());

    if (stats.max_diff < 1e-5) {
        printf("Verification SUCCESS: Optimized ensemble results match original.\n");
        return 0;
    } else {
        printf("Verification FAILED: Significant difference detected.\n");
        return 1;
    }
}
