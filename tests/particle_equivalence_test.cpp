#include "plaits/dsp/noise/particle_optimised.h"
#include "plaits/dsp/noise/particle.h"
#include "stmlib/utils/random.h"
#include <stdio.h>
#include <math.h>
#include <vector>

using namespace plaits;

int main() {
    ParticleOptimised particle_opt;
    Particle particle_orig;

    particle_opt.Init();
    particle_orig.Init();

    const size_t block_size = 48;
    float out_opt[block_size];
    float out_orig[block_size];
    float aux_opt[block_size];
    float aux_orig[block_size];

    printf("Comparing Optimized vs Original Particle Engine...\n");
    printf("%-30s | %-10s | %-10s | %-10s\n", "Parameters", "Max Sig", "Max Diff", "RMS Diff");
    printf("------------------------------------------------------------------------------------\n");

    float densities[] = { 0.01f, 0.1f, 0.5f };
    float frequencies[] = { 0.01f, 0.1f, 0.2f };
    float spreads[] = { 0.0f, 10.0f, 48.0f };
    float qs[] = { 0.5f, 10.0f, 100.0f };

    double total_max_diff = 0.0;
    double total_sum_sq_diff = 0.0;
    double max_sig_orig = 0.0;
    double max_sig_opt = 0.0;
    size_t total_samples = 0;

    for (float d : densities) {
        for (float f : frequencies) {
            for (float s : spreads) {
                for (float q : qs) {
                    // Reset and seed for determinism
                    particle_opt.Init();
                    particle_orig.Init();
                    
                    double sweep_max_diff = 0.0;
                    double sweep_sum_sq_diff = 0.0;
                    double sweep_max_sig = 0.0;

                    const float gain = 1.0f / d;

                    for (int n = 0; n < 100; ++n) {
                        // Seed RNG identically for both calls
                        uint32_t seed = 12345 + n;
                        
                        // Render Original
                        stmlib::Random::Seed(seed);
                        for (size_t i = 0; i < block_size; ++i) {
                            out_orig[i] = 0.0f;
                            aux_orig[i] = 0.0f;
                        }
                        particle_orig.Render(true, d, gain, f, s, q, out_orig, aux_orig, block_size);

                        // Render Optimized
                        stmlib::Random::Seed(seed);
                        for (size_t i = 0; i < block_size; ++i) {
                            out_opt[i] = 0.0f;
                            aux_opt[i] = 0.0f;
                        }
                        particle_opt.Render(true, d, gain, f, s, q, out_opt, aux_opt, block_size);

                        for (size_t i = 0; i < block_size; ++i) {
                            double sig = fabs(out_orig[i]);
                            if (sig > sweep_max_sig) sweep_max_sig = sig;
                            double diff = fabs(out_opt[i] - out_orig[i]);
                            if (diff > sweep_max_diff) sweep_max_diff = diff;
                            sweep_sum_sq_diff += diff * diff;
                            
                            if (diff > total_max_diff) total_max_diff = diff;
                            total_sum_sq_diff += diff * diff;
                            
                            if (fabs(out_orig[i]) > max_sig_orig) max_sig_orig = fabs(out_orig[i]);
                            if (fabs(out_opt[i]) > max_sig_opt) max_sig_opt = fabs(out_opt[i]);
                            
                            total_samples++;
                        }
                    }
                    
                    double sweep_rms = sqrt(sweep_sum_sq_diff / (100 * block_size));
                    printf("D:%-.2f F:%-.2f S:%-.2f Q:%-.2f | %-10.6f | %-10.6f | %-10.6f\n", d, f, s, q, sweep_max_sig, sweep_max_diff, sweep_rms);
                }
            }
        }
    }

    double total_rms_diff = sqrt(total_sum_sq_diff / total_samples);
    printf("----------------------------------------------------------------------\n");
    printf("Max Orig Sig: %10.6f, Max Opt Sig: %10.6f\n", max_sig_orig, max_sig_opt);
    printf("Overall Max Difference: %10.6f\n", total_max_diff);
    printf("Overall RMS Difference: %10.6f\n", total_rms_diff);

    if (total_max_diff < 1e-3) {
        printf("Verification SUCCESS: Optimized output matches original baseline.\n");
    } else {
        printf("Verification FAILED: Significant difference detected.\n");
        return 1;
    }

    return 0;
}
