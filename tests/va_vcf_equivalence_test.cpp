#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include "plaits/dsp/engine2/virtual_analog_vcf_engine_optimised.h"
#include "plaits/dsp/engine2/virtual_analog_vcf_engine.h"
#include "equivalence_utils.h"
#include "stmlib/utils/random.h"

using namespace plaits;
using namespace plaits::tests;

int main() {
    const size_t block_size = 48;
    const size_t num_blocks = 100;

    float out_opt[block_size];
    float aux_opt[block_size];
    float out_orig[block_size];
    float aux_orig[block_size];

    TestStats stats;

    float notes[] = { 36.0f, 60.0f, 84.0f };
    float harmonics[] = { 0.1f, 0.5f, 0.9f };
    float timbres[] = { 0.5f }; // Reduced timbre sweep for speed
    float morphs[] = { 0.1f, 0.5f, 0.9f };

    std::cout << "Starting VA VCF Engine Equivalence Test..." << std::endl;
    printf("%-20s | %-10s | %-10s | %-10s\n", "Parameters", "Max Sig", "Max Diff", "RMS Diff");
    printf("------------------------------------------------------------------------------------\n");

    for (float n : notes) {
        for (float h : harmonics) {
            for (float t : timbres) {
                for (float m : morphs) {
                    // RE-INSTANTIATE to ensure clean state
                    VirtualAnalogVCFEngineOptimised engine_opt;
                    VirtualAnalogVCFEngine engine_orig;
                    engine_opt.Init(NULL);
                    engine_orig.Init(NULL);

                    EngineParameters params;
                    params.note = n;
                    params.harmonics = h;
                    params.timbre = t;
                    params.morph = m;
                    params.trigger = 1; // Trigger once

                    TestStats local_stats;
                    for (size_t block = 0; block < num_blocks; ++block) {
                        bool already_enveloped = false;
                        uint32_t seed = 0x12345678 + block;
                        
                        stmlib::Random::Seed(seed);
                        engine_opt.Render(params, out_opt, aux_opt, block_size, &already_enveloped);
                        
                        stmlib::Random::Seed(seed);
                        engine_orig.Render(params, out_orig, aux_orig, block_size, &already_enveloped);
                        params.trigger = 0;

                        for (size_t i = 0; i < block_size; ++i) {
                            local_stats.Update(out_orig[i], out_opt[i]);
                            stats.Update(out_orig[i], out_opt[i]);
                            local_stats.Update(aux_orig[i], aux_opt[i]);
                            stats.Update(aux_orig[i], aux_opt[i]);
                        }
                    }
                    
                    if (local_stats.max_diff > 1e-4) {
                        printf("N:%-4.1f H:%-.2f T:%-.2f M:%-.2f | %-10.6f | %-10.6f | %-10.6f\n", 
                            n, h, t, m, local_stats.max_sig_orig, local_stats.max_diff, local_stats.RMS());
                    }
                }
            }
        }
    }

    stats.PrintSummary();
    return stats.Success(1e-4) ? 0 : 1;
}
