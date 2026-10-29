#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include "plaits/dsp/drums/hi_hat_optimised.h"
#include "plaits/dsp/drums/hi_hat.h"
#include "equivalence_utils.h"
#include "stmlib/utils/random.h"

using namespace plaits;
using namespace plaits::tests;

template<typename T1, typename T2>
void TestHiHatEquivalence(const std::string& name, TestStats& global_stats) {
    const size_t block_size = 48;
    const size_t num_blocks = 100;

    float out_opt[block_size];
    float aux_opt[block_size]; // Not used but needed for Render maybe (wait, HiHat::Render doesn't have aux)
    float out_orig[block_size];
    
    float temp1_opt[block_size];
    float temp2_opt[block_size];
    float temp1_orig[block_size];
    float temp2_orig[block_size];

    std::cout << "Testing HiHat Equivalence: " << name << "..." << std::endl;

    float notes[] = { 36.0f, 60.0f, 84.0f };
    float tones[] = { 0.1f, 0.5f, 0.9f };
    float decays[] = { 0.1f, 0.5f, 0.9f };
    float noisiness[] = { 0.1f, 0.5f, 0.9f };

    for (float n : notes) {
        for (float t : tones) {
            for (float d : decays) {
                for (float noise : noisiness) {
                    T1 engine_opt;
                    T2 engine_orig;
                    engine_opt.Init();
                    engine_orig.Init();

                    TestStats local_stats;
                    for (size_t block = 0; block < num_blocks; ++block) {
                        uint32_t seed = 0x12345678 + block;
                        
                        stmlib::Random::Seed(seed);
                        engine_opt.Render(false, block == 0, 0.8f, n, t, d, noise, temp1_opt, temp2_opt, out_opt, block_size);
                        
                        stmlib::Random::Seed(seed);
                        engine_orig.Render(false, block == 0, 0.8f, n, t, d, noise, temp1_orig, temp2_orig, out_orig, block_size);

                        for (size_t i = 0; i < block_size; ++i) {
                            local_stats.Update(out_orig[i], out_opt[i]);
                            global_stats.Update(out_orig[i], out_opt[i]);
                        }
                    }
                    if (local_stats.max_diff > 1e-4) {
                         printf("N:%-4.1f T:%-.2f D:%-.2f Noise:%-.2f | %-10.6f | %-10.6f\n", 
                            n, t, d, noise, local_stats.max_sig_orig, local_stats.max_diff);
                    }
                }
            }
        }
    }
}

int main() {
    TestStats stats;

    TestHiHatEquivalence<HiHatOptimised<SquareNoise, SwingVCA, true, false>, HiHat<SquareNoise, SwingVCA, true, false>>("AnalogHiHat", stats);
    TestHiHatEquivalence<HiHatOptimised<RingModNoise, LinearVCA, false, true>, HiHat<RingModNoise, LinearVCA, false, true>>("SyntheticHiHat", stats);

    stats.PrintSummary();
    return stats.Success(1e-4) ? 0 : 1;
}
