#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include "plaits/dsp/drums/synthetic_snare_drum_optimised.h"
#include "plaits/dsp/drums/synthetic_snare_drum.h"
#include "equivalence_utils.h"
#include "stmlib/utils/random.h"

using namespace plaits;
using namespace plaits::tests;

int main() {
    const size_t block_size = 48;
    const size_t num_blocks = 100;

    float out_opt[block_size];
    float out_orig[block_size];

    TestStats stats;

    float notes[] = { 36.0f, 60.0f, 84.0f };
    float decays[] = { 0.1f, 0.5f, 0.9f };
    float snappies[] = { 0.1f, 0.5f, 0.9f };
    float fm_amounts[] = { 0.1f, 0.9f };

    std::cout << "Starting Synthetic Snare Drum Equivalence Test..." << std::endl;

    for (float n : notes) {
        for (float d : decays) {
            for (float s : snappies) {
                for (float fm : fm_amounts) {
                    SyntheticSnareDrumOptimised engine_opt;
                    SyntheticSnareDrum engine_orig;
                    engine_opt.Init();
                    engine_orig.Init();

                    TestStats local_stats;
                    for (size_t block = 0; block < num_blocks; ++block) {
                        uint32_t seed = 0x12345678 + block;
                        
                        stmlib::Random::Seed(seed);
                        engine_opt.Render(false, block == 0, 0.8f, n, fm, d, s, out_opt, block_size);
                        
                        stmlib::Random::Seed(seed);
                        engine_orig.Render(false, block == 0, 0.8f, n, fm, d, s, out_orig, block_size);

                        for (size_t i = 0; i < block_size; ++i) {
                            local_stats.Update(out_orig[i], out_opt[i]);
                            stats.Update(out_orig[i], out_opt[i]);
                        }
                    }
                    if (local_stats.max_diff > 1e-4) {
                         printf("N:%-4.1f D:%-.2f S:%-.2f FM:%-.2f | %-10.6f | %-10.6f\n", 
                            n, d, s, fm, local_stats.max_sig_orig, local_stats.max_diff);
                    }
                }
            }
        }
    }

    stats.PrintSummary();
    return stats.Success(1e-4) ? 0 : 1;
}
