#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include "plaits/dsp/drums/analog_snare_drum_optimised.h"
#include "plaits/dsp/drums/analog_snare_drum.h"
#include "equivalence_utils.h"
#include "stmlib/utils/random.h"

using namespace plaits;
using namespace plaits::tests;

int main() {
    const size_t block_size = 48;
    const size_t num_blocks = 200;

    AnalogSnareDrumOptimised snare_opt;
    AnalogSnareDrum snare_orig;

    snare_opt.Init();
    snare_orig.Init();

    float out_opt[block_size];
    float out_orig[block_size];

    TestStats stats;

    // Test parameters
    float frequencies[] = { 0.01f, 0.1f, 0.4f };
    float accents[] = { 0.0f, 0.5f, 1.0f };
    float tones[] = { 0.1f, 0.5f, 0.9f };
    float decays[] = { 0.1f, 0.5f, 0.9f };
    float snappies[] = { 0.1f, 0.5f, 0.9f };

    std::cout << "Starting Analog Snare Drum Equivalence Test..." << std::endl;
    printf("%-20s | %-10s | %-10s | %-10s\n", "Parameters", "Max Sig", "Max Diff", "RMS Diff");
    printf("------------------------------------------------------------------------------------\n");

    for (float f0 : frequencies) {
        for (float a : accents) {
            for (float t : tones) {
                for (float d : decays) {
                    for (float s : snappies) {
                        snare_opt.Init();
                        snare_orig.Init();

                        TestStats local_stats;
                        bool trigger = true;
                        uint32_t base_seed = 0x1234;

                        for (size_t block = 0; block < num_blocks; ++block) {
                            uint32_t current_seed = base_seed + block;
                            
                            stmlib::Random::Seed(current_seed);
                            snare_opt.Render(false, trigger, a, f0, t, d, s, out_opt, block_size);
                            
                            stmlib::Random::Seed(current_seed);
                            snare_orig.Render(false, trigger, a, f0, t, d, s, out_orig, block_size);
                            
                            trigger = false;

                            for (size_t i = 0; i < block_size; ++i) {
                                local_stats.Update(out_orig[i], out_opt[i]);
                                stats.Update(out_orig[i], out_opt[i]);
                            }
                        }
                        
                        if (local_stats.max_diff > 1e-4) {
                            printf("F:%-.2f A:%-.2f T:%-.2f D:%-.2f S:%-.2f | %-10.6f | %-10.6f | %-10.6f\n", 
                                f0, a, t, d, s, local_stats.max_sig_orig, local_stats.max_diff, local_stats.RMS());
                        }
                    }
                }
            }
        }
    }

    stats.PrintSummary();
    return stats.Success(1.5e-2) ? 0 : 1;
}
