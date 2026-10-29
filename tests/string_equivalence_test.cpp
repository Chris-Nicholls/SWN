#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include "plaits/dsp/physical_modelling/string_voice_optimised.h"
#include "plaits/dsp/physical_modelling/string_voice.h"
#include "equivalence_utils.h"
#include "stmlib/utils/random.h"

using namespace plaits;
using namespace plaits::tests;

int main() {
  const size_t block_size = 48;
  const size_t num_blocks = 100;
  
  uint8_t buffer_opt[16384];
  uint8_t buffer_orig[16384];
  stmlib::BufferAllocator allocator_opt(buffer_opt, 16384);
  stmlib::BufferAllocator allocator_orig(buffer_orig, 16384);

  StringVoiceOptimised voice_opt;
  StringVoice voice_orig;

  voice_opt.Init(&allocator_opt);
  voice_orig.Init(&allocator_orig);

  float out_opt[block_size];
  float aux_opt[block_size];
  float out_orig[block_size];
  float aux_orig[block_size];
  float temp_buf[block_size];

  TestStats stats;

  // Test parameters
  float notes[] = { 36.0f, 60.0f, 84.0f };
  float structures[] = { 0.1f, 0.5f, 0.9f };
  float brightnesses[] = { 0.1f, 0.5f, 0.9f };
  float dampings[] = { 0.1f, 0.5f, 0.9f };

  std::cout << "Starting String Voice Equivalence Test..." << std::endl;
  printf("%-20s | %-10s | %-10s | %-10s\n", "Parameters", "Max Sig", "Max Diff", "RMS Diff");
  printf("------------------------------------------------------------------------------------\n");

  for (float n : notes) {
    for (float s : structures) {
      for (float b : brightnesses) {
        for (float d : dampings) {
          voice_opt.Reset();
          voice_orig.Reset();
          
          float f0 = 440.0f / 48000.0f * powf(2.0f, (n - 69.0f) / 12.0f);
          TestStats local_stats;

          bool trigger = true;
          for (size_t block = 0; block < num_blocks; ++block) {
            uint32_t current_seed = 0x12345678 + block;
            for(size_t i=0; i<block_size; i++) {
                out_opt[i] = aux_opt[i] = out_orig[i] = aux_orig[i] = temp_buf[i] = 0.0f;
            }

            stmlib::Random::Seed(current_seed);
            voice_opt.Render(false, trigger, 1.0f, f0, s, b, d, temp_buf, out_opt, aux_opt, block_size);
            
            stmlib::Random::Seed(current_seed);
            voice_orig.Render(false, trigger, 1.0f, f0, s, b, d, temp_buf, out_orig, aux_orig, block_size);
            trigger = false;

            for (size_t i = 0; i < block_size; ++i) {
              local_stats.Update(out_orig[i], out_opt[i]);
              stats.Update(out_orig[i], out_opt[i]);
              local_stats.Update(aux_orig[i], aux_opt[i]);
              stats.Update(aux_orig[i], aux_opt[i]);
            }
          }
          
          if (local_stats.max_diff > 1e-4) {
              printf("N:%-4.1f S:%-.2f B:%-.2f D:%-.2f | %-10.6f | %-10.6f | %-10.6f\n", 
                     n, s, b, d, local_stats.max_sig_orig, local_stats.max_diff, local_stats.RMS());
          }
        }
      }
    }
  }

  stats.PrintSummary();
  return stats.Success(5e-3) ? 0 : 1;
}
