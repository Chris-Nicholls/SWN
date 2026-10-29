#include "plaits/dsp/engine2/phase_distortion_engine_optimised.h"
#include "plaits/dsp/engine2/phase_distortion_engine_naive.h"
#include "plaits/dsp/engine2/phase_distortion_engine.h"
#include "stmlib/utils/buffer_allocator.h"
#include <stdio.h>
#include <math.h>
#include <vector>

using namespace plaits;

int main() {
    char memory_wv[16384];
    char memory_naive[16384];
    char memory_orig[16384];
    stmlib::BufferAllocator allocator_wv(memory_wv, 16384);
    stmlib::BufferAllocator allocator_naive(memory_naive, 16384);
    stmlib::BufferAllocator allocator_orig(memory_orig, 16384);

    PhaseDistortionEngineOptimised engine_wv;
    PhaseDistortionEngineNaive engine_naive;
    PhaseDistortionEngine engine_orig;

    engine_wv.Init(&allocator_wv);
    engine_naive.Init(&allocator_naive);
    engine_orig.Init(&allocator_orig);

    EngineParameters params;
    params.trigger = 0;
    params.note = 60.0f;
    params.harmonics = 0.5f;
    params.timbre = 0.5f;
    params.morph = 0.5f;

    const size_t block_size = 48;
    float out_wv[block_size];
    float aux_wv[block_size];
    float out_naive[block_size];
    float aux_naive[block_size];
    float out_orig[block_size];
    float aux_orig[block_size];
    bool already_enveloped;

    printf("Comparing PD Implementations: Wavetable (WV) and Naive (NV) vs Original (OR)\n");
    printf("%-20s | %-16s | %-16s\n", "Parameters", "WV Max / RMS", "NV Max / RMS");
    printf("----------------------------------------------------------------------------\n");

    float notes[] = { 36.0f, 60.0f, 84.0f };
    float harmonics[] = { 0.0f, 0.5f, 1.0f };
    float timbres[] = { 0.1f, 0.5f, 0.9f };
    float morphs[] = { 0.1f, 0.5f, 0.9f };
    
    size_t notes_len = sizeof(notes)/sizeof(notes[0]);
    size_t harmonics_len = sizeof(harmonics)/sizeof(harmonics[0]);
    size_t timbres_len = sizeof(timbres)/sizeof(timbres[0]);
    size_t morphs_len = sizeof(morphs)/sizeof(morphs[0]);

    double total_wv_max = 0.0;
    double total_wv_sum_sq = 0.0;

    for (float n : notes) {
        for (float h : harmonics) {
            for (float t : timbres) {
                for (float m : morphs) {
                    params.note = n;
                    params.harmonics = h;
                    params.timbre = t;
                    params.morph = m;

                    engine_wv.Reset();
                    engine_naive.Reset();
                    engine_orig.Reset();

                    double wv_max_diff = 0.0;
                    double wv_sum_sq_diff = 0.0;
                    double nv_max_diff = 0.0;
                    double nv_sum_sq_diff = 0.0;

                    for (int b = 0; b < 10; ++b) {
                        engine_wv.Render(params, out_wv, aux_wv, block_size, &already_enveloped);
                        engine_naive.Render(params, out_naive, aux_naive, block_size, &already_enveloped);
                        engine_orig.Render(params, out_orig, aux_orig, block_size, &already_enveloped);

                        for (size_t i = 0; i < block_size; ++i) {
                            double d_wv = fabs(out_wv[i] - out_orig[i]);
                            if (d_wv > wv_max_diff) wv_max_diff = d_wv;
                            wv_sum_sq_diff += d_wv * d_wv;

                            double d_nv = fabs(out_naive[i] - out_orig[i]);
                            if (d_nv > nv_max_diff) nv_max_diff = d_nv;
                            nv_sum_sq_diff += d_nv * d_nv;
                        }
                    }
                    
                    double wv_rms = sqrt(wv_sum_sq_diff / (10 * block_size));
                    double nv_rms = sqrt(nv_sum_sq_diff / (10 * block_size));
                    printf("N:%-4.1f H:%-3.1f T:%-3.1f M:%-3.1f | %8.6f / %8.6f | %8.6f / %8.6f\n", 
                           n, h, t, m, wv_max_diff, wv_rms, nv_max_diff, nv_rms);
                    
                    if (wv_max_diff > total_wv_max) total_wv_max = wv_max_diff;
                    total_wv_sum_sq += wv_sum_sq_diff;
                }
            }
        }
    }

    printf("----------------------------------------------------------------------------\n");
    printf("Overall Max Difference: %10.6f\n", total_wv_max);
    printf("Overall RMS Difference: %10.6f\n", sqrt(total_wv_sum_sq / (notes_len * harmonics_len * timbres_len * morphs_len * 10 * block_size)));

    return 0;
}
