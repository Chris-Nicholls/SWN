#include "plaits_shim.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <chrono>
#include <vector>
#include <algorithm>
#include <map>

// Names of Plaits engines for reporting
const char* engine_names[] = {
    // Bank 0
    "VIRTUAL_ANALOG",
    "WAVESHAPING",
    "FM",
    "GRAINS",
    "ADDITIVE",
    "WAVETABLE",
    "CHORD",
    
    // Bank 1
    "SWARM",
    "NOISE",
    "PARTICLE",
    "STRING",
    "MODAL",
    "BASS_DRUM",
    "SNARE_DRUM",
    "HI_HAT",

    // Bank 2
    "VA_VCF",
    "PHASE_DISTORTION",
    "SIX_OP_1",
    "SIX_OP_2",
    "SIX_OP_3",
    "WAVE_TERRAIN",
    "STRING_MACHINE",
    "CHIPTUNE"
};

float random_float() {
    return (float)rand() / (float)RAND_MAX;
}

struct BenchmarkResult {
    int engine_index;
    double avg_time_us;
    double max_time_us;
};

BenchmarkResult benchmark_engine(int engine, int iterations) {
    // Warmup
    float out_buffer[48];
    PlaitsParams params;
    params.engine = engine;
    params.note = 60.0f;
    params.harmonics = 0.5f;
    params.timbre = 0.5f;
    params.morph = 0.5f;
    params.lpg_decay = 0.5f;
    params.lpg_color = 0.5f;
    params.mod_timbre = 0.0f;
    params.mod_morph = 0.0f;
    params.mod_harmonics = 0.0f;
    params.mod_freq = 0.0f;
    params.trigger = 0.0f;
    params.use_internal_lpg = true;

    // Render a few blocks to initialize any tables/state
    for(int i=0; i<50; i++) {
         Plaits_Render(0, &params, out_buffer, 48);
    }
    
    std::vector<double> times;
    times.reserve(iterations);

    auto start_total = std::chrono::high_resolution_clock::now();
    auto warmup = 1000;
    auto hold = 100;
    for (int i = 0; i < iterations + warmup; i++) {
        // Randomize params slightly to simulate real usage
        if (i % hold == 0) {
            params.note = 36.0f + random_float() * 48.0f;
            params.harmonics = random_float();
            params.timbre = random_float();
            params.morph = random_float();
        }
        
        auto t1 = std::chrono::high_resolution_clock::now();
        Plaits_Render(0, &params, out_buffer, 48);
        auto t2 = std::chrono::high_resolution_clock::now();
        
        std::chrono::duration<double, std::micro> ms_double = t2 - t1;
        if (i >= warmup) {
            times.push_back(ms_double.count());
        }
    }

    double sum = 0;
    double max_val = 0;

    // throw away the longest 5% of samples
    std::sort(times.begin(), times.end());
    times.erase(times.end() - iterations / 20, times.end());

    for(double t : times) {
        sum += t;
        if(t > max_val) max_val = t;
    }

    BenchmarkResult result;
    result.engine_index = engine;
    result.avg_time_us = sum / iterations;
    result.max_time_us = max_val;
    
    return result;
}

int main(int argc, char** argv) {
    srand(time(NULL));
    int iterations = 5000; // Enough to get a stable average

    printf("Benchmarking Plaits Engines (%d iterations per engine)...\n", iterations);
    printf("------------------------------------------------------------\n");
    printf("| %-2s | %-20s | %-10s | %-10s | %-10s |\n", "ID", "Engine Name", "Avg (us)", "Max (us)", "Cost");
    printf("|----|----------------------|------------|------------|------------|\n");

    // Initialize Plaits once for all engines
    Plaits_Init();

    std::vector<BenchmarkResult> results;
    double baseline = 0;

    for (int e = 0; e < 23; e++) {
        // Skip auxiliary engines if they are just duplicates/modes of others, 
        // but Plaits treats them as separate engines in the list (0-15 are main, 16-23 are aux?) 
        // Actually Plaits usually exposes models. 
        // Let's test everything available in the shim.
        
        BenchmarkResult r = benchmark_engine(e, iterations);
        results.push_back(r);
        
        // Use Virtual Analog (0) as baseline "100" cost unit, or find the min.
        if (e == 0) baseline = r.avg_time_us;
    }
    
    // Normalize costs so Virtual Analog is ~100 (or minimal engine is 100)
    // Actually, let's normalize so the CHEAPEST engine is 100.
    double min_avg = 999999.0;
    for(auto& r : results) {
        if(r.avg_time_us < min_avg) min_avg = r.avg_time_us;
    }

    for (auto& r : results) {
        int cost = (int)((r.avg_time_us / min_avg) * 100.0);
        const char* name = (r.engine_index < 24) ? engine_names[r.engine_index] : "UNKNOWN";
        
        printf("| %-2d | %-20s | %-10.2f | %-10.2f | %-10d |\n", 
            r.engine_index, name, r.avg_time_us, r.max_time_us, cost);
    }
    printf("------------------------------------------------------------\n");
    
    return 0;
}
