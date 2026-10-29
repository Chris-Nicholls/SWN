#include <iostream>
#include <chrono>
#include <vector>
#include <iomanip>
#include <cmath>

#include "plaits/dsp/physical_modelling/resonator.h"
#include "plaits/dsp/physical_modelling/resonator_orig.h"
#include "plaits/dsp/noise/particle.h"
#include "plaits/dsp/noise/particle_orig.h"
#include "plaits/dsp/physical_modelling/string_voice.h"
#include "plaits/dsp/physical_modelling/string_voice_orig.h"
#include "stmlib/utils/buffer_allocator.h"

using namespace plaits;
using namespace std;

const size_t kTestBlockSize = 48;
const size_t kTestIterations = 2000;

struct ComparisonResult {
    string name;
    double orig_time_ms;
    double opt_time_ms;
};

void print_result(const ComparisonResult& res) {
    double speedup = (res.orig_time_ms / res.opt_time_ms - 1.0) * 100.0;
    cout << "| " << left << setw(15) << res.name 
         << " | " << right << setw(12) << fixed << setprecision(3) << res.orig_time_ms 
         << " | " << right << setw(12) << res.opt_time_ms 
         << " | " << right << setw(8) << setprecision(1) << speedup << "% |" << endl;
}

int main() {
    uint8_t buffer_opt[16384];
    uint8_t buffer_orig[16384];
    stmlib::BufferAllocator allocator_opt(buffer_opt, 16384);
    stmlib::BufferAllocator allocator_orig(buffer_orig, 16384);

    vector<ComparisonResult> results;

    // --- Modal Engine (Resonator) ---
    {
        cout << "Testing Modal engine..." << endl;
        
        // Parameter combinations to test
        float f0_values[] = {0.05f, 0.1f, 0.2f};
        float structure_values[] = {0.2f, 0.5f, 0.8f};
        float brightness_values[] = {0.3f, 0.6f, 0.9f};
        float damping_values[] = {0.2f, 0.5f, 0.8f};
        
        vector<double> speedups;
        double total_orig = 0, total_opt = 0;
        
        for (float f0 : f0_values) {
            for (float structure : structure_values) {
                for (float brightness : brightness_values) {
                    for (float damping : damping_values) {
                        Resonator resonator_opt;
                        ResonatorOrig resonator_orig;
                        resonator_opt.Init(0.5f, 16);
                        resonator_orig.Init(0.5f, 16);
                        
                        float in[kTestBlockSize];
                        float out[kTestBlockSize];
                        for(size_t i=0; i<kTestBlockSize; ++i) in[i] = 0.1f;

                        // Warmup
                        for(int i=0; i<50; ++i) {
                            resonator_opt.Process(f0, structure, brightness, damping, in, out, kTestBlockSize);
                            resonator_orig.Process(f0, structure, brightness, damping, in, out, kTestBlockSize);
                        }

                        auto start_orig = chrono::high_resolution_clock::now();
                        for(size_t i=0; i<kTestIterations; ++i) {
                            resonator_orig.Process(f0, structure, brightness, damping, in, out, kTestBlockSize);
                        }
                        auto end_orig = chrono::high_resolution_clock::now();

                        auto start_opt = chrono::high_resolution_clock::now();
                        for(size_t i=0; i<kTestIterations; ++i) {
                            resonator_opt.Process(f0, structure, brightness, damping, in, out, kTestBlockSize);
                        }
                        auto end_opt = chrono::high_resolution_clock::now();

                        double orig_ms = chrono::duration<double, milli>(end_orig - start_orig).count();
                        double opt_ms = chrono::duration<double, milli>(end_opt - start_opt).count();
                        total_orig += orig_ms;
                        total_opt += opt_ms;
                        speedups.push_back((orig_ms / opt_ms - 1.0) * 100.0);
                    }
                }
            }
        }
        
        double avg_speedup = 0;
        for (double s : speedups) avg_speedup += s;
        avg_speedup /= speedups.size();
        
        auto minmax = minmax_element(speedups.begin(), speedups.end());
        
        results.push_back({
            "Modal (avg)",
            total_orig / speedups.size(),
            total_opt / speedups.size()
        });
        
        cout << "  Tested " << speedups.size() << " parameter combinations" << endl;
        cout << "  Speedup range: " << fixed << setprecision(1) << *minmax.first << "% to " << *minmax.second << "%" << endl;
    }

    // --- Particle Engine ---
    {
        cout << "Testing Particle engine..." << endl;
        
        float density_values[] = {0.3f, 0.6f, 0.9f};
        float frequency_values[] = {0.05f, 0.15f, 0.3f};
        float q_values[] = {0.3f, 0.6f, 0.9f};
        
        vector<double> speedups;
        double total_orig = 0, total_opt = 0;
        
        for (float density : density_values) {
            for (float frequency : frequency_values) {
                for (float q : q_values) {
                    Particle particle_opt;
                    ParticleOrig particle_orig;
                    particle_opt.Init();
                    particle_orig.Init();

                    float out[kTestBlockSize];
                    float aux[kTestBlockSize];

                    bool sync = false;
                    float gain = 0.5f;
                    float spread = 0.5f;

                    // Warmup
                    for(int i=0; i<50; ++i) {
                        particle_opt.Render(sync, density, gain, frequency, spread, q, out, aux, kTestBlockSize);
                        particle_orig.Render(sync, density, gain, frequency, spread, q, out, aux, kTestBlockSize);
                    }

                    auto start_orig = chrono::high_resolution_clock::now();
                    for(size_t i=0; i<kTestIterations; ++i) {
                        particle_orig.Render(sync, density, gain, frequency, spread, q, out, aux, kTestBlockSize);
                    }
                    auto end_orig = chrono::high_resolution_clock::now();

                    auto start_opt = chrono::high_resolution_clock::now();
                    for(size_t i=0; i<kTestIterations; ++i) {
                        particle_opt.Render(sync, density, gain, frequency, spread, q, out, aux, kTestBlockSize);
                    }
                    auto end_opt = chrono::high_resolution_clock::now();

                    double orig_ms = chrono::duration<double, milli>(end_orig - start_orig).count();
                    double opt_ms = chrono::duration<double, milli>(end_opt - start_opt).count();
                    total_orig += orig_ms;
                    total_opt += opt_ms;
                    speedups.push_back((orig_ms / opt_ms - 1.0) * 100.0);
                }
            }
        }
        
        double avg_speedup = 0;
        for (double s : speedups) avg_speedup += s;
        avg_speedup /= speedups.size();
        
        auto minmax = minmax_element(speedups.begin(), speedups.end());
        
        results.push_back({
            "Particle (avg)",
            total_orig / speedups.size(),
            total_opt / speedups.size()
        });
        
        cout << "  Tested " << speedups.size() << " parameter combinations" << endl;
        cout << "  Speedup range: " << fixed << setprecision(1) << *minmax.first << "% to " << *minmax.second << "%" << endl;
    }

    // --- String Engine (Voice) ---
    {
        cout << "Testing String engine..." << endl;
        
        float f0_values[] = {0.005f, 0.01f, 0.02f};
        float structure_values[] = {0.2f, 0.5f, 0.8f};
        float brightness_values[] = {0.3f, 0.6f, 0.9f};
        float damping_values[] = {0.3f, 0.6f, 0.9f};
        
        vector<double> speedups;
        double total_orig = 0, total_opt = 0;
        
        for (float f0 : f0_values) {
            for (float structure : structure_values) {
                for (float brightness : brightness_values) {
                    for (float damping : damping_values) {
                        // Create fresh buffers and allocators for each test
                        uint8_t test_buffer_opt[16384];
                        uint8_t test_buffer_orig[16384];
                        stmlib::BufferAllocator test_allocator_opt(test_buffer_opt, 16384);
                        stmlib::BufferAllocator test_allocator_orig(test_buffer_orig, 16384);
                        
                        StringVoice voice_opt;
                        StringVoiceOrig voice_orig;
                        voice_opt.Init(&test_allocator_opt);
                        voice_orig.Init(&test_allocator_orig);

                        float out[kTestBlockSize];
                        float aux[kTestBlockSize];
                        float temp[kTestBlockSize];

                        // Warmup
                        for(int i=0; i<50; ++i) {
                            voice_opt.Render(false, i==0, 1.0f, f0, structure, brightness, damping, temp, out, aux, kTestBlockSize);
                            voice_orig.Render(false, i==0, 1.0f, f0, structure, brightness, damping, temp, out, aux, kTestBlockSize);
                        }

                        auto start_orig = chrono::high_resolution_clock::now();
                        for(size_t i=0; i<kTestIterations; ++i) {
                            voice_orig.Render(false, false, 1.0f, f0, structure, brightness, damping, temp, out, aux, kTestBlockSize);
                        }
                        auto end_orig = chrono::high_resolution_clock::now();

                        auto start_opt = chrono::high_resolution_clock::now();
                        for(size_t i=0; i<kTestIterations; ++i) {
                            voice_opt.Render(false, false, 1.0f, f0, structure, brightness, damping, temp, out, aux, kTestBlockSize);
                        }
                        auto end_opt = chrono::high_resolution_clock::now();

                        double orig_ms = chrono::duration<double, milli>(end_orig - start_orig).count();
                        double opt_ms = chrono::duration<double, milli>(end_opt - start_opt).count();
                        total_orig += orig_ms;
                        total_opt += opt_ms;
                        speedups.push_back((orig_ms / opt_ms - 1.0) * 100.0);
                    }
                }
            }
        }
        
        double avg_speedup = 0;
        for (double s : speedups) avg_speedup += s;
        avg_speedup /= speedups.size();
        
        auto minmax = minmax_element(speedups.begin(), speedups.end());
        
        results.push_back({
            "String (avg)",
            total_orig / speedups.size(),
            total_opt / speedups.size()
        });
        
        cout << "  Tested " << speedups.size() << " parameter combinations" << endl;
        cout << "  Speedup range: " << fixed << setprecision(1) << *minmax.first << "% to " << *minmax.second << "%" << endl;
    }

    cout << "\nDirect Performance Comparison (" << kTestIterations << " iterations, " << kTestBlockSize << " samples/block)" << endl;
    cout << "----------------------------------------------------------------" << endl;
    cout << "| Engine          | Original (ms) | Optimized (ms) | Speedup |" << endl;
    cout << "|-----------------|---------------|----------------|---------|" << endl;
    for (const auto& res : results) {
        print_result(res);
    }
    cout << "----------------------------------------------------------------" << endl;

    return 0;
}
