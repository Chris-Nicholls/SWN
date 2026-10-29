#include "test_runner.h"
#include "plaits_shim.h"
#include <stdio.h>
#include <math.h>

void test_reproduce_wave_terrain_crash() {
    TEST_START("test_reproduce_wave_terrain_crash");
    Plaits_Init();
    float out[24];
    PlaitsParams params;
    // Engine 5 is Wave Terrain
    params.engine = 5;
    params.note = 48.0f;
    // We want terrain_index 5 to trigger bank 2
    // z = harmonics * 1.05 * (8 - 1.0001) = harmonics * 1.05 * 6.9999
    // harmonics = 0.7
    params.harmonics = 0.7f; 
    params.timbre = 0.5f;
    params.morph = 1.0f; // Max morph/x selects last wave index
    params.trigger = 0.0f;
    params.use_internal_lpg = 0;
    
    // Render MANY blocks to find it
    for (int i=0; i<100; i++) {
        Plaits_Render(0, &params, out, 24);
    }
    TEST_PASS();
}

void test_reproduce_speech_crash() {
    TEST_START("test_reproduce_speech_crash");
    Plaits_Init();
    float out[24];
    PlaitsParams params;
    // Engine 15 is Speech
    params.engine = 15;
    params.note = 48.0f;
    params.harmonics = 0.5f; // Random word mode
    params.timbre = 1.0f;    // Max timbre selects end of word
    params.morph = 1.0f;     
    params.trigger = 1.0f;   // Trigger word
    params.use_internal_lpg = 1;
    
    for (int i=0; i<100; i++) {
        Plaits_Render(0, &params, out, 24);
    }
    TEST_PASS();
}

int main() {
    test_reproduce_wave_terrain_crash();
    test_reproduce_speech_crash();
    return 0;
}
