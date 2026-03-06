#include "test_runner.h"
#include "plaits_shim.h"
#include <vector>

void test_host_crash_engine_7() {
    TEST_START("test_host_crash_engine_7");
    
    // 1. Initialize
    Plaits_Init();
    
    // 2. Select Engine 7 (Chiptune) explicitly
    PlaitsParams params;
    memset(&params, 0, sizeof(params));
    params.engine = 7;
    
    float out[24];
    
    // 3. Render a few frames to force engine switch and detailed allocation
    printf("Switching to Engine 7...\n");
    Plaits_Render(0, &params, out, 24);
    
    // 4. Stress parameters
    // Based on code:
    // clocked mode (trigger patched) uses Arpeggiator and Pattern Selector
    // unclocked uses ChordBank inversion
    
    printf("Testing Unclocked Mode (Chords)...\n");
    params.trigger = 0.0f; // Unpatched
    for (float harmonics = 0.0f; harmonics <= 1.0f; harmonics += 0.1f) {
        for (float timbre = 0.0f; timbre <= 1.0f; timbre += 0.1f) {
            params.harmonics = harmonics;
            params.timbre = timbre;
            Plaits_Render(0, &params, out, 24);
        }
    }
    
    printf("Testing Clocked Mode (Arpeggiator)...\n");
    params.trigger = 1.0f; // Patched
    // Rising edge trigger
    for (float harmonics = 0.0f; harmonics <= 1.0f; harmonics += 0.1f) {
        for (float timbre = 0.0f; timbre <= 1.0f; timbre += 0.1f) {
            params.harmonics = harmonics;
            params.timbre = timbre;
            params.trigger = 1.0f;
            Plaits_Render(0, &params, out, 24);
            params.trigger = 0.0f;
            Plaits_Render(0, &params, out, 24);
        }
    }
    
    TEST_PASS();
}

void test_host_crash_engine_6() {
    TEST_START("test_host_crash_engine_6 (String Machine)");
    
    // 1. Initialize
    // Plaits_Init(); // Already called? No, need fresh or reset? 
    // Plaits_Init calls Init on all voices. We can just reuse.
    
    PlaitsParams params;
    memset(&params, 0, sizeof(params));
    params.engine = 6; // String Machine
    
    float out[24];
    
    printf("Switching to Engine 6...\n");
    Plaits_Render(0, &params, out, 24);
    
    printf("Testing Engine 6 Parameters...\n");
    params.trigger = 1.0f; 
    for (float harmonics = 0.0f; harmonics <= 1.0f; harmonics += 0.1f) {
        for (float timbre = 0.0f; timbre <= 1.0f; timbre += 0.1f) {
            params.harmonics = harmonics;
            params.timbre = timbre;
            Plaits_Render(0, &params, out, 24);
        }
    }
    TEST_PASS();
}

int main() {
    Plaits_Init(); // Call once
    test_host_crash_engine_6();
    test_host_crash_engine_7();
    return 0;
}
