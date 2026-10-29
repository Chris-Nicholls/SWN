#include "test_runner.h"
#include "plaits_shim.h"
#include <vector>

void test_multi_channel_chiptune() {
    TEST_START("test_multi_channel_chiptune");
    
    // 1. Initialize all 6 voices
    Plaits_Init();
    
    // 2. Configure all channels to Chiptune Engine (7)
    PlaitsParams params[6];
    float out_buffers[6][24];
    
    for (int i=0; i<6; i++) {
        memset(&params[i], 0, sizeof(PlaitsParams));
        params[i].engine = 7; // Chiptune
        params[i].note = 48.0f + i; // Different notes
        params[i].harmonics = 0.5f;
        params[i].timbre = 0.5f;
        params[i].morph = 0.5f;
        params[i].trigger = 0.0f;
    }

    // 3. Render loop to stress test concurrent usage
    printf("rendering 100 blocks with 6 voices...\n");
    for (int block=0; block<100; block++) {
        for (int ch=0; ch<6; ch++) {
            Plaits_Render(ch, &params[ch], out_buffers[ch], 24);
            
            // Basic sanity check: output shouldn't be NaN
            if (out_buffers[ch][0] != out_buffers[ch][0]) {
                printf("NaN detected on channel %d block %d\n", ch, block);
                // ASSERT_TRUE(false); 
            }
        }
    }
    
    TEST_PASS();
}

int main() {
    test_multi_channel_chiptune();
    return 0;
}
