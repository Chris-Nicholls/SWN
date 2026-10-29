#include "test_runner.h"
#include "plaits_shim.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>

float random_float() {
    return (float)rand() / (float)RAND_MAX;
}

void test_engine(int engine, int iterations) {
    char test_name[50];
    sprintf(test_name, "test_engine_%d", engine);
    TEST_START(test_name);
    
    Plaits_Init();
    
    float out_buffer[24];
    PlaitsParams params;
    params.engine = engine;

    for (int i = 0; i < iterations; i++) {
        // Randomize params every block
        params.note = 12.0f + random_float() * 72.0f;
        params.harmonics = random_float();
        params.timbre = random_float();
        params.morph = random_float();
        params.lpg_decay = random_float();
        params.lpg_color = random_float();
        params.mod_timbre = random_float() * 2.0f - 1.0f;
        params.mod_morph = random_float() * 2.0f - 1.0f;
        params.mod_harmonics = random_float() * 2.0f - 1.0f;
        params.mod_freq = random_float() * 2.0f - 1.0f;
        params.trigger = (random_float() > 0.8f) ? 1.0f : 0.0f;
        params.use_internal_lpg = (random_float() > 0.5f);

        Plaits_Render(0, &params, out_buffer, 24);

        // Check for NaN or Inf
        for (int sample = 0; sample < 24; sample++) {
            if (isnan(out_buffer[sample]) || isinf(out_buffer[sample])) {
                printf(" ERROR: NaN/Inf detected at block %d, sample %d\n", i, sample);
                TEST_FAIL("Numerical instability detected");
                return;
            }
        }
    }
    
    TEST_PASS();
}

int main(int argc, char** argv) {
    srand(time(NULL));
    int iterations = 1000;

    if (argc > 1) {
        if (strcmp(argv[1], "all") == 0) {
            for (int e = 0; e < 24; e++) {
                test_engine(e, iterations);
            }
        } else {
            int e = atoi(argv[1]);
            test_engine(e, iterations);
        }
    } else {
        printf("Usage: %s <engine_index|all>\n", argv[0]);
        return 1;
    }

    if (tests_failed > 0) {
        printf("\nFAILED: %d tests failed\n", tests_failed);
        return 1;
    } else {
        printf("\nPASSED: Plaits engine test completed\n");
        return 0;
    }
}
