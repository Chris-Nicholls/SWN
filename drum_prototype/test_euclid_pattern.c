/*
 * test_euclid_pattern.c
 *
 * Standalone host-side test for inc/euclid_pattern.h + src/euclid_pattern.c.
 * No STM32 dependencies; builds with a plain host C compiler:
 *
 *   cc -I ../inc -o test_euclid_pattern test_euclid_pattern.c ../src/euclid_pattern.c
 *   ./test_euclid_pattern
 */

#include <stdio.h>
#include <string.h>
#include "euclid_pattern.h"

static int g_failures = 0;

static void pattern_to_str(uint32_t pattern, int n, char *out)
{
    for (int i = 0; i < n; i++)
        out[i] = (pattern & (1u << i)) ? '1' : '0';
    out[n] = '\0';
}

static void check_pattern(const char *label, int k, int n, int rotation, const char *expected)
{
    EuclidChannelState st;
    euclid_init(&st);
    euclid_set_n(&st, n);
    euclid_set_k(&st, k);
    euclid_set_rotation(&st, rotation);

    char got[EUCLID_MAX_STEPS + 1];
    pattern_to_str(st.pattern, n, got);

    if (strcmp(got, expected) != 0) {
        printf("FAIL %s: E(%d,%d) rot=%d -> got %s, want %s\n", label, k, n, rotation, got, expected);
        g_failures++;
    } else {
        printf("PASS %s: E(%d,%d) rot=%d -> %s\n", label, k, n, rotation, got);
    }
}

static void check_true(const char *label, int cond)
{
    if (!cond) {
        printf("FAIL %s\n", label);
        g_failures++;
    } else {
        printf("PASS %s\n", label);
    }
}

int main(void)
{
    /* Canonical tresillo, step 0 read as the least-significant char. */
    check_pattern("tresillo", 3, 8, 0, "10010010");
    check_pattern("E(4,16)", 4, 16, 0, "1000100010001000");
    check_pattern("E(5,8)", 5, 8, 0, "10110110");

    {
        EuclidChannelState st;
        euclid_init(&st);
        euclid_set_n(&st, 8);
        euclid_set_k(&st, 0);
        check_true("k=0 has no active steps", st.pattern == 0);
    }

    {
        EuclidChannelState st;
        euclid_init(&st);
        euclid_set_n(&st, 8);
        euclid_set_k(&st, 8);
        check_true("k=n has all steps active", st.pattern == 0xFFu);
    }

    {
        /* Rotating tresillo by 1 should shift every active step down one
         * index (with wraparound): 10010010 rotated by 1 -> 00100101. */
        EuclidChannelState st;
        euclid_init(&st);
        euclid_set_n(&st, 8);
        euclid_set_k(&st, 3);
        euclid_set_rotation(&st, 1);
        char got[EUCLID_MAX_STEPS + 1];
        pattern_to_str(st.pattern, 8, got);
        check_true("rotation by 1 matches manual shift", strcmp(got, "00100101") == 0);
    }

    {
        /* euclid_advance should walk current_step through a full cycle and
         * report active/inactive consistently with euclid_step_active. */
        EuclidChannelState st;
        euclid_init(&st);
        euclid_set_n(&st, 8);
        euclid_set_k(&st, 3);
        euclid_set_rotation(&st, 0);

        int ok = 1;
        for (int i = 0; i < 16; i++) {
            bool triggered = euclid_advance(&st);
            bool expected = euclid_step_active(&st, st.current_step);
            if (triggered != expected)
                ok = 0;
        }
        check_true("advance matches step_active over two full cycles", ok);
        check_true("current_step wraps into [0,n)", st.current_step >= 0 && st.current_step < st.n);
    }

    {
        /* k and n clamp to valid ranges; rotation wraps (it's a
         * position on a circle of n steps, not a bounded value). */
        EuclidChannelState st;
        euclid_init(&st);
        euclid_set_n(&st, 100);
        check_true("n clamps to EUCLID_MAX_STEPS", st.n == EUCLID_MAX_STEPS);
        euclid_set_n(&st, 0);
        check_true("n clamps to >=1", st.n == 1);
        euclid_set_n(&st, 8);
        euclid_set_k(&st, -5);
        check_true("k clamps to >=0", st.k == 0);
        euclid_set_k(&st, 99);
        check_true("k clamps to <=n", st.k == st.n);
        euclid_set_rotation(&st, -3);
        check_true("rotation wraps negative into [0,n)", st.rotation == 5);
        euclid_set_rotation(&st, 99);
        check_true("rotation wraps large values into [0,n)", st.rotation == 3);
    }

    if (g_failures == 0) {
        printf("\nAll tests passed.\n");
        return 0;
    } else {
        printf("\n%d test(s) FAILED.\n", g_failures);
        return 1;
    }
}
