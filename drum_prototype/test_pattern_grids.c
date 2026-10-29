/*
 * test_pattern_grids.c
 *
 * Standalone host-side test for inc/pattern_grids.h + src/pattern_grids.c.
 * No STM32 dependencies; builds with a plain host C compiler:
 *
 *   cc -I ../inc -o test_pattern_grids test_pattern_grids.c ../src/pattern_grids.c
 *   ./test_pattern_grids
 */

#include <stdio.h>
#include <stdlib.h>
#include "pattern_grids.h"

static int g_failures = 0;

static void check_true(const char *label, int cond)
{
    if (!cond) {
        printf("FAIL %s\n", label);
        g_failures++;
    } else {
        printf("PASS %s\n", label);
    }
}

static void check_eq(const char *label, int got, int want)
{
    if (got != want) {
        printf("FAIL %s: got %d, want %d\n", label, got, want);
        g_failures++;
    } else {
        printf("PASS %s: %d\n", label, got);
    }
}

/* drum_map[0][0] is node_10; the first bytes of each of its three parts,
 * transcribed straight out of Grids' resources.cc. */
static const uint8_t node_10_kick_step0  = 145;
static const uint8_t node_10_kick_step12 = 255;
static const uint8_t node_10_snare_step4 = 159;
static const uint8_t node_10_hat_step0   = 255;

/* The two neighbours of that corner: drum_map[1][0] (x+1) is node_15,
 * drum_map[0][1] (y+1) is node_8. */
static const uint8_t node_15_kick_step12 = 182;
static const uint8_t node_8_kick_step12  = 218;

int main(void)
{
    /* x=0,y=0 sits exactly on drum_map[0][0], so both interpolation
     * balances are 0 and the raw authored byte should come back. */
    check_eq("corner (0,0) kick step 0",  grids_read_map(0, 0,  0, 0), node_10_kick_step0);
    check_eq("corner (0,0) kick step 12", grids_read_map(0, 12, 0, 0), node_10_kick_step12);
    check_eq("corner (0,0) snare step 4", grids_read_map(1, 4,  0, 0), node_10_snare_step4);
    check_eq("corner (0,0) hihat step 0", grids_read_map(2, 0,  0, 0), node_10_hat_step0);

    /* x=64 crosses into the next cell, whose corner is drum_map[1][0]. */
    check_eq("corner (64,0) kick step 12", grids_read_map(0, 12, 64, 0), node_15_kick_step12);

    /* x=32 is exactly halfway between those two corners (x_frac = 128),
     * so the level should land on their average. */
    {
        int a = node_10_kick_step12;
        int b = node_15_kick_step12;
        int want = a + (b - a) * 128 / 255;
        int got = grids_read_map(0, 12, 32, 0);
        check_true("midpoint x=32 averages its two corners", abs(got - want) <= 1);
        printf("     (got %d, want ~%d, corners %d/%d)\n", got, want, a, b);
    }
    /* Same again along y, whose neighbouring corner is drum_map[0][1]. */
    check_eq("corner (0,64) kick step 12", grids_read_map(0, 12, 0, 64), node_8_kick_step12);
    {
        int a = node_10_kick_step12;
        int b = node_8_kick_step12;
        int want = a + (b - a) * 128 / 255;
        int got = grids_read_map(0, 12, 0, 32);
        check_true("midpoint y=32 averages its two corners", abs(got - want) <= 1);
        printf("     (got %d, want ~%d, corners %d/%d)\n", got, want, a, b);
    }

    /* Every read must stay inside the map for the full input range. */
    {
        int ok = 1;
        for (int part = 0; part < GRIDS_NUM_PARTS; part++)
            for (int step = 0; step < GRIDS_NUM_STEPS; step++)
                for (int x = 0; x < 256; x += 17)
                    for (int y = 0; y < 256; y += 17)
                        if (grids_read_map((uint8_t)part, (uint8_t)step, (uint8_t)x, (uint8_t)y) > 255)
                            ok = 0;
        check_true("read_map never leaves 0..255 across the whole x/y range", ok);
    }

    {
        GridsState st;
        grids_init(&st);
        /* grids_advance() increments before its step is evaluated, so
         * init parks one step *before* 0 -- the first advance anywhere
         * (here or in firmware) wraps exactly onto step 0 instead of
         * skipping straight to step 1. See grids_init()'s comment. */
        check_eq("init parks one step before 0", st.step, GRIDS_NUM_STEPS - 1);
        grids_advance(&st);
        check_eq("first advance after init lands on step 0", st.step, 0);

        for (int i = 0; i < GRIDS_NUM_STEPS; i++)
            grids_advance(&st);
        check_eq("advance wraps after 32 steps", st.step, 0);
    }

    {
        /* density 0 => threshold 255 => nothing fires; density 255 =>
         * threshold 0 => everything with a non-zero level fires. */
        GridsState st;
        grids_init(&st);

        int any_at_zero = 0, missed_at_full = 0;
        for (int part = 0; part < GRIDS_NUM_PARTS; part++) {
            for (int step = 0; step < GRIDS_NUM_STEPS; step++) {
                if (grids_step_active(&st, (uint8_t)part, (uint8_t)step, 128, 128, 0, 0, NULL))
                    any_at_zero = 1;
                uint8_t level = grids_read_map((uint8_t)part, (uint8_t)step, 128, 128);
                if (level > 0 && !grids_step_active(&st, (uint8_t)part, (uint8_t)step, 128, 128, 255, 0, NULL))
                    missed_at_full = 1;
            }
        }
        check_true("density 0 fires nothing", !any_at_zero);
        check_true("density 255 fires every non-zero step", !missed_at_full);
    }

    {
        /* Chaos only matters once a jitter byte has been rolled (that
         * happens on the wrap back to step 0), and it can only ever
         * push a step over the threshold, never under it. */
        GridsState st;
        grids_init(&st);
        for (int i = 0; i < GRIDS_NUM_STEPS; i++)
            grids_advance(&st);

        int rolled = 0;
        for (int part = 0; part < GRIDS_NUM_PARTS; part++)
            if (st.jitter[part] != 0)
                rolled = 1;
        check_true("advancing a full lap rolls the jitter bytes", rolled);

        int monotonic = 1;
        for (int part = 0; part < GRIDS_NUM_PARTS; part++)
            for (int step = 0; step < GRIDS_NUM_STEPS; step++)
                if (grids_step_active(&st, (uint8_t)part, (uint8_t)step, 100, 200, 128, 0, NULL) &&
                    !grids_step_active(&st, (uint8_t)part, (uint8_t)step, 100, 200, 128, 255, NULL))
                    monotonic = 0;
        check_true("chaos never silences a step that already fired", monotonic);
    }

    {
        /* out_level should report the exact post-perturbation level
         * grids_step_active() itself thresholds against -- so a caller
         * checking out_level > GRIDS_ACCENT_LEVEL reproduces Grids'
         * own accent flag exactly, including chaos's contribution. */
        GridsState st;
        grids_init(&st);
        for (int i = 0; i < GRIDS_NUM_STEPS; i++)
            grids_advance(&st);

        int consistent = 1;
        for (int part = 0; part < GRIDS_NUM_PARTS; part++) {
            for (int step = 0; step < GRIDS_NUM_STEPS; step++) {
                uint8_t out_level = 0xFF;
                grids_step_active(&st, (uint8_t)part, (uint8_t)step, 100, 200, 128, 96, &out_level);

                uint16_t expect = grids_read_map((uint8_t)part, (uint8_t)step, 100, 200);
                expect += ((uint16_t)st.jitter[part] * 96) >> 8;
                if (expect > 255)
                    expect = 255;

                if (out_level != (uint8_t)expect)
                    consistent = 0;
            }
        }
        check_true("out_level matches the post-perturbation level used for the threshold", consistent);
    }

    if (g_failures == 0) {
        printf("\nAll tests passed.\n");
        return 0;
    } else {
        printf("\n%d test(s) FAILED.\n", g_failures);
        return 1;
    }
}
