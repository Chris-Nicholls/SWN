/*
 * euclid_pattern.c
 *
 * Standalone Euclidean-rhythm pattern engine (Bjorklund's algorithm).
 * See euclid_pattern.h for the public API and module boundaries.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * See http://creativecommons.org/licenses/MIT/ for more information.
 *
 * -----------------------------------------------------------------------------
 */

#include "euclid_pattern.h"

/* Bjorklund's algorithm builds its output as a recursive interleaving of
 * two symbol groups (bresenham-style "as evenly spaced as possible"), then
 * rotates so the sequence starts on the first onset. That internal
 * first-onset rotation is what makes E(3,8) come out as the canonical
 * tresillo "10010010" (steps 0,3,6) instead of an arbitrary rotation of
 * the same rhythm. Bit i of the resulting mask (bit 0 = step 0, reading
 * left-to-right / LSB-to-MSB as step 0..n-1) corresponds to step i. */

static int euclid_clampi(int v, int lo, int hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

/* Rotation is a position on a circle of n steps, not a bounded value --
 * clamping it would stick at 0/n-1 instead of letting continuous
 * turning cycle through every position. */
static int euclid_wrapi(int v, int n)
{
    if (n <= 0)
        return 0;
    v %= n;
    if (v < 0)
        v += n;
    return v;
}

static void euclid_build(int level, const int *counts, const int *remainders, int *pattern, int *len)
{
    if (level == -1) {
        pattern[*len] = 0;
        (*len)++;
        return;
    }
    if (level == -2) {
        pattern[*len] = 1;
        (*len)++;
        return;
    }
    for (int i = 0; i < counts[level]; i++)
        euclid_build(level - 1, counts, remainders, pattern, len);
    if (remainders[level] != 0)
        euclid_build(level - 2, counts, remainders, pattern, len);
}

static uint32_t euclid_bjorklund(int k, int n)
{
    if (n <= 0 || k <= 0)
        return 0;
    if (k >= n)
        return (n >= 32) ? 0xFFFFFFFFu : ((1u << n) - 1u);

    /* Depth of the recursion is bounded by the Euclidean (GCD-style)
     * division chain on (n-k, k), which for n<=EUCLID_MAX_STEPS never
     * comes close to EUCLID_MAX_STEPS itself. */
    int counts[EUCLID_MAX_STEPS];
    int remainders[EUCLID_MAX_STEPS + 1];
    int divisor = n - k;
    int level = 0;

    remainders[0] = k;
    for (;;) {
        counts[level] = divisor / remainders[level];
        remainders[level + 1] = divisor % remainders[level];
        divisor = remainders[level];
        level++;
        if (remainders[level] <= 1)
            break;
    }
    counts[level] = divisor;

    int pattern[EUCLID_MAX_STEPS];
    int pattern_len = 0;
    euclid_build(level, counts, remainders, pattern, &pattern_len);

    int first_one = 0;
    for (int i = 0; i < pattern_len; i++) {
        if (pattern[i]) {
            first_one = i;
            break;
        }
    }

    uint32_t bits = 0;
    for (int i = 0; i < pattern_len; i++) {
        int idx = (first_one + i) % pattern_len;
        if (pattern[idx])
            bits |= (1u << i);
    }
    return bits;
}

/* User-facing rotation is applied on top of Bjorklund's own canonical
 * first-onset rotation: step i of the rotated pattern takes on whatever
 * base step (i+rotation)%n was, i.e. the whole rhythm is advanced by
 * `rotation` steps relative to the step counter. */
static uint32_t euclid_rotate(uint32_t base, int n, int rotation)
{
    if (n <= 0)
        return 0;
    rotation = ((rotation % n) + n) % n;
    if (rotation == 0)
        return base;

    uint32_t result = 0;
    for (int i = 0; i < n; i++) {
        int src = (i + rotation) % n;
        if (base & (1u << src))
            result |= (1u << i);
    }
    return result;
}

static void euclid_recompute(EuclidChannelState *st)
{
    uint32_t base = euclid_bjorklund(st->k, st->n);
    st->pattern = euclid_rotate(base, st->n, st->rotation);
}

void euclid_init(EuclidChannelState *st)
{
    st->n = 16;
    st->k = 4;
    st->rotation = 0;
    st->current_step = 0;
    euclid_recompute(st);
}

void euclid_set_n(EuclidChannelState *st, int n)
{
    st->n = euclid_clampi(n, 1, EUCLID_MAX_STEPS);
    st->k = euclid_clampi(st->k, 0, st->n);
    st->rotation = euclid_wrapi(st->rotation, st->n);
    if (st->current_step >= st->n)
        st->current_step = st->current_step % st->n;
    euclid_recompute(st);
}

void euclid_set_k(EuclidChannelState *st, int k)
{
    st->k = euclid_clampi(k, 0, st->n);
    euclid_recompute(st);
}

void euclid_set_rotation(EuclidChannelState *st, int rotation)
{
    st->rotation = euclid_wrapi(rotation, st->n);
    euclid_recompute(st);
}

bool euclid_advance(EuclidChannelState *st)
{
    st->current_step = (st->current_step + 1) % st->n;
    return euclid_step_active(st, st->current_step);
}

bool euclid_step_active(const EuclidChannelState *st, int step)
{
    int idx = ((step % st->n) + st->n) % st->n;
    return (st->pattern & (1u << idx)) != 0;
}
