/*
 * euclid_pattern.h
 *
 * Standalone Euclidean-rhythm pattern engine (Bjorklund's algorithm).
 *
 * This module is intentionally dependency-free (only standard C headers)
 * so it can be compiled and unit tested on a host machine, independent of
 * the STM32 firmware build. It has no knowledge of the rest of the SWN
 * codebase — the caller is responsible for wiring it into the drum
 * voices / LED ring.
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

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Standalone module: kept independent of the firmware's own NUM_CHANNELS
 * define so this file has zero coupling to other headers. */
#define EUCLID_NUM_CHANNELS    6
#define EUCLID_MAX_STEPS       18

typedef struct o_euclid_channel_state {
    int         k;                  /* active steps, 0..n */
    int         n;                  /* total steps, 1..EUCLID_MAX_STEPS */
    int         rotation;           /* rotation offset, 0..n-1 */
    int         current_step;       /* 0..n-1 */
    uint32_t    pattern;            /* bit i set => step i is active (bit 0 = step 0) */
} EuclidChannelState;

void euclid_init(EuclidChannelState *st);

void euclid_set_k(EuclidChannelState *st, int k);
void euclid_set_n(EuclidChannelState *st, int n);
void euclid_set_rotation(EuclidChannelState *st, int rotation);

bool euclid_advance(EuclidChannelState *st);
bool euclid_step_active(const EuclidChannelState *st, int step);

#ifdef __cplusplus
}
#endif
