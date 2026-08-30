/*
 * pattern_grids.h
 *
 * Standalone port of Mutable Instruments Grids' drum pattern generator:
 * three parts (kick/snare/hihat) x 32 steps, bilinearly interpolated
 * between four of 25 pre-authored pattern nodes laid out on a 5x5 map,
 * then thresholded per-part by a density setting with optional random
 * perturbation ("chaos").
 *
 * Like euclid_pattern.h this module is intentionally dependency-free
 * (only standard C headers) so it can be compiled and unit tested on a
 * host machine, independent of the STM32 firmware build. The pattern
 * data is transcribed from Grids' resources.cc; the code is our own.
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GRIDS_NUM_PARTS		3	/* 0=kick, 1=snare, 2=hihat */
#define GRIDS_NUM_STEPS		32
#define GRIDS_MAP_BYTES		(GRIDS_NUM_PARTS * GRIDS_NUM_STEPS)	/* one node table: part-major */

/* Grids' own accent threshold: a step whose post-perturbation level
 * exceeds this is a "loud" hit, same cutoff the real firmware uses. */
#define GRIDS_ACCENT_LEVEL	192

typedef struct o_grids_state {
	uint8_t		step;					/* 0..GRIDS_NUM_STEPS-1 */
	uint8_t		jitter[GRIDS_NUM_PARTS];	/* raw random byte per part, rerolled once per 32-step lap */
	uint32_t	rng;					/* xorshift32 state */
} GridsState;

void grids_init(GridsState *st);

/* Advances to the next step (wrapping at 32); on the wrap back to step 0
 * each part's jitter byte is rerolled, matching Grids' once-per-pattern
 * perturbation rather than a fresh roll on every single step. */
void grids_advance(GridsState *st);

/* Interpolated 0..255 "how strongly does this step want to fire" level.
 * part 0..2, step 0..31, x/y 0..255 anywhere on the 5x5 node map. */
uint8_t grids_read_map(uint8_t part, uint8_t step, uint8_t x, uint8_t y);

/* True if this step fires: interpolated level plus this part's jitter
 * scaled by `chaos`, compared against (255 - density). If out_level is
 * non-NULL, the post-perturbation 0..255 level used for that decision
 * is written there too -- Grids flags level>192 as an accent, which is
 * exposed this way rather than baked into a bool so callers can turn
 * it into whatever "louder hit" means for them (e.g. an output gain). */
bool grids_step_active(const GridsState *st, uint8_t part, uint8_t step,
                       uint8_t x, uint8_t y, uint8_t density, uint8_t chaos,
                       uint8_t *out_level);

#ifdef __cplusplus
}
#endif
