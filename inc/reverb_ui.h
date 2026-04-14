/*
 * reverb_ui.h
 *
 * UI handling for the REVERB_EDIT mode.
 * Entry/exit: hold LFOVCA button for ≥2 seconds.
 * Sliders control 6 parameters:
 *   A) dry/wet send (hold channel button for per-channel)
 *   B) pre-gain (saturation drive) [0, 4]
 *   C) reverb_time [0, 1]
 *   D) diffusion [0, 1]
 *   E) reverb_lp [0, 1]
 *   F) output level [0, 2]
 */

#pragma once

#include <stdint.h>

// Called once per update_oscillators() tick, before the channel loop.
// Detects the LFOVCA MED_PRESS to enter/exit REVERB_EDIT mode.
void check_reverb_edit_entry_exit(void);

// Called each tick while ui_mode == REVERB_EDIT.
// Maps 6 sliders to reverb parameters with pick-up catch-up.
void update_reverb_edit_sliders(void);
