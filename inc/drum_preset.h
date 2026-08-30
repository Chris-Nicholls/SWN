/*
 * drum_preset.h - kit save/load slots on the PRESET encoder
 *
 * Turning rotm_PRESET selects a slot (0..DRUM_PRESET_NUM_SLOTS-1);
 * releasing its pushbutton after a short press loads that slot into
 * drum_chan[], releasing after a long press (LONG_PRESSED, the same
 * ~6s threshold used elsewhere on this panel) saves the current
 * drum_chan[] state into it. Independent of the old preset_manager.c,
 * which still serializes the pre-drum-station o_params/o_lfos blob --
 * this is a small, dedicated flash sector for just the drum kit state.
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>

#define DRUM_PRESET_NUM_SLOTS	16

/* How long the outer-ring slot display stays up after the last turn or
 * a load/save, in ms -- see start_ongoing_display_drum_preset() in
 * led_cont.c. */
#define DRUM_PRESET_DISPLAY_TIMER_LIMIT	900

void	init_drum_preset(void);

/* Main-loop poll: PRESET turn selects a slot, its pushbutton
 * short/long-press release loads/saves. */
void	read_drum_preset_ui(void);

uint8_t	drum_preset_selected_slot(void);
uint8_t	drum_preset_slot_filled(uint8_t slot);
