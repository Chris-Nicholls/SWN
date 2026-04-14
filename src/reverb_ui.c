/*
 * reverb_ui.c
 *
 * UI handling for the REVERB_EDIT mode.
 */

#include "reverb_ui.h"
#include "ui_modes.h"
#include "params_update.h"
#include "UI_conditioning.h"
#include "analog_conditioning.h"
#include "hardware_controls.h"
#include "globals.h"
#include "math_util.h"

#include <math.h>

extern enum UI_Modes    ui_mode;
extern o_params         params;
extern o_calc_params    calc_params;
extern o_analog         analog[NUM_ANALOG_ELEMENTS];

// Per-slider catch-up state (one per physical slider, not per channel).
// Reuses the PanStates enum: pan_CACHED_LEVEL = waiting to catch up,
// pan_INACTIVE = physical slider is in control.
// Slider mapping:
//   0 = dry/wet send (inverted: top=dry)
//   1 = pre-gain       [0, 4]
//   2 = reverb_time    [0, 1]
//   3 = diffusion      [0, 1]
//   4 = reverb_lp      [0, 1]
//   5 = output level   [0, 2]
static enum PanStates   reverb_catch_state[NUM_CHANNELS];

// Cached slider ADC value (0–4095) for pick-up scheme.
static float            reverb_cached_pos[NUM_CHANNELS];

// Saved pan state so we can restore on exit.
static enum PanStates   saved_pan_state[NUM_CHANNELS];
static float            saved_cached_level[NUM_CHANNELS];

// ---------------------------------------------------------------------------

void check_reverb_edit_entry_exit(void)
{
    uint8_t med     = button_med_pressed(butm_LFOVCA_BUTTON);
    uint8_t handled = calc_params.already_handled_button[butm_LFOVCA_BUTTON];

    if (med && !handled) {
        if (ui_mode == PLAY) {
            ui_mode = REVERB_EDIT;

            // Clear any armed LFO->VCA toggles that built up during the
            // SHORT_PRESS phase of the hold, preventing a spurious toggle
            // when the button is eventually released.
            for (uint8_t c = 0; c < NUM_CHANNELS; c++)
                calc_params.armed[armf_LFOTOVCA][c] = 0;
            // Suppress the LFO-tovca ongoing display on release
            calc_params.button_safe_release[0] = 1;

            // Save pan state so slider catch-up works on exit.
            for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
                saved_pan_state[c]    = calc_params.adjusting_pan_state[c];
                saved_cached_level[c] = calc_params.cached_level[c];
            }

            // Arm pick-up for all 6 sliders to their respective param values.
            // Slider 0 (dry/wet): use channel 0's send as reference (inverted).
            reverb_catch_state[0] = pan_CACHED_LEVEL;
            reverb_cached_pos[0]  = (1.0f - params.reverb_send[0]) * 4095.0f;
            // Slider 1 (pre-gain): 0–4 → 0–4095
            reverb_catch_state[1] = pan_CACHED_LEVEL;
            reverb_cached_pos[1]  = (params.reverb_input_gain / 4.0f) * 4095.0f;
            // Slider 2 (reverb_time): 0–1 → 0–4095
            reverb_catch_state[2] = pan_CACHED_LEVEL;
            reverb_cached_pos[2]  = params.reverb_time * 4095.0f;
            // Slider 3 (diffusion): 0–1 → 0–4095
            reverb_catch_state[3] = pan_CACHED_LEVEL;
            reverb_cached_pos[3]  = params.reverb_diffusion * 4095.0f;
            // Slider 4 (reverb_lp): 0–1 → 0–4095
            reverb_catch_state[4] = pan_CACHED_LEVEL;
            reverb_cached_pos[4]  = params.reverb_lp * 4095.0f;
            // Slider 5 (output level): 0–2 → 0–4095
            reverb_catch_state[5] = pan_CACHED_LEVEL;
            reverb_cached_pos[5]  = (params.reverb_output_level / 2.0f) * 4095.0f;
            calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 1;

        } else if (ui_mode == REVERB_EDIT) {
            ui_mode = PLAY;

            // Restore pan state. Force catch-up so sliders
            // don't jump to their reverb-param positions as volume.
            for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
                calc_params.cached_level[c]         = saved_cached_level[c];
                calc_params.adjusting_pan_state[c]  = pan_CACHED_LEVEL;
            }

            calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 1;
        }
    }

    // Clear the handled flag once the button is fully released.
    if (!button_pressed(butm_LFOVCA_BUTTON)) {
        calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 0;
    }
}

// ---------------------------------------------------------------------------

void update_reverb_edit_sliders(void)
{
    // Detect held channel button for per-channel dry/wet editing (slider 0).
    int8_t held_chan = -1;
    for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
        if (button_pressed(c)) {
            held_chan = (int8_t)c;
            break;
        }
    }

    for (uint8_t s = 0; s < NUM_CHANNELS; s++) {
        float slider_val = analog[A_SLIDER + s].lpf_val;

        switch (reverb_catch_state[s]) {
            case pan_CACHED_LEVEL:
                // Hold until the physical slider reaches the cached position.
                if (fabsf(slider_val - reverb_cached_pos[s]) < 10.0f) {
                    reverb_catch_state[s] = pan_INACTIVE;
                }
                break;

            case pan_INACTIVE:
                reverb_cached_pos[s] = slider_val;

                switch (s) {
                    case 0: {
                        // Dry/wet send. Inverted: top (4095) = dry, bottom (0) = wet.
                        float send = 1.0f - (slider_val / 4095.0f);
                        if (held_chan >= 0) {
                            params.reverb_send[held_chan] = send;
                        } else {
                            for (uint8_t c = 0; c < NUM_CHANNELS; c++)
                                params.reverb_send[c] = send;
                        }
                    } break;

                    case 1: // Pre-gain [0, 4]
                        params.reverb_input_gain = (slider_val / 4095.0f) * 4.0f;
                        break;

                    case 2: // Reverb time [0, 1]
                        params.reverb_time = slider_val / 4095.0f;
                        break;

                    case 3: // Diffusion [0, 1]
                        params.reverb_diffusion = slider_val / 4095.0f;
                        break;

                    case 4: // LP / brightness [0, 1]
                        params.reverb_lp = slider_val / 4095.0f;
                        break;

                    case 5: // Output level [0, 2]
                        params.reverb_output_level = (slider_val / 4095.0f) * 2.0f;
                        break;
                }
                break;

            default:
                break;
        }
    }
}
