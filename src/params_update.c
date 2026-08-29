/*
 * params_update.c - Parameters
 *
 * Authors: Dan Green (danngreen1@gmail.com), Hugo Paris (hugoplho@gmail.com)
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


#include "params_update.h"
#include "params_changes.h"
#include "led_cont.h"
#include "gpio_pins.h"
#include "exp_1voct_10_41V.h"
#include "flash_params.h"
#include "math.h"
#include "lfo_wavetable_bank.h"
#include "quantz_scales.h"
#include "led_cont.h"
#include "adc_interface.h"
#include "analog_conditioning.h"
#include "UI_conditioning.h"
#include "timekeeper.h"
#include "system_settings.h"
#include "math_util.h"
#include "params_wt_browse.h"
#include "quantz_scales.h"
#include "oscillator.h"
#include "hardware_controls.h"
#include "ui_modes.h"
#include "key_combos.h"
#include "preset_manager.h"
#include "preset_manager_UI.h"
#include "params_pitch.h"
#include "params_lfo_clk.h"
#include "params_lfo_period.h"
#include "params_lfo.h"
#include "flash_params.h"
#include "ui_modes.h"
#include "oscillator.h"
#include "drivers/flashram_spidma.h"
#include "preset_manager_selbus.h"

extern o_wt_osc wt_osc;
extern enum UI_Modes ui_mode;
extern o_lfos lfos;
extern SystemCalibrations *system_calibrations;


extern enum UI_Modes ui_mode;
extern	o_systemSettings	system_settings;
extern	o_analog	analog[NUM_ANALOG_ELEMENTS];
extern	o_macro_states macro_states;
extern	o_rotary	rotary[NUM_ROTARIES];
extern	o_button	button[NUM_BUTTONS];
extern	o_switch	hwSwitch[NUM_SWITCHES];
extern	o_monoLed 	monoLed[NUM_MONO_LED];
extern	o_led_cont	led_cont;

extern const uint8_t ALL_CHANNEL_MASK;

extern const int16_t TTONE[WT_TABLELEN];

const int8_t		CHORD_LIST[NUM_CHORDS][NUM_CHANNELS] =
{
	// DEFAULT
	{ 0 	, 0 	, 0 	, 0 	, 0 	, 0		}  , 	// NONE

	// FIFTH
	{ 0 	, 0 	, 7 	, 7 	, 0 	, 0		}  , 	// FIFTH
	{ 0 	, 0 	, 7 	, 7 	, -5 	, -5	}  , 	// FIFTH w/ inversion
	{ 0 	, 12	, 7	, 19	, -12	, -5	}  , 	// FIFTH w/ oct

	// MAJOR
	{ 0 	, 0 	, 4 	, 4 	, 7 	, 7		}  , 	// M3rd
	{ -8	, -5	, 0 	, 4 	, 7 	, 7	}  ,	// M3rd w inv
	{ -12	, -5	, 0 	, 4 	, 7 	, 12	}  ,	// M3rd w/ oct

	// MINOR
	{ 0 	, 0 	, 3 	, 3 	, 7 	, 7		}  , 	// m3rd
	{ -9	, -5	, 0 	, 3 	, 7 	, 7	}  ,	// m3rd w inv
	{ -12	, -5	, 0 	, 3 	, 7 	, 12	}  ,	// m3rd w/ oct

	// MAJOR EXT chords (6th, 7th 9th, 11th)
	{ 0 	, 0 	, 4 	, 4 	, 7 	, 9		}  , 	// M6th
	{ -12 	, 0 	, 4 	, 7 	, 9 	, 9		}  , 	// M6th w/oct
	{ -12 	, -3	, 0 	, 4 	, 4 	, 9		}  , 	// M6th w/ inv
	{ 0 	, 0 	, 4 	, 4 	, 7 	, 11	}  , 	// M7th
	{ -12 	, 0 	, 4 	, 7 	, 11 	, 11	}  , 	// M7th w/oct
	{ -12 	, -1	, 0 	, 4 	, 7 	, 11	}  , 	// M7th w/ inv
	{ 0 	, 0		, 4 	, 7 	, 11	, 14	}  , 	// M9th
	{ 0 	, 4		, 7 	, 11 	, 14	, 18	}  , 	// M11th

	// MINOR EXT chords (6th, 7th 9th, 11th)
	{ 0 	, 0 	, 3 	, 3 	, 7 	, 9		}  , 	// m6th
	{ -12 	, 0 	, 3 	, 7 	, 9 	, 9		}  , 	// m6th w/oct
	{ -12 	, -3	, 0 	, 3 	, 3 	, 9		}  , 	// m6th w/ inv
	{ 0 	, 0 	, 3 	, 3 	, 7 	, 10	}  , 	// m7th
	{ -12 	, 0 	, 3 	, 7 	, 10 	, 10	}  , 	// m7th w/oct
	{ -12 	, -2	, 0 	, 3 	, 7 	, 10	}  , 	// m7th w/ inv
	{ 0 	, 0		, 3 	, 7 	, 10	, 14	}  , 	// m9th
	{ 0 	, 3		, 7 	, 10 	, 14	, 17	}   	// m11th
};

const float DISP_PATTERN[NUM_DISPPAT][6][3] =	{
	{
		{1,	1,      1},
		{1,  	1,		1},
		{1,  	1,   	1},
		{1,		1,  	1},
		{1,  	1,		1},
		{1,  	1,   	1},
	},
	{
		{1,	0,      0},
		{0,  	1,		0},
		{0,  	0,  	1},
		{-1,	0,  	0},
		{0,  	-1,	0},
		{0,  	0,   	-1},
	},
	{
		{0.50,	1.30,	0.80},
		{1.20,	-.30,	0.60},
		{0.80,	1.30,	-1.2},
		{0.50,	0.20,	1.50},
		{-2.0,	0,		0.30},
		{1.0,	0.20,	0.30},
	},
	{
		{-1.2,	0.30,	0.60},
		{2.00,	1.30,	1.20},
		{0.50,	0.30,	0.80},
		{2.00,	2.00,	0.30},
		{0.10,	0.20,	1.20},
		{0.50,	0.20,	-1.5},
	},
	{
		{-2,	0.30,	0.60},
		{-0.20, -1,	1.2},
		{2,	2,		.80},
		{-2,	-1.50,	0.30},
		{1,	2,		1.2},
		{0.30,	-2,	-1.50},
	},
	{
		{1,	-3,	2},
		{-2,  	2,		.40},
		{-1.50,	.40,   1.2},
		{0.1,	2,  	-2},
		{1.90,	-2,	-.80},
		{0.20,	.90,   -1}
	}
};

extern const float exp_1voct_10_41V[4096];

o_params			params;
o_calc_params		calc_params;

uint32_t num_spheres_filled;
SRAM1DATA o_waveform	waveform[NUM_CHANNELS][2][2][2];


//Todo: replace with init_param_object(&params), plus a few other differences
void init_params(void){

	uint8_t  i;
	uint16_t j;

	// global & display
	params.dispersion_enc = 0;
	params.disppatt_enc = 1;
	params.noise_on = 0;
	params.spread_cv = 0;

	for (i=0; i<NUM_CHANNELS; i++){
		params.key_sw[i] = ksw_MUTE;

		params.note_on[i] = 1;
		calc_params.level[i] = 4093;
		calc_params.adjusting_pan_state[i] = pan_INACTIVE;
		calc_params.cached_level[i] = 0.f;

		// individual locks
		params.osc_param_lock[i] = 0;
		params.wtsel_lock[i] = 0;
		params.wt_pos_lock[i] = 0;

		params.wtsel_spread_enc[i] = 0;
		calc_params.wtsel[i] = 1;
		params.wtsel_enc[i] = 0;
		params.wt_bank[i] = 0;

		params.wt_browse_step_pos_enc[i] = 0;
		params.wt_nav_enc[0][i] = 0;
		params.wt_nav_enc[1][i] = 0;
		params.wt_nav_enc[2][i] = 0;

		params.spread_enc[i] = 0;
		calc_params.transpose[i] = 0;
		params.transpose_enc[i] = 0;

		params.pan[i] = default_pan(i);

		calc_params.gate_in_is_sustaining[i]	= 0;

		// flags
		calc_params.already_handled_button[i] = 0;

		for (j=0; j<NUM_ARM_FLAGS; j++)
			calc_params.armed[j][i] = 0;

		params.random[i] = 1.0;

	}

	calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 0;
	calc_params.already_handled_button[butm_LFOMODE_BUTTON] = 0;
	for (i=0; i<14; i++)
		params.enabled_spheres[i]=0xFF;

	// Chord overtone weights defaults 
	params.chord_overtone_weights [0] = 1.0f;
	params.chord_overtone_weights [1] = 0.8f;
	params.chord_overtone_weights [2] = 0.25f;
	params.chord_overtone_weights [3] = 0.6f;
	params.chord_overtone_weights [4] = 0.1f;
	params.chord_overtone_weights [5] = 0.3f;
	params.chord_overtone_weights [6] = 0.1f;


}


void init_pitch_params(void)
{
	uint8_t chan;
	for (chan = 0; chan<NUM_CHANNELS; chan++)
	{
		calc_params.tuning[chan]				= 1;
		calc_params.transposition[chan]		= 1;

		params.finetune[chan]					= 0;
		params.transpose_enc[chan]				= 0;

		params.oct[chan]						= INIT_OCT;
		params.indiv_scale[chan]				= 0;
		params.indiv_scale_buf[chan]			= params.indiv_scale[chan];

		params.qtz_note_changed[chan]			= 0;
	}
}

void init_param_object(o_params *t_params){
	uint8_t chan;

	// global & display
	t_params->dispersion_enc			= 0;
	t_params->disppatt_enc				= 1;
	t_params->noise_on					= 0;
	t_params->spread_cv				= 0;

	// individual params
	for (chan=0; chan<NUM_CHANNELS; chan++)
	{
		t_params->key_sw[chan]					= ksw_MUTE;
		t_params->note_on[chan]				= 1;

		// individual locks
		t_params->osc_param_lock[chan]			= 0;
		t_params->wtsel_lock[chan]				= 0;

		t_params->wtsel_enc[chan]				= 0;
		t_params->wtsel_spread_enc[chan]		= 0;
		t_params->wt_bank[chan]					= 0;

		t_params->wt_browse_step_pos_enc[chan]	= 0.0;

		t_params->osc_param_lock[chan]			= 0;

		t_params->wt_browse_step_pos_enc[chan]	= 0;
		t_params->wt_pos_lock[chan]				= 0;
		t_params->wt_nav_enc[0][chan]			= 0;
		t_params->wt_nav_enc[1][chan]			= 0;
		t_params->wt_nav_enc[2][chan]			= 0;

		t_params->spread_enc[chan]				= 0;
		t_params->transpose_enc[chan]			= 0;

		t_params->random[chan]					= 1.0;

		t_params->finetune[chan]				= 0;
		t_params->transpose_enc[chan]			= 0;

		t_params->oct[chan]					= INIT_OCT;
		t_params->indiv_scale[chan]			= 0;
		t_params->indiv_scale_buf[chan]		= 0;

		t_params->pan[chan]						= default_pan(chan);
		t_params->qtz_note_changed[chan]		= 0;

	}

	for (uint8_t i=0; i<14; i++)
		t_params->enabled_spheres[i]=0xFF;

	t_params->soft_clip_pregain = DEFAULT_SOFT_CLIP_PREGAIN;

	t_params->_reserved_param_a = 0.0f;
	t_params->_reserved_param_b = 0.0f;

	// EQ defaults (flat)
	for (chan = 0; chan < 6; chan++)
		t_params->eq_slider_values[chan] = 2048;  // 50% = flat

	// Chord overtone weights defaults (all 1.0)
	for (chan = 0; chan < 7; chan++)
		t_params->chord_overtone_weights[chan] = 1.0f;

	// Unison defaults
	for (chan=0; chan<NUM_CHANNELS; chan++) {
		t_params->unison_spread_amt[chan] = 0.2f; // Slight detune by default
		t_params->unison_voice_count[chan] = 1;

	}

	// Reverb defaults (vF)
	for (chan = 0; chan < NUM_CHANNELS; chan++)
		t_params->reverb_send[chan] = 0.0f;
	t_params->reverb_time         = 0.6f;
	t_params->reverb_diffusion    = 0.625f;
	t_params->reverb_lp           = 0.7f;
	t_params->reverb_input_gain   = 2.0f;
	t_params->reverb_output_level = 1.0f;

	/* Halo defaults (vH) — kept in sync with the historical
	 * static initializers used by the rs_*_base macros in this file. */
	for (chan = 0; chan < NUM_CHANNELS; chan++) {
		t_params->halo_damping[chan]     = 0.2f;
		t_params->halo_noise_level[chan] = 0.1f;
		t_params->halo_noise_color[chan] = 0.4f;
		t_params->halo_wt_attack[chan]   = 0.0f;
		t_params->halo_lpf_cutoff[chan]  = 24;
	}
}

void init_calc_params(void)
{
	uint8_t chan, j;
	for (chan=0; chan<NUM_CHANNELS; chan++)
	{

		calc_params.prev_qtz_note[chan] = 0xFF;
		calc_params.prev_qtz_oct[chan] = 0xFF;
		calc_params.gate_in_is_sustaining[chan] = 0;

		for (j = 0; j < NUM_ARM_FLAGS; j++)
			calc_params.armed[j][chan] = 0;

		calc_params.adjusting_pan_state[chan] = pan_INACTIVE;
		calc_params.cached_level[chan] = 0.f;
		calc_params.already_handled_button[chan] = 0;

	}
	calc_params.keymode_pressed = 0;
	calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 0;
	calc_params.already_handled_button[butm_LFOMODE_BUTTON] = 0;
	calc_params.button_safe_release[0] = 0;
	calc_params.button_safe_release[1] = 0;
}

uint8_t is_channel_in_chord_mode(uint8_t chan) {
	(void)chan;
	return 0;
}

void set_pitch_params_to_ttone(void) {
	for (uint8_t chan=0; chan<NUM_CHANNELS; chan++)
	{
		params.finetune[chan]					= 0;
		compute_tuning(chan);

		params.transpose_enc[chan]				= TTONE_TRANSPOSE;
		params.spread_enc[chan]				= 0;

		params.oct[chan]						= TTONE_OCT;
		params.indiv_scale[chan]				= 0;
		params.indiv_scale_buf[chan]			= params.indiv_scale[chan];

		params.qtz_note_changed[chan]			= 0;

	}

	combine_transpose_spread();
	compute_transpositions();

	for (uint8_t chan=0; chan<NUM_CHANNELS; chan++)
		update_pitch(chan);
}

void check_reset_navigation(void)
{
	uint8_t i;

	if (key_combo_reset_navigation())
	{
		exit_preset_manager();
		stop_all_displays();

		params.dispersion_enc				= 0;
		params.disppatt_enc				= 1;

		for (i=0; i<NUM_CHANNELS; i++)
		{
			if (!params.wt_pos_lock[i])
			{
				params.wt_browse_step_pos_enc[i]		= 0;
				reset_wbrowse_morph(i);
				params.wt_nav_enc[0][i]					= 0;
				params.wt_nav_enc[1][i]					= 0;
				params.wt_nav_enc[2][i]					= 0;
			}
		}
	}

	if (key_combo_reset_sphere_sel())
	{
		exit_preset_manager();
		stop_all_displays();

		for (i=0; i<NUM_CHANNELS; i++)
		{
			if (!params.wtsel_lock[i])
			{
				params.wtsel_spread_enc[i]				= 0;
				calc_params.wtsel[i]					= 1;
				params.wtsel_enc[i]						= 0;
				params.wt_bank[i]						= 0;
				req_wt_interp_update(i);
			}
		}
	}
}

void cache_uncache_nav_params(enum CacheUncache cache_uncache)
{
	uint8_t i;
	static float cached_dispersion_enc;
	static int8_t cached_wtsel_enc[NUM_CHANNELS];
	static int8_t cached_wtsel_spread_enc[NUM_CHANNELS];
	static float cached_wt_nav_enc[3][NUM_CHANNELS];
	static float cached_wt_browse_step_pos_enc[NUM_CHANNELS];
	static uint8_t cached_wt_bank[NUM_CHANNELS];

	if (cache_uncache==CACHE)
	{
		cached_dispersion_enc = params.dispersion_enc;
		for (i=0;i<NUM_CHANNELS;i++)
		{
			cached_wtsel_enc[i] = params.wtsel_enc[i];
			cached_wtsel_spread_enc[i] = params.wtsel_spread_enc[i];
			cached_wt_bank[i] = params.wt_bank[i];
			cached_wt_nav_enc[0][i] = params.wt_nav_enc[0][i];
			cached_wt_nav_enc[1][i] = params.wt_nav_enc[1][i];
			cached_wt_nav_enc[2][i] = params.wt_nav_enc[2][i];
			cached_wt_browse_step_pos_enc[i] = params.wt_browse_step_pos_enc[i];
		}
	}
	else
	{
		params.dispersion_enc = cached_dispersion_enc;
		for (i=0;i<NUM_CHANNELS;i++)
		{
			if (params.wtsel_lock[i])
			{
				params.wtsel_enc[i] = cached_wtsel_enc[i];
				params.wtsel_spread_enc[i] = cached_wtsel_spread_enc[i];
				params.wt_bank[i] = cached_wt_bank[i];
			}
			if (params.wt_pos_lock[i])
			{
				params.wt_nav_enc[0][i] = cached_wt_nav_enc[0][i];
				params.wt_nav_enc[1][i] = cached_wt_nav_enc[1][i];
				params.wt_nav_enc[2][i] = cached_wt_nav_enc[2][i];
				params.wt_browse_step_pos_enc[i] = cached_wt_browse_step_pos_enc[i];
				reset_wbrowse_morph(i);
			}
		}
	}
}


void cache_uncache_pitch_params(enum CacheUncache cache_uncache)
{
	static int16_t		finetune				[NUM_CHANNELS];
	static float		tuning					[NUM_CHANNELS];
	static int32_t		transpose				[NUM_CHANNELS];
	static float		transposition			[NUM_CHANNELS];
	static int8_t		oct					[NUM_CHANNELS];
	static uint8_t		indiv_scale			[NUM_CHANNELS];
	static uint8_t		indiv_scale_buf			[NUM_CHANNELS];

	for (uint8_t chan=0; chan<NUM_CHANNELS; chan++){
		switch (cache_uncache)
		{
			case CACHE:
				finetune[chan]					= params.finetune[chan];
				tuning[chan]					= calc_params.tuning[chan];
				transpose[chan]				= params.transpose_enc[chan];
				transposition[chan]			= calc_params.transposition[chan];
				oct[chan]						= params.oct[chan];
				indiv_scale[chan]				= params.indiv_scale[chan];
				indiv_scale_buf[chan]			= params.indiv_scale_buf[chan];
				break;

			case UNCACHE:
				params.finetune[chan]			= finetune[chan];
				calc_params.tuning[chan]		= tuning[chan];
				params.transpose_enc[chan]		= transpose[chan];
				calc_params.transposition[chan] = transposition[chan];
				params.oct[chan]				= oct[chan];
				params.indiv_scale[chan]		= indiv_scale[chan];
				params.indiv_scale_buf[chan]  	= indiv_scale_buf[chan];
				break;
		}
	}
}

static uint8_t new_key_armed[NUM_CHANNELS] = {0};

enum SelBusActions {SELBUS_NO_ACTION, SELBUS_TOGGLE_RECALL, SELBUS_TOGGLE_SAVE, SELBUS_STORE_SETTINGS};
void read_selbus_buttons(void)
{
	static enum SelBusActions selbus_action_armed = SELBUS_NO_ACTION;

	if (key_combo_show_selbus_allows()) {
		calc_params.already_handled_button[1] = 1;
		calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 1;
		calc_params.already_handled_button[butm_LFOMODE_BUTTON] = 1;
		start_ongoing_display_selbus();
	}
	if (key_combo_toggle_selbus_recall()) {
		if (selbus_action_armed != SELBUS_TOGGLE_RECALL)
			sel_bus_toggle_recall_allow();
		selbus_action_armed = SELBUS_TOGGLE_RECALL;
	}
	else if (system_settings.selbus_can_save == SELBUS_SAVE_ENABLED && key_combo_disable_selbus_save()) {
		if (selbus_action_armed != SELBUS_TOGGLE_SAVE)
			sel_bus_toggle_save_allow();
		selbus_action_armed = SELBUS_TOGGLE_SAVE;
	}
	else if (system_settings.selbus_can_save == SELBUS_SAVE_DISABLED && key_combo_enable_selbus_save()) {
		if (selbus_action_armed != SELBUS_TOGGLE_SAVE)
			sel_bus_toggle_save_allow();
		selbus_action_armed = SELBUS_TOGGLE_SAVE;
	}

	if (!key_combo_toggle_selbus_recall() && (selbus_action_armed == SELBUS_TOGGLE_RECALL)) {
		start_ongoing_display_selbus();
		selbus_action_armed = SELBUS_STORE_SETTINGS;
	}
	else if (!key_combo_disable_selbus_save() && (selbus_action_armed == SELBUS_TOGGLE_SAVE)) {
		start_ongoing_display_selbus();
		selbus_action_armed = SELBUS_STORE_SETTINGS;
	}
	if (!key_combo_show_selbus_allows()) {
		//stop_all_displays();
		if (selbus_action_armed == SELBUS_STORE_SETTINGS) {
			save_flash_params();
			selbus_action_armed = SELBUS_NO_ACTION;
		}
	}

}

void read_noteon(uint8_t i)
{
	if (ui_mode == PLAY)
	{
		// Aux Selection: Wavetable Encoder Press + Channel Button
		if (button_pressed(i) && rotary_pressed(rotm_WAVETABLE))
		{
			if (!calc_params.already_handled_button[i])
			{
				if (params.wt_bank[i] >= PLAITS_SPHERE_OFFSET)
				
				calc_params.already_handled_button[i] = 1;
			}
			return;
		}
		// Button mode: Mute
		if (params.key_sw[i] == ksw_MUTE)
		{
			if (!calc_params.already_handled_button[i] && button_pressed(i))
			{
				calc_params.armed[armf_NOTE_ON][i] = 1;
			}

			else if (button_released(i))
			{
				if (calc_params.armed[armf_NOTE_ON][i])
				{
					calc_params.armed[armf_NOTE_ON][i] = 0;
					if (!calc_params.already_handled_button[i])
					{
						if (calc_params.lock_change_staged[i]==1) {
							toggle_lock(i);
							calc_params.lock_change_staged[i] = 2;
						}
						else
							params.note_on[i] = 1 - params.note_on[i];
					}
					else
						calc_params.lock_change_staged[i] = 0;
				}
				calc_params.already_handled_button[i] = 0;
			}
		}

		// Button Mode: Note/Keyboard/CVGate/CVGateSus
		// ... and auto-notes at qtz crossings
		else
		{
			if (button_pressed(i))
			{
				if (params.key_sw[i]==ksw_NOTE) {
					lfos.cycle_pos[i] = 5.0/F_MAX_LFO_TABLELEN;  // read 5th element of LFO table to avoid silence at start
				}

				if (!new_key_armed[i]) {
					new_key_armed[i] = 1;
					params.new_key[i] = 1;
					params.note_on[i] = 1;
				}
			}
			else //button_released(i)
			{
				// NOTE AUTO EG trig
				if( (params.key_sw[i] == ksw_NOTE) && !params.note_on[i] && params.qtz_note_changed[i]==1 ){
					lfos.cycle_pos[i] = 0;
					params.note_on[i] = 1;
				}
				else
				{
					new_key_armed[i] = 0;
					if (params.key_sw[i]==ksw_KEYS) {
						params.note_on[i] = 0;
						lfos.cycle_pos[i] = 0;
					}
					else if (params.key_sw[i]==ksw_NOTE){
						params.new_key[i] = 0;
					}

					if (!calc_params.already_handled_button[i])
					{
						if (calc_params.lock_change_staged[i]==1) {
							toggle_lock(i);
							calc_params.lock_change_staged[i] = 2;
						}
					}
					else
						calc_params.lock_change_staged[i] = 0;

					calc_params.already_handled_button[i] = 0;
				}
			}
		}
	}

	// other UI modes
	else{
		params.note_on[i] = 1;
	}
}

float default_pan(uint8_t chan)
{
	return (chan&1) ? 0.33f :0.66f;
}

void set_master_gain(void)
{
	static uint32_t last_slider_a=0;
	float new_gain;
	int16_t slider_motion = analog[A_SLIDER].lpf_val - last_slider_a;

	if (abs(slider_motion)>10)
	{
		last_slider_a = analog[A_SLIDER].lpf_val;

		if (rotary_med_pressed(rotm_LFOSPEED) && rotary_med_pressed(rotm_OCT)){
			new_gain = _SCALE_U2F(analog[A_SLIDER].bracketed_val, 0, 4095, 80.0, 12.0);
			system_settings.master_gain = 1.0/new_gain;
		}
	}
}


float read_vca_cv(uint8_t chan)
{
	//No VCA CV if switch is set to V/oct, or if Key Mode is Key or Note
	if ((params.voct_switch_state[chan] == SW_VOCT) || (params.key_sw[chan] != ksw_MUTE))
		return 1.0;
	else
	if (analog[A_VOCT + chan].plug_sense_switch.pressed == RELEASED)
		return 1.0;
	else
	{
		//Handle Bipolar setting by treating negative voltage as 0
		if (analog[A_VOCT + chan].polarity == AP_UNIPOLAR)
			return ((analog[A_VOCT + chan].lpf_val / 2047.5) * F_SCALING_MAX_VCACV_GAIN);
		else {
			return (_CLAMP_F(analog[A_VOCT + chan].lpf_val - 2048.0, 0.0, 2048.0) / 2048.0 ) * F_SCALING_MAX_VCACV_GAIN;
		}
	}
}


/*** Move to params_lfos.c ***/

void read_lfoto_vca_vco(uint8_t i){

	static uint8_t any_button_pressed;

	if (button_pressed(butm_LFOVCA_BUTTON)
			&& !calc_params.keymode_pressed
			&& (params.key_sw[i] == ksw_MUTE)
			&& !calc_params.already_handled_button[butm_LFOVCA_BUTTON]) {

		start_ongoing_display_lfo_tovca();

		if (!macro_states.all_af_buttons_released){
			any_button_pressed = 1;
			calc_params.armed[armf_LFOTOVCA][i] = 0;

			if (button_pressed(i))
			{
				calc_params.armed[armf_LFOTOVCA][i] = 1;
				calc_params.already_handled_button[i] = 1;
				calc_params.button_safe_release[0] = 1;
			}
		}

		if (!button_pressed(i) && any_button_pressed && calc_params.armed[armf_LFOTOVCA][i]){
			//if (!lfos.locked[i])
				lfos.to_vca[i] = 1 - lfos.to_vca[i];
			calc_params.armed[armf_LFOTOVCA][i] = 0;
		}

		else if(!any_button_pressed){
			if  (button_pressed(butm_LFOVCA_BUTTON) < MED_PRESSED) {
				calc_params.button_safe_release[0] = 0;
				if (!lfos.locked[i])
					calc_params.armed[armf_LFOTOVCA][i] = 1;
			} else {
				calc_params.button_safe_release[0] = 1;
				calc_params.armed[armf_LFOTOVCA][i] = 0;
			}
		}
	}

	else if (!button_pressed(butm_LFOVCA_BUTTON))
	{
		if (calc_params.armed[armf_LFOTOVCA][i]){
			//if (!lfos.locked[i])
				lfos.to_vca[i] = 1 - lfos.to_vca[i];
			calc_params.armed[armf_LFOTOVCA][i] = 0;
		}
		if (calc_params.button_safe_release[0]) {
			calc_params.button_safe_release[0] = 0;
			stop_all_displays();
		}
		any_button_pressed = 0;
		calc_params.already_handled_button[butm_LFOVCA_BUTTON] = 0;
	}
}

/*** Move to params_lfos.c ***/

void read_lfomode(uint8_t i)
{
	static uint8_t any_button_pressed;
	static uint8_t cached[NUM_CHANNELS] = {0};

	if (!lfos.audio_mode[i])
	{
		if (cached[i]){ // uncache lfo mode when exiting audio range
			cache_uncache_lfomode(i, UNCACHE);
			cached[i] = 0;
		}

		else if (button_pressed(butm_LFOMODE_BUTTON)
				&& !calc_params.keymode_pressed
				&& (params.key_sw[i] == ksw_MUTE)
				&& !calc_params.already_handled_button[butm_LFOMODE_BUTTON]) {

			start_ongoing_display_lfo_mode();

			if (!macro_states.all_af_buttons_released){
				any_button_pressed = 1;
				calc_params.armed[armf_LFOMODE][i] = 0;

				if(button_pressed(i)){
					//if (!lfos.locked[i])
						calc_params.armed[armf_LFOMODE][i] = 1;
					calc_params.already_handled_button[i] = 1;
					calc_params.button_safe_release[1] = 1;
				}
			}

			if (!button_pressed(i) && any_button_pressed && calc_params.armed[armf_LFOMODE][i]){
				//if (!lfos.locked[i]) {
					// Toggle between LFO and LPG modes only
					lfos.mode[i] = (lfos.mode[i] == lfot_LFO) ? lfot_LPG : lfot_LFO;
				//}
				calc_params.armed[armf_LFOMODE][i]   = 0;
			}

			else if(!any_button_pressed){
				if  (button_pressed(butm_LFOMODE_BUTTON) < MED_PRESSED) {
					calc_params.button_safe_release[1] = 0;
					if (!lfos.locked[i]) calc_params.armed[armf_LFOMODE][i] = 1;
				} else {
					calc_params.button_safe_release[1] = 1;
					calc_params.armed[armf_LFOMODE][i] = 0;
				}
			}
		}

		else if (!button_pressed(butm_LFOMODE_BUTTON)){
			if(calc_params.armed[armf_LFOMODE][i]){
				if (!lfos.locked[i]) {
					// Toggle between LFO and LPG modes only
					lfos.mode[i] = (lfos.mode[i] == lfot_LFO) ? lfot_LPG : lfot_LFO;
				}
				calc_params.armed[armf_LFOMODE][i] = 0;
			}
			if (calc_params.button_safe_release[1]) {
				calc_params.button_safe_release[1] = 0;
				stop_all_displays();
			}
			calc_params.already_handled_button[butm_LFOMODE_BUTTON] = 0;
			any_button_pressed = 0;
		}
	}

	else if (!cached[i]){ // cache lfo mode and set to LFO when entering audio range
		cache_uncache_lfomode(i, CACHE);
		lfos.mode[i] = lfot_LFO;
		cached[i] = 1;
	}
}

/*** Move to params_keymode.c ***/

void read_all_keymodes(void){

	uint8_t				i;
	static uint8_t			any_button_pressed=0;
	uint8_t				change_keymode[NUM_CHANNELS] = {0};
	enum MuteNoteKeyStates	new_keymode;

	for (i = 0; i < NUM_CHANNELS; i++)
	{
		if (key_combo_keymode_pressed())
		{
			stop_all_displays();
			calc_params.keymode_pressed = 1;

			calc_params.armed[armf_LFOMODE][i]  = 0;
			calc_params.armed[armf_LFOTOVCA][i] = 0;

			if (!macro_states.all_af_buttons_released){
				any_button_pressed = 1;
				calc_params.armed[armf_KEYMODE][i] = 0;

				if(/*!lfos.locked[i] && */button_pressed(i)){
					calc_params.armed[armf_KEYMODE][i]= 1;
					calc_params.already_handled_button[i] = 1;
					calc_params.button_safe_release[0] = 1;
					calc_params.button_safe_release[1] = 1;
				}
			}

			if (/*!lfos.locked[i] && */ !button_pressed(i) && any_button_pressed && calc_params.armed[armf_KEYMODE][i]){
				change_keymode[i] = 1;
				calc_params.armed[armf_KEYMODE][i] = 0;
			}

			else if(!any_button_pressed){
				if (button_pressed(butm_LFOVCA_BUTTON) < MED_PRESSED) {
					calc_params.button_safe_release[0] = 0;
					calc_params.button_safe_release[1] = 0;
					if (!lfos.locked[i]) calc_params.armed[armf_KEYMODE][i]= 1;
				} else {
					calc_params.button_safe_release[0] = 1;
					calc_params.button_safe_release[1] = 1;
					if (!lfos.locked[i]) calc_params.armed[armf_KEYMODE][i]= 0;
				}
			}
		}

		else if (key_combo_keymode_released()){
			if (calc_params.armed[armf_KEYMODE][i]){
				//if (!lfos.locked[i])
					change_keymode[i] = 1;
				calc_params.armed[armf_KEYMODE][i] = 0;
			}
			if (calc_params.button_safe_release[0] && calc_params.button_safe_release[1]){
				calc_params.button_safe_release[0] = 0;
				calc_params.button_safe_release[1] = 0;
				stop_all_displays();
			}
			any_button_pressed = 0;
			calc_params.keymode_pressed = 0;
		}
	}

	for (i=0; i<NUM_CHANNELS; i++){
		if (change_keymode[i]){
			new_keymode = (params.key_sw[i]+1) % NUM_MUTE_NOTE_KEY_STATES;
			apply_keymode(i, new_keymode);
		}
	}
}

//Flip Mute to Note: cache indiv_scale and LFO params, set LFOs to envelope speed/shape, remove phase
//Flip Note to Keys: do nothing
//Flip Keys to Mute: uncache LFO params
void apply_keymode(uint8_t chan, enum MuteNoteKeyStates new_keymode)
{
	if (params.key_sw[chan] != new_keymode)
	{
		//Switched from MUTE to NOTE
		if (params.key_sw[chan] == ksw_MUTE)
		{
			lfos.muted[chan] = 1; //mute to prevent race condition with update_lfo_wt_pos()

			cache_uncache_keys_params_and_lfos(chan, CACHE);
			params.note_on[chan] = 0;
			lfos.to_vca[chan] = 1;
			lfos.mode[chan] = 0;
			lfos.shape[chan] = KEY_SHAPE;

			if(params.indiv_scale[chan]==sclm_NONE) //set scale to a useful value because Note auto-triggering is disabled when scale is unquantized
				params.indiv_scale[chan]=sclm_SEMITONES;

			lfos.cycle_pos[chan] = 1.0;
			lfos.divmult_id[chan] = LFO_UNITY_DIVMULT_ID+2;
			flag_lfo_recalc(chan);

			lfos.phase_id[chan] = 0;
			lfos.phase[chan] = 0;
		}

		//Switched to MUTE
		else if (new_keymode == ksw_MUTE)
		{
			calc_params.gate_in_is_sustaining[chan] = 0;
			lfos.muted[chan] = 1; //mute to prevent race condition with update_lfo_wt_pos()
			cache_uncache_keys_params_and_lfos(chan, UNCACHE);
		}

		else if (new_keymode == ksw_KEYS)
		{
			//Todo: Restore cached scale unless indiv_scale was changed between entering and exiting NOTE mode,
			// (even if it was changed and changed back)
			//params.indiv_scale[chan] = params.indiv_scale_buf[chan];
		}

		params.key_sw[chan] = new_keymode;
	}

	lfos.muted[chan] = 0;
}

void cache_uncache_keymodes(enum CacheUncache cache_uncache)
{
	static enum MuteNoteKeyStates cached_key_sw[NUM_CHANNELS];

	for (uint8_t chan=0; chan<NUM_CHANNELS; chan++)
	{
		if (cache_uncache==CACHE){
			cached_key_sw[chan] = params.key_sw[chan];
		}
		else {
			apply_keymode(chan, cached_key_sw[chan]);
		}
	}
}

void apply_all_keymodes(enum MuteNoteKeyStates new_keymode)
{
	for (uint8_t chan=0; chan<NUM_CHANNELS; chan++)
		apply_keymode(chan, new_keymode);
}

void cache_uncache_keys_params_and_lfos(uint8_t chan, enum CacheUncache cache_uncache)
{
	if (cache_uncache==CACHE)
	{
		lfos.divmult_id_buf[chan]		= lfos.divmult_id[chan];
		lfos.shape_buf[chan]			= lfos.shape[chan];
		lfos.gain_buf[chan]			= lfos.gain[chan];
		lfos.phase_id_buf[chan]		= lfos.phase_id[chan];
		lfos.mode_buf[chan]				= lfos.mode[chan];
		lfos.to_vca_buf[chan]			= lfos.to_vca[chan];

		params.note_on_buf[chan] 		= params.note_on[chan];
		params.indiv_scale_buf[chan]	= params.indiv_scale[chan];
	}
	else
	{
		// set_lfo_divmult_id(chan, lfos.divmult_id_buf[chan]);
		lfos.divmult_id[chan]			= lfos.divmult_id_buf[chan];
		flag_lfo_recalc(chan);

		lfos.phase_id[chan]			= lfos.phase_id_buf[chan];
		lfos.phase[chan]				= calc_lfo_phase(lfos.phase_id[chan]);

		lfos.shape[chan]				= lfos.shape_buf[chan];
		lfos.gain[chan]				= lfos.gain_buf[chan];
		lfos.mode[chan]					= lfos.mode_buf[chan];
		lfos.to_vca[chan]				= lfos.to_vca_buf[chan];

		params.note_on[chan] 			= params.note_on_buf[chan];
		params.indiv_scale[chan]		= params.indiv_scale_buf[chan];

		stage_resync(chan);
	}
}



/*** Move to params_pitch.c ***/

void update_transpose_cv(void)
{
	// Calculate pitch multiplier from transpose jack CV
	params.transpose_cv = calc_expo_pitch(TRANSPOSE_CV, analog[TRANSPOSE_CV].lpf_val);
}

void update_pitch(uint8_t chan)
{
	float ch_freq, ch_freq_adc, qtz_ch_freq;
	uint8_t note;
	int8_t oct;
	int16_t oct_clamped;

	if ( params.key_sw[chan]==ksw_MUTE || params.new_key[chan] || ((params.key_sw[chan] == ksw_NOTE) && !params.note_on[chan]) )
	{
		// Calculate pitch multiplier from individual jack 1V/oct CV
		if ((params.voct_switch_state[chan] == SW_VOCT) || (params.key_sw[chan] != ksw_MUTE))
		{
			if (params.indiv_scale[chan] != sclm_NONE)
				ch_freq_adc = analog[A_VOCT + chan].bracketed_val;
			else
				ch_freq_adc = analog[A_VOCT + chan].lpf_val;

			calc_params.voct[chan] = calc_expo_pitch(A_VOCT+chan, ch_freq_adc);
		} else
			calc_params.voct[chan] = 1.0;

		if (!params.osc_param_lock[chan])
			calc_params.voct[chan] *= params.transpose_cv;

		if (params.new_key[chan]) params.new_key[chan] = 0;
	}

	ch_freq = F_BASE_FREQ  * calc_params.transposition[chan] * calc_params.voct[chan];
	oct_clamped = _CLAMP_I16(params.oct[chan], MIN_OCT , MAX_OCT);
	if (oct_clamped < 0)
		ch_freq /= (float)(1 << (-oct_clamped));
	else
		ch_freq *= (1 << oct_clamped);

	if (params.indiv_scale[chan]==sclm_NONE)
	{
		calc_params.qtz_freq[chan] = ch_freq;
	}
	else
	{
//Todo:
//		qtz_ch_freq = quantize_to_scale(params.indiv_scale[chan], ch_freq, &note, &oct, prev_qtz_note[chan], prev_qtz_oct[chan]);
		qtz_ch_freq = quantize_to_scale(params.indiv_scale[chan], ch_freq, &note, &oct);

		if (qtz_ch_freq!=ch_freq && params.qtz_note_changed[chan]==0 && (calc_params.prev_qtz_note[chan]!=note || calc_params.prev_qtz_oct[chan]!=oct))
		{
			calc_params.prev_qtz_note[chan] = note;
			calc_params.prev_qtz_oct[chan] = oct;
			calc_params.qtz_freq[chan] = qtz_ch_freq;
			if ((params.key_sw[chan]==ksw_KEYS || params.key_sw[chan]==ksw_NOTE) && !params.note_on[chan])
				params.qtz_note_changed[chan] = 1;
		}
		else {
			if (params.qtz_note_changed[chan]>0)
				params.qtz_note_changed[chan]++;
			if (params.qtz_note_changed[chan]>QTZ_CHANGE_LOCKOUT_PERIOD)
				params.qtz_note_changed[chan]=0;
		}
	}

	// Apply fine-tuning
	calc_params.pitch[chan] = _CLAMP_F(calc_params.qtz_freq[chan] * calc_params.tuning[chan], F_MIN_FREQ, F_MAX_FREQ);

}

/*** Move to params_pitch.c ***/

void update_noise(uint8_t chan)
{
	float random_cv;
	static uint32_t noise_poll_ctr=0;

	//If the channel is fine-tuned to 0, then don't apply noise
	params.noise_on = (params.finetune[chan]==0) ? 0 : 1;

	if (params.noise_on)
	{
		if ( noise_poll_ctr++ > RANDOM_UPDATE_TIME)
		{
			noise_poll_ctr	= 0;

			random_cv = _CLAMP_F(analog[RANDOM_CV].lpf_val, 0.0, 6.0);
			params.random[chan] = 1 + ((random_cv - 3.0) * F_SCALING_RANDOM);
		}
		calc_params.pitch[chan] *= params.random[chan];
	}
}



//##########################################################
//						OSC PARAMS /*** Move to params_pitch.c ***/
//##########################################################


void cache_uncache_locks(enum CacheUncache cache_uncache)
{
	uint8_t chan;

	static uint8_t osc_param_lock[NUM_CHANNELS];
	static uint8_t wt_pos_lock[NUM_CHANNELS];
	static uint8_t wtsel_lock[NUM_CHANNELS];
	static uint8_t lfo_locked[NUM_CHANNELS];

	for (chan=0; chan<NUM_CHANNELS; chan++)
	{
		switch (cache_uncache)
		{
			case CACHE:
				osc_param_lock[chan]	= params.osc_param_lock[chan];
				wt_pos_lock[chan]		= params.wt_pos_lock[chan];
				wtsel_lock[chan]		= params.wtsel_lock[chan];
				lfo_locked[chan]		= lfos.locked[chan];
				break;

			case UNCACHE:
				params.osc_param_lock[chan] = osc_param_lock[chan];
				params.wt_pos_lock[chan]	= wt_pos_lock[chan];
				params.wtsel_lock[chan]	= wtsel_lock[chan];
				lfos.locked[chan]			= lfo_locked[chan];
				break;
		}
	}
}

void unlock_all(void)
{
	uint8_t chan;
	for (chan=0; chan<NUM_CHANNELS; chan++)
	{
		params.osc_param_lock[chan] = 0;
		params.wt_pos_lock[chan]	= 0;
		params.wtsel_lock[chan]	= 0;
		lfos.locked[chan]			= 0;
	}
}

void toggle_lock(uint8_t chan)
{
	uint8_t lock_status = params.osc_param_lock[chan] ? 0 : 1;
	params.osc_param_lock[chan] = lock_status;
	params.wt_pos_lock[chan] = lock_status;
	params.wtsel_lock[chan] = lock_status;
	lfos.locked[chan] = lock_status;
}

void update_osc_param_lock(void)
{
	uint8_t chan;

	for (chan=0; chan<NUM_CHANNELS; chan++)
	{
		if (key_combo_lock_channel(chan))
		{
			if (calc_params.lock_change_staged[chan]==0)
				calc_params.lock_change_staged[chan] = 1;
		}
		else
			if (calc_params.lock_change_staged[chan]==2)
				calc_params.lock_change_staged[chan] = 0;
	}
}




void update_oct(int16_t tmp)
{
	uint8_t i;
	int32_t oct[NUM_CHANNELS];

	for (i = 0; i < NUM_CHANNELS; i++)
		oct[i] = params.oct[i];

	if (change_param_i32(oct, tmp))
	{
		start_ongoing_display_octave();

		//Clamp to active range, but preserve relative spacing
		trim_array(oct, NUM_CHANNELS, MIN_OCT, MAX_OCT);

		//don't allow any channel to be more than MAX_OCT away from active range
		for (i = 0; i < NUM_CHANNELS; i++)
			params.oct[i] = _CLAMP_I16(oct[i], -MAX_OCT, MAX_OCT*2);
	}
}


void update_scale(int16_t tmp)
{
	uint8_t i;
	int16_t indiv_scale[NUM_CHANNELS];

	for (i = 0; i < NUM_CHANNELS; i++)
		indiv_scale[i] = params.indiv_scale[i];

	if (change_param_i16(indiv_scale, tmp))
	{
		start_ongoing_display_scale();
		for (i = 0; i < NUM_CHANNELS; i++)
			params.indiv_scale[i] = _WRAP_I8(indiv_scale[i], 0, NUM_QTZ_SCALES);
	}
}


void update_finetune(int16_t tmp)
{
	uint8_t i;
	int16_t finetune[NUM_CHANNELS];
	uint8_t channels_changed;
	uint8_t do_resync_osc=0;

	for (i = 0; i < NUM_CHANNELS; i++)
		finetune[i] = params.finetune[i];

	channels_changed = change_param_i16(finetune, tmp * 10);

	if (channels_changed)
	{
		for (i = 0; i < NUM_CHANNELS; i++)
		{
			if (channels_changed & (1<<i))
			{
				params.finetune[i] = finetune[i];
				if (finetune[i]==0)	do_resync_osc+=(1<<i);
				compute_tuning(i);
				start_ongoing_display_finetune();
			}
		}
	}

	if (do_resync_osc)
		resync_audio_osc(do_resync_osc);
}

void reset_octaves(void)
{
	uint8_t i;

	for (i=0; i<NUM_CHANNELS; i++)
	{
		if (!params.osc_param_lock[i])
			params.oct[i] = INIT_OCT;
	}
}

void reset_notes(void)
{
	uint8_t i;

	for (i=0; i<NUM_CHANNELS; i++)
	{
		if (!params.osc_param_lock[i]) {
			params.spread_enc[i] = 0;
			params.transpose_enc[i] = 0;
		}
	}
	combine_transpose_spread();
	compute_transpositions();
}

void retune_oscillators(void)
{
	uint8_t i;
	uint8_t chan_mask=0;

	for (i=0; i<NUM_CHANNELS; i++){
		if (!params.osc_param_lock[i]) {
			params.finetune[i] = 0;
			compute_tuning(i);
			chan_mask += 1<<i;
		}
	}
	resync_audio_osc(chan_mask);
}

void resync_audio_osc(uint8_t channels)
{
	uint8_t i;

	for (i=0; i<NUM_CHANNELS; i++)
	{
		if (channels & (1<<i)) {
			uint8_t v;
			for(v=0;v<MAX_UNISON_VOICES;v++) wt_osc.wt_head_pos[i][v] = 0;
		}
	}
}



// Fine-tune spread multipliers per channel (0.1 cents per encoder step)
// Slightly asymmetric to avoid predictable beating
static const int16_t spread_finetune_mult[NUM_CHANNELS] = {
	-35,  // Channel A
	-19,  // Channel B
	-11,  // Channel C
	 12,  // Channel D
	 21,  // Channel E
	 33   // Channel F
};

void spread_finetune(int16_t tmp)
{
	uint8_t i;
	uint8_t do_resync_osc=0;

	for (i = 0; i < NUM_CHANNELS; i++ )
	{
		if (!params.osc_param_lock[i])
		{
			params.finetune[i] += spread_finetune_mult[i] * tmp;

			if (params.finetune[i]==0)
				do_resync_osc += (1<<i);

			compute_tuning(i);
		}
	}
	start_ongoing_display_finetune();

	if (do_resync_osc)
		resync_audio_osc(do_resync_osc);
}


void compute_tuning (uint8_t chan){
	params.finetune[chan] = _CLAMP_I16(params.finetune[chan], MIN_FINETUNE_WRAP, MAX_FINETUNE_WRAP);
	calc_params.tuning[chan] = powf(2.0f, (float)params.finetune[chan] / 12000.0f);
}


void update_transpose(int16_t tmp){
	uint8_t i;
	int32_t transpose_enc[NUM_CHANNELS];
	uint8_t channels_changed;
	int32_t min, max;

	for (i = 0; i < NUM_CHANNELS; i++)
		transpose_enc[i] = params.transpose_enc[i];

	channels_changed = change_param_i32(transpose_enc, tmp);
	if (channels_changed)
	{
		max = MAX_TRANSPOSE_WRAP;
		min = MIN_TRANSPOSE_WRAP;

		trim_array(transpose_enc, NUM_CHANNELS, min, max);

		start_ongoing_display_transpose();

		for (i = 0; i < NUM_CHANNELS; i++)
			params.transpose_enc[i] = transpose_enc[i];
	}
}


void update_spread(int16_t tmp){

	uint8_t chan;

	for (chan=0; chan<NUM_CHANNELS; chan++){
		if (!params.osc_param_lock[chan]) {
			params.spread_enc[chan] += tmp;
			
			// Force qtz update cache to invalidate so it stops lagging
			calc_params.prev_qtz_note[chan] = 255;
		}
	}
	start_ongoing_display_transpose();
}

void update_spread_cv(void)
{
	params.spread_cv = (int8_t)((float)(analog[CHORD_CV].bracketed_val)  * (float)(NUM_CHORDS) / (4095.0*1.04));//4% down-scaling allows black keys to select chords (C#0 to C#5)
}

void combine_transpose_spread(void){

	uint8_t chan;
	uint8_t chord_num;
	uint8_t spread_cv;

	for (chan=0; chan<NUM_CHANNELS; chan++)
	{
		spread_cv = params.osc_param_lock[chan] ? 0: params.spread_cv;
		chord_num = _WRAP_I8(spread_cv + params.spread_enc[chan], 0, NUM_CHORDS);

		calc_params.transpose[chan] = params.transpose_enc[chan] + CHORD_LIST[chord_num][chan];
	}
}

// trim_array() trims extra steps beyond MAX/MIN values, when all elements are beyond threshold of display_min or display_max
// It serves to clip values while still preserving the spacing between them
void trim_array(int32_t *a, uint32_t num_elements, int32_t display_min, int32_t display_max){

	uint8_t chan;
	int16_t min,max;

	uint8_t all_over_max, all_under_min;

	all_over_max = 1;
	for (chan=0; chan<num_elements; chan++)
	{
		if (a[chan] <= display_max) {
			all_over_max = 0;
			break;
		}
	}

	if (all_over_max)
	{
		min = INT16_MAX;
		for (chan = 0; chan < num_elements; chan++){
			if(a[chan] < min) min = a[chan];
		}

		for (chan = 0; chan < num_elements; chan++){
			a[chan] -= (min - display_max);
		}
		return;
	}

	all_under_min = 1;
	for (chan=0; chan<num_elements; chan++)
	{
		if (a[chan] >= display_min) {
			all_under_min = 0;
			break;
		}
	}
	if (all_under_min)
	{
		max = INT16_MIN;
		for (chan = 0; chan < num_elements; chan++){
			if(a[chan] > max) max = a[chan];
		}

		for (chan = 0; chan < num_elements; chan++){
			a[chan] -= (max - display_min);
		}
	}
}

void compute_transpositions(void)
{
	uint8_t chan;

	for( chan = 0; chan < NUM_CHANNELS; chan++){
		calc_params.transposition[chan] = compute_transposition(calc_params.transpose[chan]);
	}
}

float compute_transposition(int32_t transpose)
{
	int16_t i;
	float transposition;

	transposition = 1.0;


	if(transpose > 0) {
		while(transpose>=12) {transposition *= 2.0; transpose-=12;}
		for (i = 0; i < transpose; i++){
			transposition *= F_SCALING_TRANSPOSE;
		}
	}

	else if (transpose<0) {
		while(transpose<=-12) {transposition /= 2.0; transpose+=12;}
		for (i = 0; i > transpose; i--){
			transposition /= F_SCALING_TRANSPOSE;
		}
	}

	return transposition;
}



// ########################################################################
//						WAVETABLE POSITION + SELECTION  /*** Move to params_wt.c ***/
// ########################################################################


/* Persistent encoder base for each Halo control (per channel).
 * Each tick we combine base + CV offset into the live physics parameter.
 * Without this, the CV-driven path would overwrite any encoder movement on
 * the very next tick, making the encoder useless while a CV is patched.
 *
 * The bases live inside `o_params` (saved with presets, vH+). */
#define halo_damping_base      (params.halo_damping)
#define halo_noise_level_base  (params.halo_noise_level)
#define halo_noise_color_base  (params.halo_noise_color)
#define halo_wt_attack_base    (params.halo_wt_attack)
#define halo_lpf_base          (params.halo_lpf_cutoff)

void read_load_save_encoder(void){
	static uint8_t preset_feature_armed = 0;
	int16_t enc, enc2;

	enc   = pop_encoder_q (pec_LOADPRESET);
	enc2  = pop_encoder_q (sec_SAVEPRESET);

	if (rotary_released(rotm_PRESET) && macro_states.all_af_buttons_released && !button_pressed(butm_LFOVCA_BUTTON) && !button_pressed(butm_LFOMODE_BUTTON))
		preset_feature_armed = 1;

	if (!key_combo_all_but_preset_released()) {
		preset_feature_armed = 0;
		exit_preset_manager();
	}

	if (ui_mode==PLAY && key_combo_all_but_preset_released() && preset_feature_armed)
		handle_preset_events(enc,enc2);
}



void read_switches(void)
{
	uint8_t chan;

	if (switch_pressed(VOCTSW))
	{
		for (chan=0; chan<NUM_CHANNELS; chan++) {
			if (!params.osc_param_lock[chan])
				params.voct_switch_state[chan] = SW_VCA;
		}
	}
	else
	{
		for (chan=0; chan<NUM_CHANNELS; chan++) {
			if (!params.osc_param_lock[chan])
				params.voct_switch_state[chan] = SW_VOCT;
		}
	}
}
