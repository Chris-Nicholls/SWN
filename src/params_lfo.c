/*
 * params_lfo.c
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

#include <stm32f7xx.h>
#include "params_lfo.h"
#include "params_lfo_period.h"
#include "params_lfo_clk.h"
#include "params_update.h"
#include "params_changes.h"
#include "UI_conditioning.h"
#include "key_combos.h"
#include "preset_manager.h"
#include "math_util.h"
#include "led_cont.h"
#include "hardware_controls.h"
#include "system_settings.h"
#include "analog_conditioning.h"
#include "ui_modes.h"
#include "oscillator.h"


extern o_params params;
extern o_calc_params calc_params;
extern o_led_cont led_cont;
extern o_macro_states 	macro_states;
extern o_systemSettings system_settings;
extern o_analog analog[NUM_ANALOG_ELEMENTS];
extern enum UI_Modes ui_mode;

o_lfos   lfos;

/* Discrete musical-subdivision ladder for the unified phase-spread.
 * Indexed [PHASE_SPREAD_IDX_MIN..PHASE_SPREAD_IDX_MAX] mapped to
 * [0..PHASE_SPREAD_LADDER_LEN-1] (= idx + PHASE_SPREAD_NUM_STOPS_PER_SIDE).
 * Values are per-channel offsets in clock-period units.
 *
 *   idx     0    ±1     ±2     ±3     ±4     ±5     ±6    ±7    ±8   ±9   ±10
 *   offset  0  1/96   1/48   1/24   1/12   1/6   1/3   1/2   1    2    3
 *
 * Reading the table at idx=+8 (or -8) gives ±1.0 — each voice on the
 * next clock beat (the rhythmic transition).  ±9 / ±10 are sparse
 * rhythmic patterns (2/3 beats per voice).  Tighter values (±1..±5)
 * are pure strum territory. */
const float phase_spread_ladder[PHASE_SPREAD_LADDER_LEN] = {
	-3.0f,             /* idx -10 */
	-2.0f,             /* idx -9  */
	-1.0f,             /* idx -8  rhythmic, reverse */
	-(1.0f / 2.0f),    /* idx -7  */
	-(1.0f / 3.0f),    /* idx -6  */
	-(1.0f / 6.0f),    /* idx -5  */
	-(1.0f / 12.0f),   /* idx -4  */
	-(1.0f / 24.0f),   /* idx -3  */
	-(1.0f / 48.0f),   /* idx -2  */
	-(1.0f / 96.0f),   /* idx -1  tightest reverse strum */
	 0.0f,             /* idx  0  unison */
	 (1.0f / 96.0f),   /* idx +1  tightest strum */
	 (1.0f / 48.0f),
	 (1.0f / 24.0f),
	 (1.0f / 12.0f),
	 (1.0f / 6.0f),
	 (1.0f / 3.0f),
	 (1.0f / 2.0f),
	 1.0f,             /* idx +8  rhythmic */
	 2.0f,
	 3.0f,             /* idx +10 sparse rhythmic */
};

static inline float phase_spread_for_idx(int8_t idx) {
	if (idx < PHASE_SPREAD_IDX_MIN) idx = PHASE_SPREAD_IDX_MIN;
	if (idx > PHASE_SPREAD_IDX_MAX) idx = PHASE_SPREAD_IDX_MAX;
	return phase_spread_ladder[(int)idx + PHASE_SPREAD_NUM_STOPS_PER_SIDE];
}

// const float LFO_PHASE_TABLE[LFO_PHASE_TABLELEN]	= {0, 1.0/8.0, 1.0/7.0, 1.0/6.0, 1.0/5.0, 1.0/4.0, 2.0/7.0, 1.0/3.0, 3.0/8.0, 2.0/5.0, 3.0/7.0, 1.0/2.0, 4.0/7.0, 3.0/5.0, 5.0/8.0, 2.0/3.0, 5.0/7.0, 3.0/4.0, 4.0/5.0, 5.0/6.0, 6.0/7.0, 7.0/8.0};

void update_lfos(float multiplier)
{
	update_lfo_params();
	read_ext_clk();
	update_lfo_calcs();
	update_lfo_wt_pos(multiplier);
	update_lfo_sample();
}

void clear_lfo_locks(void)
{
	uint8_t i;
	for (i = 0; i < NUM_CHANNELS; i++)
		lfos.locked[i] = 0;
}

void use_internal_lfo_base(void)
{
	lfos.use_ext_clock = 0;
}

void init_lfos(void)
{
	uint8_t i;

	// Start with unison (all phases = 0)
	for (i = 0; i < NUM_CHANNELS; i++) {
		if (!lfos.locked[i]) lfos.phase_id[i] = 0;
	}

	lfos.phase_switch = 0;
	lfos.phase_spread_idx = 0;
	lfos.phase_spread     = phase_spread_for_idx(0);  // unison
	lfos.global_vca_level = 1.0f;  // Full level (no attenuation)

	if (!lfos.use_ext_clock)
	{
		lfos.period[REF_CLK] 		= LFO_INIT_PERIOD;
		lfos.divmult_id[REF_CLK] 	= LFO_UNITY_DIVMULT_ID;
		lfos.divmult[REF_CLK] 		= 1.0;
		flag_all_lfos_recalc();
	}

	for (i = 0; i < NUM_CHANNELS; i++)
	{
		if (!lfos.locked[i])
		{
			// LFO parameters
			lfos.shape[i]			= 0;
			lfos.gain[i]			= LFO_INIT_GAIN; 
			lfos.cycle_pos[i]		= 0;
			lfos.div_cnt[i]			= 0;
			lfos.out_lpf[i] 		= 1;
			lfos.envout_pwm[i]		= 0;
			lfos.preload[i]			= 0;
			lfos.trigout[i]			= 0;
			lfos.audio_mode[i]		= 0;
			lfos.phase[i] = calc_lfo_phase(lfos.phase_id[i]);
			lfos.trig_armed[i] = 0;
			lfos.lfo_reset_pending[i] = 0;

			// LPG parameters
			lfos.lpg_decay[i]		= 0.5f;
			lfos.lpg_color[i]		= 0.5f;
			lfos.lpg_gain[i]		= 1.0f;
			lfos.lpg_trigger_delay[i] = 0;
		}
	}

	init_lfo_speed();
}

void init_lfos_shape(void)
{
	uint8_t i;
	for (i = 0; i < NUM_CHANNELS; i++){
		if (!lfos.locked[i])
		{
			lfos.shape[i] = 0;
			lfos.mode[i] = lfot_LFO;
		}
	}
}

void init_lfo_speed(void){

	uint8_t chan;

	lfos.divmult_id[GLO_CLK] = LFO_UNITY_DIVMULT_ID;

	for (chan = 0; chan < NUM_CHANNELS; chan++){
		if (!lfos.locked[chan]) {
			lfos.divmult_id[chan] = LFO_UNITY_DIVMULT_ID;
		}
	}

	flag_all_lfos_recalc();
	update_lfo_calcs();
}

void init_lfo_object(o_lfos *t_lfo){
	uint8_t i;

	// init phase with unison (spread = 0)
	t_lfo->phase_id[0] =  0;
	t_lfo->phase_id[1] =  0;
	t_lfo->phase_id[2] =  0;
	t_lfo->phase_id[3] =  0;
	t_lfo->phase_id[4] =  0;
	t_lfo->phase_id[5] =  0;

	for (i = 0; i < NUM_CHANNELS; i++)
	{
		t_lfo->phase[i] = calc_lfo_phase(lfos.phase_id[i]);

		t_lfo->divmult_id[i] 				= LFO_UNITY_DIVMULT_ID;
		t_lfo->divmult_id_global_locked[i] 	= LFO_UNITY_DIVMULT_ID;
		t_lfo->shape[i]						= 0;
		t_lfo->gain[i]						= LFO_INIT_GAIN; 
		t_lfo->locked[i]					= 0;
		t_lfo->mode[i] 						= lfot_LFO;
		t_lfo->to_vca[i] 					= 0;
		t_lfo->muted[i] 					= 0;

		// LPG-specific parameters (independent from LFO params)
		t_lfo->lpg_decay[i]					= 0.5f;
		t_lfo->lpg_color[i]					= 0.5f;
		t_lfo->lpg_gain[i]					= 1.0f;

		t_lfo->out_lpf[i] 					= 1;
		t_lfo->envout_pwm[i]				= 0;
		t_lfo->preload[i]					= 0;

		t_lfo->cycle_pos[i]					= 0;
		t_lfo->div_cnt[i]					= 0;
		t_lfo->wt_pos[i]					= 0;

		t_lfo->trigout[i]					= 0;
		t_lfo->audio_mode[i]				= 0;

		// flags
		t_lfo->trig_armed[i] 				= 0;
		t_lfo->lpg_trigger_delay[i]			= 0;

		t_lfo->to_vca_buf[i] 				= t_lfo->to_vca[i];
	}

	t_lfo->phase_switch 		= 0;
	t_lfo->phase_spread_idx		= 0;
	t_lfo->phase_spread			= phase_spread_for_idx(0);		// Start in unison

	t_lfo->divmult_id[GLO_CLK] 	= LFO_UNITY_DIVMULT_ID;
	t_lfo->cycle_pos[GLO_CLK] 	= 0;

	t_lfo->use_ext_clock 		= 0;
	t_lfo->global_vca_level		= 1.0f;		// Full level (no attenuation)

	t_lfo->period[REF_CLK] 		= LFO_INIT_PERIOD;
	t_lfo->divmult_id[REF_CLK] 	= LFO_UNITY_DIVMULT_ID;
	t_lfo->divmult[REF_CLK] 	= 1.0;
	t_lfo->cycle_pos[REF_CLK] 	= 0;

}



void update_lfo_sample(void)
{
	float 			lfo_frac, pos_in_table;
	uint16_t		rh0, rh1;
	uint8_t			chan;

	{
		for (chan=0; chan<NUM_CHANNELS; chan++)
		{
			if (!lfos.muted[chan])
			{
				pos_in_table		= lfos.wt_pos[chan] * F_LFO_TABLELEN;
				rh0 				= (uint16_t)pos_in_table;
				rh1 				= (rh0 + 1) & (LFO_TABLELEN-1); //if rh0==255, then rh1 should = 0. (255+1)&(255) == 0x100 & 0x0FF == 0
				lfo_frac 			= pos_in_table - (float)rh0;

				// In LPG Mode, we use lfos.shape for DECAY control.
				// We must NOT let it change the Wavetable used for clocking, otherwise timing acts weird.
				// Force Shape 0 (Sine/Standard) for stable triggering in LPG Mode.
				uint8_t shape_idx = (lfos.mode[chan] == lfot_LPG) ? 0 : lfos.shape[chan];
				
				lfos.preload[chan] 	= (((lfo_wavetable[shape_idx][rh0] * (1.0-lfo_frac) + lfo_wavetable[shape_idx][rh1] * lfo_frac))) ;
			}
			else
				lfos.preload[chan] 	= 0;	
		}
	}
}


void update_lfo_params(void)
{
	apply_lfo_reset();
	read_LFO_phase();
	/* read_LFO_shape() and read_LFO_speed_gain() used to live here,
	 * popping pec_LFOSHAPE/pec_LFOSPEED at 7.2 kHz (this runs off
	 * PWM_OUTS_TIM) -- they raced drum_ui.c's own, much-lower-rate
	 * reads of the same queues (main loop) and always won, so the
	 * drum engine's per-channel voice-cycle and clock divide/multiply
	 * never actually saw a turn. Their own side effects
	 * (lfos.shape[]/lfos.gain[]/lfos.lpg_decay[]/lfos.divmult_id[])
	 * were also live in the background; none of that is read by the
	 * drum engine, so removed rather than reconciled.
	 *
	 * read_lfo_cv() (Global VCA ducking off the LFO CV jack) used to
	 * live here too -- that jack is now the pattern-reset trigger
	 * instead (see read_reset_trigger() in drum_ui.c), so the two uses
	 * would have fought over the same physical input. */
}


void apply_lfo_reset(void){
	enum ResetTypes {
		NO_RESET_STAGED,
		RESET_SPEEDS,
		RESET_SHAPES,
		RESET_PHASES,
		RESET_ALL
	};
	static enum ResetTypes reset_staged = NO_RESET_STAGED;
	uint8_t i;

	if (key_combo_reset_lfos_all()){ //reset_all happens immediately and cancels any other staged reset action
		exit_preset_manager();
		stop_all_displays();
		use_internal_lfo_base();
		init_lfos();

		reset_staged = RESET_ALL;
	}

	if (key_combo_reset_lfos_speeds() && !reset_staged){
		exit_preset_manager();
		stop_all_displays();

		reset_staged = RESET_SPEEDS;
	}

	if (key_combo_reset_lfos_shapes() && !reset_staged){
		exit_preset_manager();
		stop_all_displays();

		reset_staged = RESET_SHAPES;
	}

	if (key_combo_reset_lfos_phases() && !reset_staged){
		// exit_preset_manager();
		stop_all_displays();

		lfos.phase_switch = 1- lfos.phase_switch;

		reset_staged = RESET_PHASES;
	}
	
	if (reset_staged && key_combo_reset_lfos_released())
	{
		if (reset_staged == RESET_SPEEDS)
		{
			for ( i =0; i < NUM_CHANNELS; i++) {
				if (!lfos.locked[i])
					stage_resync(i);
			}
			init_lfo_speed();
		}
		else if (reset_staged == RESET_SHAPES) {
			init_lfos_shape();
		}
		else if (reset_staged == RESET_PHASES)
		{
			if (lfos.phase_switch)
			{
				// init phase (0deg)
				if (!lfos.locked[0]) lfos.phase_id[0] =  0;
				if (!lfos.locked[1]) lfos.phase_id[1] =  0;
				if (!lfos.locked[2]) lfos.phase_id[2] =  0;
				if (!lfos.locked[3]) lfos.phase_id[3] =  0;
				if (!lfos.locked[4]) lfos.phase_id[4] =  0;
				if (!lfos.locked[5]) lfos.phase_id[5] =  0;	
			}
			else
			{
				// init phase (60deg)
				if (!lfos.locked[0]) lfos.phase_id[0] =  0;
				if (!lfos.locked[1]) lfos.phase_id[1] =  20;
				if (!lfos.locked[2]) lfos.phase_id[2] =  16;
				if (!lfos.locked[3]) lfos.phase_id[3] =  12;
				if (!lfos.locked[4]) lfos.phase_id[4] =  8;
				if (!lfos.locked[5]) lfos.phase_id[5] =  4;	
			}

			for ( i =0; i < NUM_CHANNELS; i++) {
				if (!lfos.locked[i])
				{
					lfos.phase[i] = calc_lfo_phase(lfos.phase_id[i]);
					lfos.cycle_pos[i] = 0;
				}
			}
		}

		else if (reset_staged == RESET_ALL){
			//here we can do anything upon release of buttons after doing a reset all
		}

		reset_staged = NO_RESET_STAGED;
	}
}

void read_LFO_speed_gain(void)
{
	int16_t	enc, enc2;
		
	enc  = pop_encoder_q(pec_LFOSPEED);
	enc2 = pop_encoder_q(sec_LFOGAIN);

	if(enc)	
		read_lfo_speed(enc);
	else if (enc2)
		update_lfo_gain(enc2);

}


void update_lfo_gain(int16_t turn)
{
	uint8_t i;
	uint8_t channels_changed;
	float  	turn_amt;
	float 	gain[NUM_CHANNELS];

	// Press main wavetable encoder + Press LFO Speed (in gain mode) + turn = adjust soft clip pregain
	if (rotary_pressed(rotm_WAVETABLE)) {
		params.soft_clip_pregain += turn * SOFT_CLIP_PREGAIN_SCALING;
		params.soft_clip_pregain = _CLAMP_F(params.soft_clip_pregain, MIN_SOFT_CLIP_PREGAIN, MAX_SOFT_CLIP_PREGAIN);
		start_ongoing_display_soft_clip();
		return;
	}

	turn_amt = turn * (switch_pressed(FINE_BUTTON) ? F_SCALING_FINE_LFO_GAIN : F_SCALING_LFO_GAIN);

	// Build array of current gains based on mode
	for (i=0; i<NUM_CHANNELS; i++)
		gain[i] = (lfos.mode[i] == lfot_LPG) ? lfos.lpg_gain[i] : lfos.gain[i];

	channels_changed = change_param_f(gain, turn_amt);

	for (i=0; i<NUM_CHANNELS; i++)
	{
		if (channels_changed & (1<<i)) {
			if (lfos.mode[i] == lfot_LPG) {
				// LPG mode: adjust LPG peak level
				lfos.lpg_gain[i] = _CLAMP_F(lfos.lpg_gain[i] + turn_amt, F_SCALING_LFO_GAIN, 1.0);
			} else {
				// LFO mode: adjust LFO gain
				lfos.gain[i] = _CLAMP_F(lfos.gain[i] + turn_amt, F_SCALING_LFO_GAIN, 1.0);
			}
		}
	}
}


void read_lfo_speed(int16_t turn)
{
	uint8_t i;
	uint8_t fine_pressed;
	float turn_amt, test_divmult_id;
	uint8_t any_lpg_mode = 0;

	if (!turn) return;

	// Check if any unlocked channel is in LPG mode (for global behavior)
	for (i=0; i<NUM_CHANNELS; i++) {
		if (!lfos.locked[i] && lfos.mode[i] == lfot_LPG)
			any_lpg_mode = 1;
	}

	// FINE
	turn_amt = turn;
	fine_pressed = switch_pressed(FINE_BUTTON);

	if (fine_pressed)
		turn_amt *= F_SCALING_FINE_LFO_SPEED;
		
	// GLOBAL
	if (macro_states.all_af_buttons_released)
	{
		// LFO mode: adjust divmult_id
		if (!any_lpg_mode) {
			test_divmult_id = lfos.divmult_id[GLO_CLK] + turn_amt;
			if (!fine_pressed) test_divmult_id = (int8_t)(test_divmult_id);

			lfos.divmult_id[GLO_CLK] = _CLAMP_F(test_divmult_id, LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID);

			flag_all_lfos_recalc();

			if (lfos.use_ext_clock)
				stage_resync_lfos();
		}

		// LPG mode: adjust lpg_decay for all LPG channels
		for (i=0; i<NUM_CHANNELS; i++) {
			if (!lfos.locked[i] && lfos.mode[i] == lfot_LPG) {
				lfos.lpg_decay[i] = _CLAMP_F(lfos.lpg_decay[i] + (turn_amt / 20.0f), 0.0f, 1.0f);
			}
		}
	}

	// INDIVIDUAL
	else{
		for (i = 0; i < NUM_CHANNELS; i++){
			if (button_pressed(i))
			{
				calc_params.already_handled_button[i] = 1; 

				if (lfos.mode[i] == lfot_LPG) {
					// LPG mode: adjust lpg_decay for this channel
					lfos.lpg_decay[i] = _CLAMP_F(lfos.lpg_decay[i] + (turn_amt / 20.0f), 0.0f, 1.0f);
				} else {
					// LFO mode: adjust divmult_id for this channel
					lfos.divmult_id[i] = _CLAMP_F(lfos.divmult_id[i] + turn_amt, LFO_MIN_DIVMULT_ID, LFO_MAX_DIVMULT_ID);
					
					flag_lfo_recalc(i);

					if (lfos.use_ext_clock) {
						stage_resync(i);
					}
				}
			}
		}
	}
}

void sync_LFO_phase(void)
{
	uint8_t i;
	uint8_t channels_changed;
	int16_t	phase[NUM_CHANNELS];

	for (i=0; i<NUM_CHANNELS; i++)
		phase[i] = 0;

	channels_changed = change_param_i16(phase, 1);

	if (channels_changed==0b111111)
		stage_resync_lfos();
	else
	{
		for (i=0; i<NUM_CHANNELS; i++)
		{
			if (channels_changed & (1<<i))
				stage_resync(i);
		}
	}
}


void read_LFO_phase(void)
{
	uint8_t			i, fine, enc_pressed;
	int8_t			enc_turn;
	float			enc_amount;
	static uint8_t 	stage_phase_sync=0;
	static uint8_t 	disable_phase_sync=0;

	enc_turn  = pop_encoder_q(sec_LFOPHASE);
	enc_pressed = rotary_pressed(rotm_LFOSHAPE);
	fine = switch_pressed(FINE_BUTTON);

	/* PHASE SPREAD CONTROL (knob pressed + turned)
	 *
	 * The encoder steps a discrete musical ladder (phase_spread_ladder
	 * in this file): each click moves the index by ±1.  Index 0 is
	 * unison; positive walks 1/96-of-a-beat → 1/6 → on-the-beat → 3
	 * beats apart; negative mirrors the same ladder with reversed
	 * voice order.  The cached float `lfos.phase_spread` is the
	 * resolved per-voice offset in clock periods, used directly by
	 * the LPG strum scheduler and indirectly (via apply_phase_spread)
	 * by the LFO mode channels. */
	if (enc_pressed)
	{
		if (enc_turn) {
			/* Each encoder click moves the ladder index by ±1; FINE has
			 * no effect because the ladder stops are already musical. */
			int new_idx = (int)lfos.phase_spread_idx + (int)(-enc_turn);
			if (new_idx < PHASE_SPREAD_IDX_MIN) new_idx = PHASE_SPREAD_IDX_MIN;
			if (new_idx > PHASE_SPREAD_IDX_MAX) new_idx = PHASE_SPREAD_IDX_MAX;
			lfos.phase_spread_idx = (int8_t)new_idx;
			lfos.phase_spread     = phase_spread_for_idx(lfos.phase_spread_idx);

			/* Bulk-apply to LFO-mode channels.  LPG-mode channels read
			 * lfos.phase_spread directly inside the chord-strum
			 * scheduler, so they need no per-channel state here. */
			apply_phase_spread();

			disable_phase_sync = 1;
		} else {
			// Knob pressed but not turned - stage for sync
			if (!disable_phase_sync)
				stage_phase_sync=1;
		}
	}
	else  // Knob not pressed
	{
		// If knob was just released and sync was staged, sync all LFOs
		if (!enc_turn && stage_phase_sync) {
			sync_LFO_phase();
		}
		stage_phase_sync = 0;
		disable_phase_sync = 0;

		/* INDIVIDUAL PHASE NUDGE (knob not pressed + turned).
		 * Only meaningful for LFO-mode channels: lets the user offset
		 * one voice's free-running phase relative to the spread baseline.
		 * LPG-mode channels are timed entirely by phase_spread + the
		 * clock, so individual nudge does not apply. */
		if (enc_turn)
		{
			if (fine)
				enc_amount = -enc_turn * F_SCALING_FINE_LFO_PHASE;
			else
				enc_amount = -enc_turn;

			// GLOBAL (no channel buttons held)
			if (macro_states.all_af_buttons_released){
				for (i = 0; i < NUM_CHANNELS; i++){
					if (!lfos.locked[i] && lfos.mode[i] != lfot_LPG)
					{
						if (!fine)
							lfos.phase_id[i] = _WRAP_I16(lfos.phase_id[i] + enc_amount, 0, LFO_PHASE_TABLELEN);
						else
							lfos.phase_id[i] = _WRAP_F(lfos.phase_id[i] + enc_amount, 0, LFO_PHASE_TABLELEN);
						lfos.phase[i] = calc_lfo_phase(lfos.phase_id[i]);
					}
				}
			}
			// INDIVIDUAL (channel button held)
			else{
				for (i = 0; i < NUM_CHANNELS; i++){
					if(button_pressed(i) && lfos.mode[i] != lfot_LPG)
					{
						if (!fine)
							lfos.phase_id[i] = _WRAP_I16(lfos.phase_id[i] + enc_amount, 0, LFO_PHASE_TABLELEN);
						else
							lfos.phase_id[i] = _WRAP_F(lfos.phase_id[i] + enc_amount, 0, LFO_PHASE_TABLELEN);
						lfos.phase[i] = calc_lfo_phase(lfos.phase_id[i]);

						calc_params.already_handled_button[i] = 1;
					}
				}
			}
		}
	}
}

void read_LFO_shape(void)
{
	uint8_t		i;
	int8_t		enc; 
	
	enc  = pop_encoder_q(pec_LFOSHAPE);

	if(enc)
	{
		if (macro_states.all_af_buttons_released)
		{
			for (i = 0; i < NUM_CHANNELS; i++){
				if (!lfos.locked[i])
				{
					if (lfos.mode[i] == lfot_LPG) {
						// LPG mode: adjust lpg_color (resonance)
						lfos.lpg_color[i] = _CLAMP_F(lfos.lpg_color[i] + (enc / 20.0f), 0.0f, 1.0f);
					} else {
						// LFO mode: adjust shape
						lfos.shape[i] = _WRAP_I16(lfos.shape[i] + enc, 0 , NUM_LFO_SHAPES);
					}

					led_cont.ongoing_lfoshape[i] = 1;
					led_cont.lfoshape_timeout[i] = params.key_sw[i]!=ksw_MUTE;								
				}
			}
		}

		// INDIVIDUAL
		else{
			for (i=0; i < NUM_CHANNELS; i++){ 
				if(button_pressed(i))
				{
					if (lfos.mode[i] == lfot_LPG) {
						// LPG mode: adjust lpg_color (resonance)
						lfos.lpg_color[i] = _CLAMP_F(lfos.lpg_color[i] + (enc / 20.0f), 0.0f, 1.0f);
					} else {
						// LFO mode: adjust shape
						lfos.shape[i] = _WRAP_I16(lfos.shape[i] + enc, 0 , NUM_LFO_SHAPES);
					}

					calc_params.already_handled_button[i] = 1; 

					led_cont.ongoing_lfoshape[i] = 1;
					led_cont.lfoshape_timeout[i] = params.key_sw[i]!=ksw_MUTE;							
				}	
			}
		}
	}	
	
	for (i=0; i < NUM_CHANNELS; i++)
	{
		if (led_cont.ongoing_lfoshape[i]==1){ 
			led_cont.lfoshape_timeout[i]++;
			if(led_cont.lfoshape_timeout[i]> LFOBANK_DISPLAYTMR){
				led_cont.ongoing_lfoshape[i] 	= 0;
				led_cont.lfoshape_timeout[i] 	= 0;
			}
		}
	}
}

float calc_lfo_phase(float phase_id)
{
	return phase_id/((float)LFO_PHASE_TABLELEN);

	// uint8_t i_phase;
	// float f_phase;
	// float phase;

	// i_phase = (uint8_t)phase_id;
	// f_phase = phase_id - (float)i_phase;

	// if (i_phase<(LFO_PHASE_TABLELEN-1)) {
	// 	return _CROSSFADE(LFO_PHASE_TABLE[i_phase], LFO_PHASE_TABLE[i_phase+1], f_phase);
	// }
	// else {
	// 	phase = _CROSSFADE(LFO_PHASE_TABLE[i_phase], 1.0, f_phase);
	// 	if (phase == 1.0) phase = 0.0;
	// 	return phase;
	// }
}


/* ─────────────────────────────────────────────────────────────────────
 *  Unified phase-spread → per-channel phase_id derivation
 *
 *  phase_spread is in clock-period units between adjacent channels.
 *  For an LFO running at divmult[chan] × GLO_CLK, one clock period
 *  corresponds to divmult[chan] LFO cycles, so:
 *
 *      target_delay_seconds = chan * phase_spread * clock_period
 *      target_delay_in_lfo_cycles = chan * phase_spread * divmult[chan]
 *
 *  The LFO output samples wt_pos = cycle_pos + phase, so positive
 *  `phase` _advances_ the output (peak arrives sooner).  To delay the
 *  voice we set phase to a negative offset, then wrap to [0, 1).
 *  Multiplied by LFO_PHASE_TABLELEN to land in phase_id units.
 * ───────────────────────────────────────────────────────────────────── */
float phase_id_from_spread(uint8_t chan)
{
	if (chan >= NUM_CHANNELS) return 0.0f;
	float dm = lfos.divmult[chan];
	if (dm <= 0.0f || !isfinite(dm)) dm = 1.0f;

	float p = -((float)chan) * lfos.phase_spread * dm * (float)LFO_PHASE_TABLELEN;

	/* Wrap to [0, LFO_PHASE_TABLELEN). */
	p -= (float)LFO_PHASE_TABLELEN
	   * floorf(p / (float)LFO_PHASE_TABLELEN);
	if (p < 0.0f) p = 0.0f;                       /* floor() rounding edge */
	if (p >= (float)LFO_PHASE_TABLELEN) p = 0.0f; /* same */
	return p;
}

void apply_phase_spread(void)
{
	for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
		if (lfos.locked[i]) continue;
		if (lfos.mode[i] == lfot_LPG) continue;   /* LPG reads spread directly */
		lfos.phase_id[i] = phase_id_from_spread(i);
		lfos.phase[i]    = calc_lfo_phase(lfos.phase_id[i]);
	}
}


void read_lfo_cv(void)
{
	/* LFO CV jack repurposed as Global VCA, INVERTED.
	 *   0 V (or unplugged)  → full volume  (level = 1.0)
	 *   5 V                 → silent       (level = 0.0)
	 *
	 * The inversion lets the user patch a falling envelope (e.g.
	 * a decaying LFO/EG output sitting at 0 V at rest, briefly
	 * rising to 5 V) and have it duck the synth volume rather than
	 * gate it open — which matches how a "ducker" or sidechain
	 * input is normally wired in modular setups.
	 *
	 * The conditioning chain treats LFO_CV as AP_UNIPOLAR so
	 * bracketed_val is always 0..4095. */
	if (analog_jack_plugged(LFO_CV)) {
		float v = (float)analog[LFO_CV].bracketed_val / 4095.0f;
		if (v < 0.0f) v = 0.0f;
		if (v > 1.0f) v = 1.0f;
		lfos.global_vca_level = 1.0f - v;
	} else {
		lfos.global_vca_level = 1.0f;  // Full volume when unplugged
	}
}


void init_lfo_to_vc_mode(void)
{
	uint8_t i;

	for (i = 0; i < NUM_CHANNELS; i++){
		lfos.to_vca[i] 				= 0;
		lfos.to_vca_buf[i] 			= lfos.to_vca[i];
		lfos.mode[i] 				= 0;
	}																 	  
}



void cache_uncache_all_lfo_to_vca(enum CacheUncache cache_uncache)
{
	static int16_t to_vca[NUM_CHANNELS];

	uint8_t i;
	for (i = 0; i < NUM_CHANNELS; i++){

		switch (cache_uncache)
		{
			case CACHE:
				to_vca[i] = lfos.to_vca[i];
				break;

			case UNCACHE:
				lfos.to_vca[i] = to_vca[i];
				break;		
		}
	}
}

void set_all_lfo_to_vca(uint8_t newstate)
{
	for (uint8_t chan=0; chan<NUM_CHANNELS; chan++)
		lfos.to_vca[chan] = newstate;
}

void cache_uncache_lfomode(uint8_t chan, enum CacheUncache cache_uncache)
{
	static uint8_t cached_mode[NUM_CHANNELS];

	switch (cache_uncache){

		case CACHE:
			cached_mode[chan] = lfos.mode[chan];
			break;

		case UNCACHE:
			lfos.mode[chan] = cached_mode[chan];
			break;
	}
}

void cache_uncache_all_lfomodes(enum CacheUncache cache_uncache)
{
	for (uint8_t chan=0; chan<NUM_CHANNELS; chan++)
		cache_uncache_lfomode(chan, cache_uncache);
}

void set_all_lfo_mode(enum lfoModes mode)
{
	for (uint8_t chan=0; chan<NUM_CHANNELS; chan++)
		lfos.mode[chan] = mode;
}
