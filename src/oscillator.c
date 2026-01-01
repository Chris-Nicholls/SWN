/*
 * oscillator.c
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

#include "oscillator.h"
#include "arm_math.h"
#include "globals.h"
#include "analog_conditioning.h"
#include "audio_util.h"
#include "flash_params.h"
#include "params_update.h"
#include "envout_pwm.h"
#include "adc_interface.h"
#include "timekeeper.h"
#include "compressor.h"
#include "system_settings.h"
#include "codec_sai.h"
#include "ui_modes.h"
#include "wavetable_editing.h"
#include "wavetable_recording.h"
#include "rotary_driver.h"
#include "math_util.h"
#include "gpio_pins.h"
#include "wavetable_play_export.h"
#include "lfo_wavetable_bank.h"

// Per-channel LFO speed multipliers for organic drift (channels 0-5)
// Slightly different rates create evolving phase relationships
const float phase_spread_speed_mult[NUM_CHANNELS] = {
    0.95f, 1.0f, 1.95f, 2.1f, 3.57f, 3.84f 
};

extern enum UI_Modes 	ui_mode;
extern o_rotary 		rotary[NUM_ROTARIES];
extern o_params 		params;
extern o_calc_params	calc_params;
extern o_systemSettings	system_settings;
extern o_led_cont 		led_cont;

extern o_recbuf 		recbuf;
o_wt_osc				wt_osc;
uint8_t 				audio_in_gate;

//Private:
void update_sphere_wt(void);

void process_audio_block_codec(int32_t *src, int32_t *dst)
{
	int16_t 		i_sample;
	uint8_t 		chan;
	float 			smpl;
	float			xfade0, xfade1;
	int32_t			audio_in_sample, outL, outR;
	float			output_buffer_evens[MONO_BUFSZ] = {0.f};
	float			output_buffer_odds[MONO_BUFSZ] = {0.f};

	float 			oscout_status, audiomon_status;

	static float 	prev_level[NUM_CHANNELS] = {0.f};
	float 			interpolated_level, level_inc;

	static float 	prev_pan[NUM_CHANNELS] = {0.f};
	float 			interpolated_pan, pan_inc;

	float 			audio_in_sum;
	static uint8_t	audio_gate_ctr=0;

	// Phase modulation variables
	float			phase_mod_lfo_val;
	float			phase_offset;
	float			read_pos;
	uint16_t		lfo_table_pos;

	// DEBUG0_ON;

	//Todo: use a separate callback for WTTTONE mode, and another one for WTRECORDING/WTMONITORING/WTREC_WAIT
	oscout_status = 	((ui_mode != WTRECORDING) && (ui_mode != WTMONITORING) && (ui_mode != WTREC_WAIT));
	audiomon_status = 	((ui_mode == WTRECORDING) || (ui_mode == WTMONITORING) || (ui_mode == WTREC_WAIT) || (ui_mode == WTTTONE));

	audio_in_sum = 0;

	for (chan = 0; chan < NUM_CHANNELS; chan++)
	{
		// Advance phase modulation LFO per channel (once per buffer for efficiency)
		wt_osc.phase_mod_lfo_pos[chan] += wt_osc.phase_mod_lfo_inc[chan] * MONO_BUFSZ;
		while (wt_osc.phase_mod_lfo_pos[chan] >= 1.0f) wt_osc.phase_mod_lfo_pos[chan] -= 1.0f;

		// Get LFO value from shape table (0-255 from table, normalize to -1 to +1)
		lfo_table_pos = (uint16_t)(wt_osc.phase_mod_lfo_pos[chan] * (LFO_TABLELEN - 1));
		phase_mod_lfo_val = ((float)lfo_wavetable[params.phase_mod_lfo_shape[chan]][lfo_table_pos] / 127.5f) - 1.0f;

		// Calculate phase offset for this channel (Cruinn-style phase spread)
		phase_offset = params.phase_spread_amt[chan] * phase_mod_lfo_val;

		read_level_and_pan(chan);
		level_inc = (calc_params.level[chan] - prev_level[chan]) / MONO_BUFSZ;
		interpolated_level = prev_level[chan];
		prev_level[chan] = calc_params.level[chan];
		
		pan_inc = (params.pan[chan] - prev_pan[chan]) / MONO_BUFSZ;
		interpolated_pan = prev_pan[chan];
		prev_pan[chan] = params.pan[chan];

		for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++)
		{
			wt_osc.wt_head_pos[chan] += wt_osc.wt_head_pos_inc[chan];
			while (wt_osc.wt_head_pos[chan] >= (float)WT_TABLELEN)
				wt_osc.wt_head_pos[chan] -= (float)(WT_TABLELEN);

			// Apply phase offset to read position
			read_pos = wt_osc.wt_head_pos[chan] + phase_offset;
			while (read_pos >= (float)WT_TABLELEN) read_pos -= (float)WT_TABLELEN;
			while (read_pos < 0.0f) read_pos += (float)WT_TABLELEN;

			wt_osc.rh0[chan] 	= (uint16_t)(read_pos);
			wt_osc.rh1[chan] 	= (wt_osc.rh0[chan] + 1) & (WT_TABLELEN-1);
			wt_osc.rhd[chan] 	= read_pos - (float)(wt_osc.rh0[chan]);
			wt_osc.rhd_inv[chan] = 1.0f - wt_osc.rhd[chan];

			xfade0 = (wt_osc.mc[wt_osc.buffer_sel[chan]][chan][wt_osc.rh0[chan]] * wt_osc.rhd_inv[chan]) + (wt_osc.mc[wt_osc.buffer_sel[chan]][chan][wt_osc.rh1[chan]] * wt_osc.rhd[chan]);

			if (wt_osc.wt_xfade[chan] > 0)
			{
				wt_osc.wt_xfade[chan] -= XFADE_INC;
				xfade1 = wt_osc.mc[1-wt_osc.buffer_sel[chan]][chan][wt_osc.rh0[chan]] * wt_osc.rhd_inv[chan] + wt_osc.mc[1-wt_osc.buffer_sel[chan]][chan][wt_osc.rh1[chan]] * wt_osc.rhd[chan];

				smpl = ((xfade0 * (1.0 - wt_osc.wt_xfade[chan])) + (xfade1 * wt_osc.wt_xfade[chan])) * interpolated_level;
			} else {
				smpl = xfade0  * interpolated_level;
			}
			interpolated_level += level_inc;

			output_buffer_evens[i_sample] += smpl * interpolated_pan;
			output_buffer_odds[i_sample] += smpl * (1.f - interpolated_pan);
			interpolated_pan += pan_inc;

			if (chan==5)
			{
				outL=0;
				outR=0;

				audio_in_sample = convert_s24_to_s32(*src++);
				UNUSED(*src++);  // ignore right channel input (not connected in hardware)

				if (oscout_status) {
					// Apply pregain and tanh soft clipping for phase spread volume compensation
					float pregain = params.phase_spread_pregain;
					float clippedL = tanhf(output_buffer_evens[i_sample] * pregain / 8388608.0f) * 8388608.0f;
					float clippedR = tanhf(output_buffer_odds[i_sample] * pregain / 8388608.0f) * 8388608.0f;
					outL = (int32_t)(clippedL * system_settings.master_gain);
					outR = (int32_t)(clippedR * system_settings.master_gain);
				}
				if (audiomon_status) {
					outL += audio_in_sample;
					outR += audio_in_sample;
				}

				*dst++ = compress(outL);
				*dst++ = compress(outR);

				if (audio_in_sample<0)
					audio_in_sum += audio_in_sample;
			}
		}
	}

	//Requires: Min 4V trigger, min 0.25V/ms rise time (@5V = 20ms, @8V = 32ms), 20ms off time between pulses
	if (audio_in_sum < AUDIO_GATE_THRESHOLD)
	{
		if (++audio_gate_ctr >= AUDIO_GATE_DEBOUNCE_LENGTH)
		{
			audio_in_gate = 1;
			audio_gate_ctr = 0;
		}
	}
	else
		audio_in_gate = 0;

	// DEBUG0_OFF;
}


void update_oscillators(void){
	int8_t chan;

	check_reset_navigation();
	update_wt();
	read_all_keymodes();

	combine_transpose_spread();
	compute_transpositions();
	update_transpose_cv();

	read_ext_trigs();

	for (chan = 0; chan < NUM_CHANNELS; chan++){

		if ((ui_mode != SELECT_PARAMS) && (ui_mode != RGB_COLOR_ADJUST)) {

			read_noteon(chan);

			if(ui_mode == PLAY)
			{
				read_lfomode(chan);
				read_lfoto_vca_vco (chan);
			}
		}
		update_pitch (chan);

		if (ui_mode == PLAY)
			update_noise(chan);
	}

}

void start_osc_updates(void){
	start_timer_IRQ(OSC_TIM_number, &update_oscillators);
}

void update_sphere_wt(void){
	render_full_sphere();
	update_wt_interp();
}

void start_osc_interp_updates(void){
	start_timer_IRQ(WT_INTERP_TIM_number, &update_sphere_wt);
}

void init_wt_osc(void) {
	uint8_t  i;

	for (i=0;i<NUM_CHANNELS;i++)
	{
		wt_osc.wt_head_pos[i] 					= 0;
		wt_osc.buffer_sel[i] 					= 0;
		wt_osc.wt_interp_request[i]				= WT_INTERP_REQ_FORCE;

		// Phase modulation LFO runtime state - per channel
		// Spread initial phases so channels don't all hit zero at the same time
		wt_osc.phase_mod_lfo_pos[i] = (float)i / NUM_CHANNELS;
		wt_osc.phase_mod_lfo_inc[i] = (0.5f * phase_spread_speed_mult[i] / F_SAMPLERATE);  // ~0.5Hz default with per-channel variation
	}
}
