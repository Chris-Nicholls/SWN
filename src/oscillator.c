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
#include "eq.h"
#include "UI_conditioning.h"
#include "hardware_controls.h"

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
	float 			fade_gain_current, fade_gain_prev;

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

	float			read_pos;

	// Resonator mode variables
	uint8_t			resonator_mode_active;
	float			audio_in_buffer[MONO_BUFSZ];

	// DEBUG0_ON;

	//Todo: use a separate callback for WTTTONE mode, and another one for WTRECORDING/WTMONITORING/WTREC_WAIT
	oscout_status = 	((ui_mode != WTRECORDING) && (ui_mode != WTMONITORING) && (ui_mode != WTREC_WAIT));
	audiomon_status = 	((ui_mode == WTRECORDING) || (ui_mode == WTMONITORING) || (ui_mode == WTREC_WAIT) || (ui_mode == WTTTONE));

	// Check if resonator mode is active (waveform input jack plugged)
	resonator_mode_active = jack_plugged(WAVEFORMIN_SENSE) && oscout_status;

	// Pre-read audio input buffer for resonator mode (needed for all channels)
	if (resonator_mode_active) {
		int32_t *src_temp = src;
		for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++) {
			audio_in_buffer[i_sample] = (float)convert_s24_to_s32(*src_temp++) / 8388608.0f;
			src_temp++;  // ignore right channel (not connected in hardware)
		}
	}

	audio_in_sum = 0;

	for (chan = 0; chan < NUM_CHANNELS; chan++)
	{
		read_level_and_pan(chan);
		level_inc = (calc_params.level[chan] - prev_level[chan]) / MONO_BUFSZ;
		interpolated_level = prev_level[chan];
		prev_level[chan] = calc_params.level[chan];
		
		pan_inc = (params.pan[chan] - prev_pan[chan]) / MONO_BUFSZ;
		interpolated_pan = prev_pan[chan];
		prev_pan[chan] = params.pan[chan];

		for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++)
		{
			// ---------------------------------------------------------
			// UNISON VOICE LOOP
			// ---------------------------------------------------------
			
			smpl = 0.0f;
			uint8_t voice_count = params.unison_voice_count[chan];
			float voice_scale = 1.0f;
			
			// Simple gain compensation: 1/sqrt(N) to maintain roughly constant power
			if (voice_count > 1) {
				voice_scale = 1.0f / sqrtf((float)voice_count);
				// boost slightly as sqrt can result in perceived volume drop
				voice_scale *= 1.2f; 
			}


			// Crossfade logic (update once per sample)
			if (wt_osc.wt_xfade[chan] > 0.0f) {
				wt_osc.wt_xfade[chan] -= XFADE_INC;
				if (wt_osc.wt_xfade[chan] < 0.0f) wt_osc.wt_xfade[chan] = 0.0f;
				
				fade_gain_prev = wt_osc.wt_xfade[chan];
				fade_gain_current = 1.0f - fade_gain_prev;
			} else {
				fade_gain_prev = 0.0f;
				// Calculate frequencies for new detuned voices (spread around base_inc)
				// Use non-linear spacing for richer sound
				// Factor scaling: Increased for wider unison
				// Original: {0, -0.0012, 0.0012, -0.0028, 0.0028, -0.005, 0.005, -0.008};
				static const float osc_detune_factors[8] = {0, -0.0024f, 0.0024f, -0.0056f, 0.0056f, -0.01f, 0.01f, -0.016f};

				float base_inc = wt_osc.wt_head_pos_inc[chan][0]; // Assuming voice 0 holds the base increment
				for (uint8_t v = 0; v < voice_count; v++) {
					float detune_factor = 1.0f + (osc_detune_factors[v] * params.unison_spread_amt[chan] * 4.0f); // 4x scaling
					wt_osc.wt_head_pos_inc[chan][v] = base_inc * detune_factor;
				}
			}

			for (uint8_t v = 0; v < voice_count; v++) 
			{
				float voice_smpl = 0.0f;
				
				// Increment read head
				wt_osc.wt_head_pos[chan][v] += wt_osc.wt_head_pos_inc[chan][v];
				if (wt_osc.wt_head_pos[chan][v] >= WT_TABLELEN) wt_osc.wt_head_pos[chan][v] -= WT_TABLELEN;

				// Calculate interpolation indices
				read_pos = wt_osc.wt_head_pos[chan][v];
				
				wt_osc.rh0[chan][v] = (uint16_t)read_pos;
				wt_osc.rh1[chan][v] = (wt_osc.rh0[chan][v] + 1) & (WT_TABLELEN - 1);
				wt_osc.rhd[chan][v] = read_pos - (float)wt_osc.rh0[chan][v];
				wt_osc.rhd_inv[chan][v] = 1.0f - wt_osc.rhd[chan][v];

				// Read from wavetable (Crossfade between buffers)
				float smpl_buff1 = wt_osc.mc[wt_osc.buffer_sel[chan]][chan][wt_osc.rh0[chan][v]] * wt_osc.rhd_inv[chan][v] + 
								 wt_osc.mc[wt_osc.buffer_sel[chan]][chan][wt_osc.rh1[chan][v]] * wt_osc.rhd[chan][v];

				if (fade_gain_prev > 0.0f) {
					float smpl_buff2 = wt_osc.mc[1-wt_osc.buffer_sel[chan]][chan][wt_osc.rh0[chan][v]] * wt_osc.rhd_inv[chan][v] + 
									 wt_osc.mc[1-wt_osc.buffer_sel[chan]][chan][wt_osc.rh1[chan][v]] * wt_osc.rhd[chan][v];

					voice_smpl = smpl_buff1 * fade_gain_current + smpl_buff2 * fade_gain_prev;
				} else {
					voice_smpl = smpl_buff1;
				}
				
				smpl += voice_smpl;
			}
			
			smpl *= voice_scale;

			// ---------------------------------------------------------

			// Resonator mode: quadrature ring-modulation for phase-independent coherence
			// (VCA is applied in read_vca_cv() based on coherence_env)
			if (resonator_mode_active) {
				// Compute 90-degree phase shifted oscillator sample (quadrature)
				float read_pos_Q = read_pos + (WT_TABLELEN / 4);
				while (read_pos_Q >= (float)WT_TABLELEN) read_pos_Q -= (float)WT_TABLELEN;
				uint16_t rh0_Q = (uint16_t)(read_pos_Q);
				uint16_t rh1_Q = (rh0_Q + 1) & (WT_TABLELEN - 1);
				float rhd_Q = read_pos_Q - (float)(rh0_Q);
				float osc_Q = (wt_osc.mc[wt_osc.buffer_sel[chan]][chan][rh0_Q] * (1.0f - rhd_Q)) +
				              (wt_osc.mc[wt_osc.buffer_sel[chan]][chan][rh1_Q] * rhd_Q);

				// Ring modulate both I (in-phase) and Q (quadrature) with input
				float audio_norm = audio_in_buffer[i_sample];
				float ring_mod_I = (smpl / 32768.0f) * audio_norm;
				float ring_mod_Q = (osc_Q / 32768.0f) * audio_norm;

				// Stage 1: Slow symmetric LPF extracts DC components
				float dc_alpha = 100.0f / F_SAMPLERATE;
				wt_osc.coherence_dc_I[chan] = dc_alpha * ring_mod_I + (1.0f - dc_alpha) * wt_osc.coherence_dc_I[chan];
				wt_osc.coherence_dc_Q[chan] = dc_alpha * ring_mod_Q + (1.0f - dc_alpha) * wt_osc.coherence_dc_Q[chan];

				// Compute phase-independent magnitude: sqrt(I² + Q²)
				float dc_mag = sqrtf(wt_osc.coherence_dc_I[chan] * wt_osc.coherence_dc_I[chan] +
				                     wt_osc.coherence_dc_Q[chan] * wt_osc.coherence_dc_Q[chan]);

				// Stage 2: Fast attack / slow decay envelope on magnitude
				float attack_alpha = params.resonator_attack_freq / F_SAMPLERATE;
				float decay_alpha = params.resonator_decay_freq / F_SAMPLERATE;
				if (dc_mag > wt_osc.coherence_env[chan]) {
					wt_osc.coherence_env[chan] += attack_alpha * (dc_mag - wt_osc.coherence_env[chan]);
				} else {
					wt_osc.coherence_env[chan] += decay_alpha * (dc_mag - wt_osc.coherence_env[chan]);
				}
			}

			// Apply level after coherence calculation
			smpl *= interpolated_level;
			interpolated_level += level_inc;

			output_buffer_evens[i_sample] += smpl * interpolated_pan;
			output_buffer_odds[i_sample] += smpl * (1.f - interpolated_pan);
			interpolated_pan += pan_inc;

			if (chan==5)
			{
				audio_in_sample = convert_s24_to_s32(*src++);
				UNUSED(*src++);  // ignore right channel input (not connected in hardware)
				
				if (audio_in_sample<0)
					audio_in_sum += audio_in_sample;
			}
		}
	}

	// Apply soft clipping to mixed output buffers
	// Formula: out = tanh(x * pregain) / pregain
	if (oscout_status) {
		float pregain = params.soft_clip_pregain;
		if (pregain < 0.05f) pregain = 0.05f; // Safety against div/0

		// Non-linear compensation: 1.0 / sqrt(pregain)
		// Maintains more volume when cranking saturation
		float inv_pregain = 1.0f / sqrtf(pregain);
 
		// 8388608 = 2^23 = max value for signed 24-bit audio
		float scaler = 8388608.0f * 8.0f;
		float inv_scaler = 1.0f / scaler;

		for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++) {
			float in_even = output_buffer_evens[i_sample] * inv_scaler;
			float in_odd = output_buffer_odds[i_sample] * inv_scaler;

			output_buffer_evens[i_sample] = tanhf(in_even * pregain) * inv_pregain * scaler;
			output_buffer_odds[i_sample] = tanhf(in_odd * pregain) * inv_pregain * scaler;
		}
	}

	// Apply EQ after soft clipping
	eq_process(output_buffer_evens, output_buffer_odds, MONO_BUFSZ);
	
	// Write output samples with compression
	for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++)
	{
		outL = 0;
		outR = 0;
		
		if (oscout_status) {
			outL = (int32_t)(output_buffer_evens[i_sample] * system_settings.master_gain);
			outR = (int32_t)(output_buffer_odds[i_sample] * system_settings.master_gain);
		}
		if (audiomon_status) {
			// Read audio input for monitoring modes (already read in chan==5 loop but need to re-read)
			audio_in_sample = convert_s24_to_s32(src[i_sample * 2]);
			outL += audio_in_sample;
			outR += audio_in_sample;
		}

		*dst++ = compress(outL);
		*dst++ = compress(outR);
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
	uint8_t  i, j;

	for (i=0;i<NUM_CHANNELS;i++){
		wt_osc.buffer_sel[i] 					= 0;
		wt_osc.wt_interp_request[i]				= WT_INTERP_REQ_FORCE;

		// Init standard playback state
	
		for (j=0; j<MAX_UNISON_VOICES; j++){
			wt_osc.wt_head_pos[i][j] 		= 0;
			wt_osc.wt_head_pos_inc[i][j]	= 3.0; //meaningless default
			wt_osc.rh0[i][j]				= 0;
			wt_osc.rh1[i][j]				= 0;
			wt_osc.rhd[i][j]				= 0;
			wt_osc.rhd_inv[i][j]			= 0;
			wt_osc.rhd[i][j]				= 0;
			wt_osc.rhd_inv[i][j]			= 0;
		}

		wt_osc.coherence_dc_I[i] = 0.0f;
		wt_osc.coherence_dc_Q[i] = 0.0f;
		wt_osc.coherence_env[i] = 0.0f;
	}
}
