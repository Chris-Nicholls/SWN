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
#include "params_lfo.h"
#include "plaits_shim.h"
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
extern o_analog 		analog[NUM_ANALOG_ELEMENTS];

extern o_recbuf 		recbuf;
__attribute__((aligned(32))) o_wt_osc	wt_osc;
uint8_t 				audio_in_gate;

//Private:
void update_sphere_wt(void);

void process_audio_block_codec(int32_t * __restrict__ src, int32_t * __restrict__ dst)
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

	float 			audio_in_sum = 0.0f;
	static uint8_t	audio_gate_ctr=0;

	float			read_pos;

	// Resonator mode variables
	uint8_t			resonator_mode_active;
	float			audio_in_buffer[MONO_BUFSZ];
	
	// Plaits/Wavetable accumulation buffer
	float			temp_buffer[MONO_BUFSZ];
	uint8_t			is_plaits_mode;

	oscout_status = 	((ui_mode != WTRECORDING) && (ui_mode != WTMONITORING) && (ui_mode != WTREC_WAIT));
	audiomon_status = 	((ui_mode == WTRECORDING) || (ui_mode == WTMONITORING) || (ui_mode == WTREC_WAIT) || (ui_mode == WTTTONE));

	resonator_mode_active = jack_plugged(WAVEFORMIN_SENSE) && oscout_status;

	// 1. UNIFIED INPUT READING
	int32_t *src_ptr = src;
	for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++) {
		audio_in_sample = convert_s24_to_s32(*src_ptr++);
		src_ptr++; // ignore right
		audio_in_buffer[i_sample] = (float)audio_in_sample / 8388608.0f;
		if (audio_in_sample < 0) audio_in_sum += (float)audio_in_sample;
	}

	for (chan = 0; chan < NUM_CHANNELS; chan++)
	{
		read_level_and_pan(chan);
		level_inc = (calc_params.level[chan] - prev_level[chan]) / MONO_BUFSZ;
		interpolated_level = prev_level[chan];
		prev_level[chan] = calc_params.level[chan];
		
		pan_inc = (params.pan[chan] - prev_pan[chan]) / MONO_BUFSZ;
		interpolated_pan = prev_pan[chan];
		prev_pan[chan] = params.pan[chan];

		is_plaits_mode = (params.wt_bank[chan] >= PLAITS_SPHERE_OFFSET);

		if (is_plaits_mode) {
			// --- PLAITS PATH ---
			if (params.note_on[chan]) {
				PlaitsParams p;
				p.note = 69.0f + 12.0f * log2f(calc_params.pitch[chan] / 440.0f);
				p.harmonics = calc_params.wt_pos[0][chan] / 2.0f;
				p.timbre = calc_params.wt_pos[1][chan] / 2.0f;
				p.morph = calc_params.wt_pos[2][chan] / 2.0f;
				p.engine = params.wt_bank[chan] - PLAITS_SPHERE_OFFSET;
				if (p.engine < 0) p.engine = 0;
				if (p.engine > 23) p.engine = 23; 
				p.lpg_decay = (float)lfos.shape[chan] / (float)NUM_LFO_SHAPES;
				p.lpg_color = (float)params.plaits_lpg_color[chan] / 255.0f;
				p.mod_timbre = (float)params.plaits_timbre_mod[chan] / 127.0f;
				p.mod_morph = (float)params.plaits_morph_mod[chan] / 127.0f;
				p.mod_harmonics = (float)params.plaits_harmonics_mod[chan] / 127.0f;
				p.mod_freq = (float)params.plaits_freq_mod[chan] / 127.0f;
				p.output_mode = params.plaits_aux_select[chan];
				p.use_internal_lpg = calc_params.plaits_use_lpg[chan];
				
				p.trigger = 0.0f;
				if (params.key_sw[chan] != ksw_MUTE) {
					if (params.new_key[chan] || lfos.trigout[chan]) p.trigger = 1.0f;
				}
				uint8_t jack_is_plugged = (analog[A_VOCT + chan].plug_sense_switch.pressed == PRESSED);
				if ((params.voct_switch_state[chan] == SW_VCA) && jack_is_plugged) {
					float vca_cv = (analog[A_VOCT + chan].polarity == AP_UNIPOLAR) ? (analog[A_VOCT + chan].lpf_val / 4095.0f) : (_CLAMP_F(analog[A_VOCT + chan].lpf_val - 2048.0f, 0.0f, 2048.0f) / 2048.0f);
					if (vca_cv > 0.2f && wt_osc.plaits_last_cv_input[chan] <= 0.2f) p.trigger = 1.0f;
					wt_osc.plaits_last_cv_input[chan] = vca_cv;
				} else {
					wt_osc.plaits_last_cv_input[chan] = 0.0f;
				}

				Plaits_Render(chan, &p, temp_buffer, MONO_BUFSZ);

				if (lfos.mode[chan] == lfot_LPG && lfos.to_vca[chan]) {
					if (lfos.trigout[chan]) Shim_LPG_Trigger(chan);
					Shim_LPG_Process(chan, temp_buffer, MONO_BUFSZ, p.lpg_decay, p.lpg_color);
				}
			} else {
				for(int k=0; k<MONO_BUFSZ; k++) temp_buffer[k] = 0.0f;
			}

			for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++) {
				smpl = temp_buffer[i_sample] * 32768.0f * interpolated_level;
				output_buffer_evens[i_sample] += smpl * interpolated_pan;
				output_buffer_odds[i_sample] += smpl * (1.0f - interpolated_pan);
				interpolated_level += level_inc;
				interpolated_pan += pan_inc;
			}
		} else {
			// --- WAVETABLE PATH ---
			uint8_t voice_count = params.unison_voice_count[chan];
			float voice_scale = (voice_count > 1) ? (1.2f / sqrtf((float)voice_count)) : 1.0f;

			if (wt_osc.wt_xfade[chan] > 0.0f) {
				wt_osc.wt_xfade[chan] -= XFADE_INC;
				if (wt_osc.wt_xfade[chan] < 0.0f) wt_osc.wt_xfade[chan] = 0.0f;
			} else {
				static const float osc_detune_factors[8] = {0, -0.0024f, 0.0024f, -0.0056f, 0.0056f, -0.01f, 0.01f, -0.016f};
				float base_inc = wt_osc.wt_head_pos_inc[chan][0];
				for (uint8_t v = 0; v < voice_count; v++) {
					wt_osc.wt_head_pos_inc[chan][v] = base_inc * (1.0f + (osc_detune_factors[v] * params.unison_spread_amt[chan] * 4.0f));
				}
			}
			fade_gain_prev = wt_osc.wt_xfade[chan];
			fade_gain_current = 1.0f - fade_gain_prev;

			for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++) {
				float s = 0.0f;
				for (uint8_t v = 0; v < voice_count; v++) {
					wt_osc.wt_head_pos[chan][v] += wt_osc.wt_head_pos_inc[chan][v];
					if (wt_osc.wt_head_pos[chan][v] >= WT_TABLELEN) wt_osc.wt_head_pos[chan][v] -= WT_TABLELEN;
					read_pos = wt_osc.wt_head_pos[chan][v];
					uint16_t rh0 = (uint16_t)read_pos;
					uint16_t rh1 = (rh0 + 1) & (WT_TABLELEN - 1);
					float rhd = read_pos - (float)rh0;
					float smpl_v = wt_osc.mc[wt_osc.buffer_sel[chan]][chan][rh0] * (1.0f - rhd) + wt_osc.mc[wt_osc.buffer_sel[chan]][chan][rh1] * rhd;
					if (fade_gain_prev > 0.0f) {
						float smpl_v2 = wt_osc.mc[1-wt_osc.buffer_sel[chan]][chan][rh0] * (1.0f - rhd) + wt_osc.mc[1-wt_osc.buffer_sel[chan]][chan][rh1] * rhd;
						s += smpl_v * fade_gain_current + smpl_v2 * fade_gain_prev;
					} else {
						s += smpl_v;
					}
				}
				temp_buffer[i_sample] = s * voice_scale;
			}

			if (lfos.mode[chan] == lfot_LPG && lfos.to_vca[chan]) {
				for(int k=0; k<MONO_BUFSZ; k++) temp_buffer[k] /= 32768.0f;
				if (lfos.trigout[chan]) Shim_LPG_Trigger(chan);
				float decay = _CLAMP_F((float)lfos.shape[chan] / (float)NUM_LFO_SHAPES, 0.0f, 1.0f);
				float color = (float)params.plaits_lpg_color[chan] / 255.0f;
				Shim_LPG_Process(chan, temp_buffer, MONO_BUFSZ, decay, color);
				for(int k=0; k<MONO_BUFSZ; k++) temp_buffer[k] *= 32768.0f;
			}

			for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++) {
				smpl = temp_buffer[i_sample];
				if (resonator_mode_active) {
					float read_pos_Q = wt_osc.wt_head_pos[chan][0] + (WT_TABLELEN / 4);
					if (read_pos_Q >= (float)WT_TABLELEN) read_pos_Q -= (float)WT_TABLELEN;
					uint16_t rh0_Q = (uint16_t)read_pos_Q;
					uint16_t rh1_Q = (rh0_Q + 1) & (WT_TABLELEN - 1);
					float rhd_Q = read_pos_Q - (float)rh0_Q;
					float osc_Q = (wt_osc.mc[wt_osc.buffer_sel[chan]][chan][rh0_Q] * (1.0f - rhd_Q)) + (wt_osc.mc[wt_osc.buffer_sel[chan]][chan][rh1_Q] * rhd_Q);
					float ring_I = (smpl / 32768.0f) * audio_in_buffer[i_sample];
					float ring_Q = (osc_Q / 32768.0f) * audio_in_buffer[i_sample];
					float dc_alpha = 100.0f / F_SAMPLERATE;
					wt_osc.coherence_dc_I[chan] = dc_alpha * ring_I + (1.0f - dc_alpha) * wt_osc.coherence_dc_I[chan];
					wt_osc.coherence_dc_Q[chan] = dc_alpha * ring_Q + (1.0f - dc_alpha) * wt_osc.coherence_dc_Q[chan];
					float dc_mag = sqrtf(wt_osc.coherence_dc_I[chan] * wt_osc.coherence_dc_I[chan] + wt_osc.coherence_dc_Q[chan] * wt_osc.coherence_dc_Q[chan]);
					float env_alpha = (dc_mag > wt_osc.coherence_env[chan]) ? (params.resonator_attack_freq / F_SAMPLERATE) : (params.resonator_decay_freq / F_SAMPLERATE);
					wt_osc.coherence_env[chan] += env_alpha * (dc_mag - wt_osc.coherence_env[chan]);
					smpl *= wt_osc.coherence_env[chan];
				}
				smpl *= interpolated_level;
				output_buffer_evens[i_sample] += smpl * interpolated_pan;
				output_buffer_odds[i_sample] += smpl * (1.0f - interpolated_pan);
				interpolated_level += level_inc;
				interpolated_pan += pan_inc;
			}
		}
	}

	// Apply soft clipping to mixed output buffers
	// Formula: out = tanh(x * pregain) / pregain
	// if (oscout_status) {
	// 	float pregain = params.soft_clip_pregain;
	// 	if (pregain < 0.05f) pregain = 0.05f; // Safety against div/0

	// 	// Non-linear compensation: 1.0 / sqrt(pregain)
	// 	// Maintains more volume when cranking saturation
	// 	float inv_pregain = 1.0f / sqrtf(pregain);
 
	// 	// 8388608 = 2^23 = max value for signed 24-bit audio
	// 	float scaler = 8388608.0f * 8.0f;
	// 	float inv_scaler = 1.0f / scaler;

	// 	for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++) {
	// 		float in_even = output_buffer_evens[i_sample] * inv_scaler;
	// 		float in_odd = output_buffer_odds[i_sample] * inv_scaler;

	// 		output_buffer_evens[i_sample] = tanhf(in_even * pregain) * inv_pregain * scaler;
	// 		output_buffer_odds[i_sample] = tanhf(in_odd * pregain) * inv_pregain * scaler;
	// 	}
	// }

	// Apply EQ after soft clipping
	// eq_process(output_buffer_evens, output_buffer_odds, MONO_BUFSZ);
	
	// 4. FINAL OUTPUT COMPRESSION & GATE
	for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++)
	{
		outL = 0; outR = 0;
		if (oscout_status) {
			outL = (int32_t)(output_buffer_evens[i_sample] * system_settings.master_gain);
			outR = (int32_t)(output_buffer_odds[i_sample] * system_settings.master_gain);
		}
		if (audiomon_status) {
			int32_t mon_smpl = (int32_t)(audio_in_buffer[i_sample] * 8388608.0f);
			outL += mon_smpl;
			outR += mon_smpl;
		}
		*dst++ = compress(outL);
		*dst++ = compress(outR);
	}

	if (audio_in_sum < AUDIO_GATE_THRESHOLD) {
		if (++audio_gate_ctr >= AUDIO_GATE_DEBOUNCE_LENGTH) { audio_in_gate = 1; audio_gate_ctr = 0; }
	} else audio_in_gate = 0;
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
		wt_osc.plaits_last_cv_input[i] = 0.0f;
	}

	Plaits_Init();
	Shim_LPG_Init();
}
