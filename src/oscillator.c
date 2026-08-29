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
#include "halo.h"
#include "halo_voice.hpp"   /* extern "C" facade — active engine */
#include <string.h>
#include "arm_math.h"
#include "params_lfo.h"
#include "plaits_shim.h"
#include "sphere_flash_io.h"
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
#include "reverb.h"
#include "reverb_ui.h"
#include "flashram_spidma.h"
#include "diag_log.h"
#include "diag_fsk.h"
#include "led_cont.h"

extern enum UI_Modes 	ui_mode;
extern o_rotary 		rotary[NUM_ROTARIES];
extern o_params 		params;
extern o_calc_params	calc_params;
extern o_systemSettings	system_settings;
extern o_led_cont 		led_cont;
extern o_analog 		analog[NUM_ANALOG_ELEMENTS];

extern o_recbuf 		recbuf;
__attribute__((aligned(32))) o_wt_osc	wt_osc;

/* Physics-cost diagnostic — peak duration of one halo_advance_cycle()
 * call in DWT cycles.  Now updated from the audio ISR (which owns the
 * advance pass) and from halo_tick (trigger fast path in
 * OSC_TIM).  Drained / displayed by led_cont.c. */
extern volatile uint32_t diag_advance_cycle_peak_cycles;

//Private:
void update_sphere_wt(void);

void process_audio_block_codec(int32_t * __restrict__ src, int32_t * __restrict__ dst)
{
	/* Temporary: the SAI DMA audio ISR runs at priority 0,0 and preempts
	 * every other ISR including OSC_TIM — if anything inside here ever
	 * takes >50 ms it would explain the ~200 ms main-loop period spike
	 * visible on the outer LED ring in chord mode.  Peak committed at
	 * the end of the function and displayed on inner LED ring LED 2. */
	extern volatile uint32_t diag_audio_isr_peak_cycles;
	uint32_t audio_isr_start_cycles = DWT->CYCCNT;

	int16_t 		i_sample;
	uint8_t 		chan;

	int32_t			audio_in_sample, outL, outR;
	float			output_buffer_evens[MONO_BUFSZ] = {0.f};
	float			output_buffer_odds[MONO_BUFSZ] = {0.f};

	float 			oscout_status, audiomon_status;

	static float 	prev_level[NUM_CHANNELS] = {0.f};
	float 			interpolated_level, level_inc;

	static float 	prev_pan[NUM_CHANNELS] = {0.f};
	float 			interpolated_pan, pan_inc;

	/* Waveform-in jack samples, only populated when audiomon_status
	 * is set (WT recording / monitoring screens) so the user can hear
	 * the input while staging a wavetable rec. */
	int32_t			audio_in_raw[MONO_BUFSZ];
	
	// Per-channel accumulation buffer
	float			temp_buffer[MONO_BUFSZ];

	// Reverb send accumulation (wavetable channels only; cleared each block)
	float			reverb_send_L[MONO_BUFSZ] = {0.f};
	float			reverb_send_R[MONO_BUFSZ] = {0.f};
	/* Set if any channel actually contributed audio to the reverb send
	 * bus (chan_wet > 0.001f).  When clear we skip the whole reverb
	 * processing block — Reverb_Process + the pre-saturator + the
	 * output add together cost ~80 µs in the audio ISR.  Reverb is
	 * frequently switched off entirely (chan_send=0 on every channel),
	 * which makes that 80 µs pure overhead. */
	uint8_t			any_reverb_active = 0;

	oscout_status = 	((ui_mode != WTRECORDING) && (ui_mode != WTMONITORING) && (ui_mode != WTREC_WAIT));
	audiomon_status = 	((ui_mode == WTRECORDING) || (ui_mode == WTMONITORING) || (ui_mode == WTREC_WAIT) || (ui_mode == WTTTONE));

	// 1. UNIFIED INPUT READING — only needed for the audiomon path.
	if (audiomon_status) {
		int32_t *src_ptr = src;
		for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++) {
			audio_in_sample = convert_s24_to_s32(*src_ptr++);
			src_ptr++; // ignore right
			audio_in_raw[i_sample] = audio_in_sample;
		}
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

		// 1. Unified Trigger Detection & Policy (Refractory Lockout)
		if (wt_osc.plaits_refractory_timer[chan] > 0) {
			if (wt_osc.plaits_refractory_timer[chan] > MONO_BUFSZ) 
				wt_osc.plaits_refractory_timer[chan] -= MONO_BUFSZ;
			else 
				wt_osc.plaits_refractory_timer[chan] = 0;
		}

		uint8_t jack_is_plugged = (analog[A_VOCT + chan].plug_sense_switch.pressed == PRESSED);
		float vca_cv = (analog[A_VOCT + chan].polarity == AP_UNIPOLAR) ? (analog[A_VOCT + chan].lpf_val / 4095.0f) : (_CLAMP_F(analog[A_VOCT + chan].lpf_val - 2048.0f, 0.0f, 2048.0f) / 2048.0f);
		
		uint8_t jack_trig = 0;
		if (vca_cv > 0.2f && wt_osc.plaits_last_cv_input[chan] <= 0.2f && wt_osc.plaits_refractory_timer[chan] == 0) {
			jack_trig = 1;
			wt_osc.plaits_refractory_timer[chan] = 480; // ~10ms refractory lockout
		}
		wt_osc.plaits_last_cv_input[chan] = vca_cv;

		uint8_t lpg_active = (lfos.mode[chan] == lfot_LPG && lfos.to_vca[chan]);

		// Halo trigger — three modes based on envelope routing:
		//
		// 1. No VCA (to_vca off): No envelope controls the string.
		//    Noise excitation is constant (user's noise level control).
		//    new_key / jack gate → reseed halo immediately.
		//    No LPG processing on output.
		//
		// 2. LPG (lfot_LPG && to_vca): LPG controls output VCA/filter.
		//    new_key / jack gate → Shim_LPG_Trigger + reseed together.
		//    Noise envelope = LPG vactrol level (via externalEnvLevel).
		//    LPG is single-shot — does NOT retrigger on LFO zero crossing.
		//    Chord path: params_update.c handles with strum delay.
		//
		// 3. LFO (to_vca, non-LPG): LFO shapes output level & noise env.
		//    LFO zero crossing → reseed halo (new cycle).
		//    new_key / jack gate → reseed.
		//    Chord path: params_update.c reseeds on chord change.
		//
		// jack_trig_gate: only when switch is SW_VCA (gate input).
		// SW_VOCT jacks carry pitch CV that crosses the trigger threshold
		// on note changes, so they must not produce spurious triggers.

		static uint8_t prev_lfo_trigout[NUM_CHANNELS] = {0};
		uint8_t lfo_edge_raw = lfos.trigout[chan] && !prev_lfo_trigout[chan];
		prev_lfo_trigout[chan] = lfos.trigout[chan];
		uint8_t jack_trig_gate = jack_trig && (params.voct_switch_state[chan] == SW_VCA);

		/* Edge-detect params.new_key[]: it is a *latched* flag set by the
		 * key/chord path and cleared later by update_pitch() in OSC_TIM.
		 * The audio ISR runs at the highest priority and can observe
		 * new_key=1 across several consecutive blocks before OSC_TIM gets
		 * a chance to clear it — the level-triggered reads here would
		 * then fire multiple (re)seeds and LPG triggers per single key
		 * press. Keep a per-channel previous-state shadow so each latch
		 * produces exactly one trigger edge. */
		static uint8_t prev_new_key[NUM_CHANNELS] = {0};
		uint8_t new_key_edge = params.new_key[chan] && !prev_new_key[chan];
		prev_new_key[chan] = params.new_key[chan];

		{
			// --- HALO PATH ---
			// Trigger detection runs in the audio ISR (priority 0,0); the
			// physics is rendered by the cpp-class voice (streaming /
			// amortised across audio samples) via halo_fill_block.
			// All buffer/flip/crossfade machinery moved INSIDE the class;
			// here we only set the trigger flag.
			o_halo *rs = &wt_osc.halo_state[chan];

			uint8_t do_reseed = 0;
			uint8_t do_lpg_trigger = 0;

			if (!lfos.to_vca[chan]) {
				// Mode 1: No VCA envelope — explicit events reseed only
				if (new_key_edge || jack_trig_gate)
					do_reseed = 1;
			} else if (lpg_active) {
				// Mode 2: LPG — trigger LPG + reseed together (single-shot)
				if (new_key_edge || jack_trig_gate) {
					do_lpg_trigger = 1;
					do_reseed = 1;
				}
				// LPG does NOT loop — no LFO zero-crossing retrigger
			} else {
				// Mode 3: LFO-VCA — LFO zero crossing reseeds
				if (lfo_edge_raw)
					do_reseed = 1;
				if (new_key_edge || jack_trig_gate) {
					do_reseed = 1;
					/* Defer the LFO phase reset to OSC_TIM's per-channel
					 * trigger atomic block — see history for full rationale
					 * (LFO must reset in lockstep with the seed/pitch
					 * commit, otherwise the envelope re-attacks before
					 * pitch updates and produces an audible click-then-
					 * pitch-shift artefact). */
					for (uint8_t lc = 0; lc < NUM_CHANNELS; lc++) {
						if (!lfos.locked[lc] && lfos.to_vca[lc] && lfos.mode[lc] != lfot_LPG) {
							lfos.lfo_reset_pending[lc] = 1;
							prev_lfo_trigout[lc] = 0;
						}
					}
				}
			}

			if (do_reseed)
				rs->triggerPending = 1;
			if (do_lpg_trigger)
				Shim_LPG_Trigger(chan);

			// VCA mode: CV drives noise level (single float store is atomic).
			if (params.voct_switch_state[chan] == SW_VCA && jack_is_plugged) {
				rs->noiseLevel = vca_cv * 0.5f;
				halo_set_noise_level(chan, vca_cv * 0.5f);
			}

			// Render this block's audio.  fillBlock handles the read head,
			// streaming physics, and trigger crossfade, and writes
			// samples in the unified ±1.0 float scale used throughout
			// the rest of the audio pipeline.  HaloVoice::fillBlock
			// clamps non-finite samples to 0 before writing.
			halo_fill_block(chan, temp_buffer, MONO_BUFSZ);
		}

		// LPG Processing (keep for amplitude shaping)
		if (lpg_active) {
			float decay = lfos.lpg_decay[chan];
			float color = lfos.lpg_color[chan];
			/* LPG trigger handled above in Halo trigger block */
			Shim_LPG_Process(chan, temp_buffer, MONO_BUFSZ, decay, color);
		}

		// Reverb send for this channel
		float chan_send = params.reverb_send[chan];
		if (chan_send < 0.0f) chan_send = 0.0f;
		if (chan_send > 1.0f) chan_send = 1.0f;
		// Equal-power crossfade: dry^2 + wet^2 = 1 → constant power
		float chan_dry = sqrtf(1.0f - chan_send);
		float chan_wet = sqrtf(chan_send);
		uint8_t chan_has_wet = (chan_wet > 0.001f);
		if (chan_has_wet) any_reverb_active = 1;

		/* Fused mixing loop: gain, pan, dry/wet split, accumulate into
		 * the output and reverb-send buses in one inline pass.  Replaces
		 * 5×arm_scale_f32 + 2–4×arm_add_f32 per 4-sample sub-block.  The
		 * per-call setup of CMSIS-DSP at n=4 dominates the actual work,
		 * so a straight C loop with FPU-vectorised muladd is materially
		 * faster on Cortex-M7.  Level and pan are still held constant
		 * inside each 4-sample block (matching the previous behaviour);
		 * the gain coefficients are pre-multiplied once per sub-block.
		 *
		 * calc_params.level[] is a 0..4095 slider value; the inverse
		 * scale is folded into the level recurrence so the output bus
		 * stays in the unified ±1.0 scale with no per-sample mul.
		 *
		 * The dry- and dry+wet inner loops are split out to keep the
		 * sub-block branch predicted on chan_has_wet only once per
		 * channel (instead of every 4 samples) and to let the wet
		 * gains drop out of the dry-only path entirely.
		 */
		const float kInvLevelMax = 1.0f / 4095.0f;
		float avg_level      = (interpolated_level + level_inc * 1.5f) * kInvLevelMax;
		float avg_pan        = interpolated_pan   + pan_inc   * 1.5f;
		const float lvl_step = level_inc * 4.0f * kInvLevelMax;
		const float pan_step = pan_inc   * 4.0f;

		if (chan_has_wet) {
			for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample += 4) {
				float gL = avg_level * avg_pan;
				float gR = avg_level - gL;            /* = avg_level*(1-avg_pan) */
				float gL_dry = gL * chan_dry;
				float gR_dry = gR * chan_dry;
				float gL_wet = gL * chan_wet;
				float gR_wet = gR * chan_wet;

				for (int j = 0; j < 4; j++) {
					float t = temp_buffer[i_sample + j];
					output_buffer_evens[i_sample + j] += t * gL_dry;
					output_buffer_odds [i_sample + j] += t * gR_dry;
					reverb_send_L      [i_sample + j] += t * gL_wet;
					reverb_send_R      [i_sample + j] += t * gR_wet;
				}

				avg_level += lvl_step;
				avg_pan   += pan_step;
			}
		} else {
			for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample += 4) {
				float gL = avg_level * avg_pan;
				float gL_dry = gL * chan_dry;
				float gR_dry = (avg_level - gL) * chan_dry;

				for (int j = 0; j < 4; j++) {
					float t = temp_buffer[i_sample + j];
					output_buffer_evens[i_sample + j] += t * gL_dry;
					output_buffer_odds [i_sample + j] += t * gR_dry;
				}

				avg_level += lvl_step;
				avg_pan   += pan_step;
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

	// Apply Reverb (wavetable channels only; send levels per channel).
	// The send bus accumulates temp_buffer × (level/4095) × pan × wet
	// already in the unified ±1.0 float scale, so the reverb input
	// soft-clipper, the tank itself, and the user-controlled output
	// level all operate directly on that range with no extra
	// normalisation passes.
	/* Skip the entire reverb pipeline when no channel is sending.
	 * Reverb_Process + the input saturator + the output add account
	 * for ~80 µs of audio ISR; with reverb fully off (chan_send=0
	 * everywhere) that work has zero acoustic effect — the tank just
	 * processes silence into silence — and starves PWM_OUTS_TIM for no
	 * reason.  Note: when the user dials reverb back in the tank starts
	 * cold (zeroed state) since we haven't been advancing it, but the
	 * tank's ~1 s decay means it builds up imperceptibly. */
	if (oscout_status && any_reverb_active) {
		Reverb_SetParams(params.reverb_time, params.reverb_diffusion, params.reverb_lp);

		/* Input drive + soft-clip: replace tanhf(x*ig)/ig with the
		 * stmlib SoftLimit Padé approximation
		 *     y = x*(27 + x²) / (27 + 9x²)
		 * which is monotonic, has the same -1..+1 bound and the same
		 * tanh-like shape near zero, but costs ~3 mul + 1 div instead
		 * of a full tanhf (~50–60 cycles).  Hard-clamp at |x*ig|>3 so
		 * the approximation never extrapolates past its useful range. */
		float ig = params.reverb_input_gain;
		if (ig < 0.0f) ig = 0.0f;
		if (ig > 0.0f) {
			float inv_ig = 1.0f / ig;
			for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++) {
				float yL = reverb_send_L[i_sample] * ig;
				float yR = reverb_send_R[i_sample] * ig;
				if      (yL >  3.0f) yL =  1.0f;
				else if (yL < -3.0f) yL = -1.0f;
				else                 yL = yL * (27.0f + yL*yL) / (27.0f + 9.0f*yL*yL);
				if      (yR >  3.0f) yR =  1.0f;
				else if (yR < -3.0f) yR = -1.0f;
				else                 yR = yR * (27.0f + yR*yR) / (27.0f + 9.0f*yR*yR);
				reverb_send_L[i_sample] = yL * inv_ig;
				reverb_send_R[i_sample] = yR * inv_ig;
			}
		}

		{
			extern volatile uint32_t diag_reverb_peak_cycles;
			uint32_t reverb_start_cycles = DWT->CYCCNT;
			Reverb_Process(reverb_send_L, reverb_send_R, MONO_BUFSZ);
			uint32_t reverb_dur = DWT->CYCCNT - reverb_start_cycles;
			if (reverb_dur > diag_reverb_peak_cycles)
				diag_reverb_peak_cycles = reverb_dur;
		}

		float out_level = params.reverb_output_level;
		if (out_level < 0.0f) out_level = 0.0f;
		arm_scale_f32(reverb_send_L, out_level, reverb_send_L, MONO_BUFSZ);
		arm_scale_f32(reverb_send_R, out_level, reverb_send_R, MONO_BUFSZ);
		arm_add_f32(output_buffer_evens, reverb_send_L, output_buffer_evens, MONO_BUFSZ);
		arm_add_f32(output_buffer_odds, reverb_send_R, output_buffer_odds, MONO_BUFSZ);
	}

	// Apply Global VCA from LFO CV jack (only when jack is plugged)
	if (analog_jack_plugged(LFO_CV) && lfos.global_vca_level < 1.0f) {
		arm_scale_f32(output_buffer_evens, lfos.global_vca_level, output_buffer_evens, MONO_BUFSZ);
		arm_scale_f32(output_buffer_odds, lfos.global_vca_level, output_buffer_odds, MONO_BUFSZ);
	}

	// 4. FINAL OUTPUT COMPRESSION & GATE
	//
	// TEMPORARY: when the diagnostic firehose is enabled AND the user
	// has the CPU-usage LED indicator active (long-press LFOSPEED
	// rotary to toggle), the right channel carries an FSK-modulated
	// bit stream of (evt_type, cycles) packets — see inc/diag_fsk.h.
	// Robust through AC coupling, amplitude attenuation, and 16-bit
	// truncation; only zero-crossings and run-lengths matter at the
	// decoder.  Capture at 48 kHz on the right jack and decode with
	// app/diag_decode.py.  Gating on the CPU-usage display means the
	// FSK signal is silent in normal play and only appears while the
	// user is actively diagnosing — keeping the right output usable
	// as a regular audio jack the rest of the time.
	uint8_t fsk_out_active = diag_log_enabled
	                      && (led_cont.ongoing_display == ONGOING_DISPLAY_CPU_USAGE);

	/* Convert the unified ±1.0 float bus into the integer scale that
	 * compress() and the SAI codec expect.  master_gain is the user-
	 * adjustable headroom trim (default 1/48); the 32768×4095 factor
	 * preserves the same nominal output level we used to get when the
	 * mix accumulator held ±32768 × (level/4095) per channel. */
	const float dac_scale = system_settings.master_gain * (32768.0f * 4095.0f);
	for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++)
	{
		outL = 0; outR = 0;
		if (oscout_status) {
			outL = (int32_t)(output_buffer_evens[i_sample] * dac_scale);
			outR = (int32_t)(output_buffer_odds[i_sample] * dac_scale);
		}
		if (audiomon_status) {
			int32_t mon_smpl = audio_in_raw[i_sample];
			outL += mon_smpl;
			outR += mon_smpl;
		}
		*dst++ = compress(outL);
		if (fsk_out_active) {
			*dst++ = diag_fsk_next_sample();
		} else {
			*dst++ = compress(outR);
		}
	}

	/* ── Temporary: commit audio ISR peak duration. ── */
	{
		uint32_t dur = DWT->CYCCNT - audio_isr_start_cycles;
		if (dur > diag_audio_isr_peak_cycles)
			diag_audio_isr_peak_cycles = dur;
		diag_log(DIAG_EVT_AUDIOISR, dur);
	}
}


/* Keep wt_osc.seed_cache[][2][] in sync with wt_osc.pending_seed_pos[].
 * For each channel we cache the two adjacent wavetable entries spanning
 * pending_seed_pos (floor and ceil) so that on the next note trigger the
 * Halo can seed q_back as a smooth lerp between them. Without this
 * the seed would snap between integer positions, which the user hears
 * as a discrete timbre jump while scrolling the browse encoder.
 *
 * Cache invalidation covers both bank changes and position changes —
 * each slot tracks (active_seed_idx, active_seed_bank) independently, so
 * moving the browse encoder one step only reloads the slot that fell
 * off the end (the other slot is still valid at its new role).
 *
 * Runs in OSC_TIM IRQ (priority 1,1) which shares priority with
 * WT_INTERP_TIM, so the two flash-reading timers serialise rather than
 * preempt one another. The audio SAI ISR (priority 0,0) can still
 * preempt us, but it only reads seed_cache via halo_seed_lerp
 * on trigger — and we set active_seed_idx=0xFFFF while DMA is writing
 * to force the lerp to treat that slot as silent instead of garbage. */
static void refresh_ring_seed_caches(void)
{
	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		float pos = wt_osc.pending_seed_pos[c];
		if (pos < 0.0f) pos = 0.0f;
		if (pos > (float)(NUM_WAVEFORMS_IN_SPHERE - 1))
			pos = (float)(NUM_WAVEFORMS_IN_SPHERE - 1);

		uint16_t idx_a = (uint16_t)pos;
		uint16_t idx_b = idx_a + 1;
		if (idx_b >= NUM_WAVEFORMS_IN_SPHERE)
			idx_b = NUM_WAVEFORMS_IN_SPHERE - 1; /* clamp top endpoint */

		uint16_t pending_bank = (uint16_t)params.wt_bank[c];
		uint16_t want_idx[2] = { idx_a, idx_b };

		for (uint8_t s = 0; s < 2; s++) {
			if (wt_osc.active_seed_idx[c][s]  == want_idx[s] &&
			    wt_osc.active_seed_bank[c][s] == pending_bank)
				continue;

			uint8_t wf_flat = want_idx[s] % NUM_WAVEFORMS_IN_SPHERE;
			uint8_t wx = wf_flat % WT_DIM_SIZE;
			uint8_t wy = (wf_flat / WT_DIM_SIZE) % WT_DIM_SIZE;
			uint8_t wz = wf_flat / (WT_DIM_SIZE * WT_DIM_SIZE);

			wt_osc.active_seed_idx[c][s]  = 0xFFFF;
			wt_osc.active_seed_bank[c][s] = 0xFFFF;
			load_extflash_wave_raw(pending_bank,
			                       wt_osc.seed_cache[c][s],
			                       wx, wy, wz);
			/* Block until RX DMA completes so seed_cache is fully
			 * populated before we republish the slot. */
			while (get_flash_state() != sFLASH_NOTBUSY) { ; }
			wt_osc.active_seed_idx[c][s]  = want_idx[s];
			wt_osc.active_seed_bank[c][s] = pending_bank;
		}
	}
}

void update_oscillators(void){
	int8_t chan;

	/* ── Temporary: measure OSC_TIM tick duration ── */
	extern volatile uint32_t diag_osc_tim_peak_cycles;
	uint32_t osc_tim_start_cycles = DWT->CYCCNT;

	check_reset_navigation();
	update_wt();
	read_all_keymodes();

	// Sync seed_cache[][] AFTER update_wt() so any pending_seed_pos change
	// made this tick (by case 15 of update_wt's dispatcher) is picked up now.
	{
		extern volatile uint32_t diag_osc_refresh_peak_cycles;
		uint32_t refresh_start = DWT->CYCCNT;
		refresh_ring_seed_caches();
		uint32_t refresh_dur = DWT->CYCCNT - refresh_start;
		if (refresh_dur > diag_osc_refresh_peak_cycles)
			diag_osc_refresh_peak_cycles = refresh_dur;
	}

	/* Live _wtOriginal refresh: round-robin one channel per tick so the
	 * per-cycle injection source follows browse-encoder drift in real
	 * time.  Without this, wtAttack > 0 would keep feeding the
	 * previous note's waveform into v[] until the next trigger — so a
	 * user scrubbing the browse encoder while holding a sustained
	 * bowed attack would hear no timbre change until they retriggered.
	 *
	 * Cost per call is one lerp-resample into a phys_N-length float
	 * buffer (≤ 512 samples, ~2 µs on F7) so it's cheap enough to do
	 * every tick, but round-robin matches the physics advance pass
	 * and keeps the per-tick OSC_TIM budget tight.  At 1.8 kHz / 6
	 * channels = 300 Hz per-channel refresh, a 55 ms glide between
	 * adjacent wavetable entries gets ≈ 16 refreshes — the audible
	 * timbre morph is essentially continuous. */
	{
		static uint8_t wt_orig_rr = 0;
		uint8_t c = wt_orig_rr;
		wt_orig_rr = (wt_orig_rr + 1) % NUM_CHANNELS;
		o_halo *rs = &wt_osc.halo_state[c];

		float pos = wt_osc.pending_seed_pos[c];
		if (pos < 0.0f) pos = 0.0f;
		if (pos > (float)(NUM_WAVEFORMS_IN_SPHERE - 1))
			pos = (float)(NUM_WAVEFORMS_IN_SPHERE - 1);
		float frac = pos - (float)(uint16_t)pos;

		const int16_t *seed_a = (wt_osc.active_seed_idx[c][0] != 0xFFFF)
		                        ? wt_osc.seed_cache[c][0] : (const int16_t *)0;
		const int16_t *seed_b = (wt_osc.active_seed_idx[c][1] != 0xFFFF)
		                        ? wt_osc.seed_cache[c][1] : (const int16_t *)0;
		/* Critical section: the audio ISR's streaming physics reads
		 * the voice's per-cycle injection source on every step, so
		 * the refresh must be atomic w.r.t. audio.  Lockout =
		 * 512-sample build_seed + M-sample resample inside
		 * load_wavetable, ~10 µs total — well within the audio
		 * block's 1.33 ms budget. */
		static float seed_wave_rr[WT_TABLELEN];
		halo_build_seed_wave(seed_a, seed_b, frac, seed_wave_rr);
		__disable_irq();
		halo_load_wavetable(c, seed_wave_rr);
		__enable_irq();
		(void)rs;
	}

	combine_transpose_spread();
	compute_transpositions();
	update_transpose_cv();

	check_reverb_edit_entry_exit();

	// Per-channel update: read inputs, compute pitch, then handle any
	// pending trigger immediately (cheap path, ~10 µs/ch even at M=512).
	// Heavy advance_cycle work is handled separately below in a
	// round-robin pass — at most one channel per OSC_TIM tick — so a
	// chord retrigger never has to wait behind 5 other voices' physics.
	//
	// Order within the per-channel loop:
	//   1. update_pitch(chan): refreshes calc_params.pitch[chan] and
	//      wt_head_pos_inc[chan][0] using the channel's CURRENT phys_N.
	//   2. If triggerPending: pick the new pitch-adapted phys_N from
	//      calc_params.pitch[chan], invalidate LPF cache, recompute inc
	//      under the new M, then halo_tick consumes the trigger
	//      (seed + buffer flip in ~10 µs).
	extern volatile uint32_t diag_osc_chanloop_peak_cycles;
	extern volatile uint32_t diag_osc_ringtick_peak_cycles;
	extern volatile uint32_t diag_retrigger_peak_cycles[];
	extern volatile uint32_t diag_trigger_arm_cycle[];
	uint32_t chanloop_start = DWT->CYCCNT;

	/* Snapshot triggerPending once so the per-channel pre-work and the
	 * batched commit below see the same flag values.  The audio ISR
	 * (priority 0,0) can preempt update_oscillators (priority 1,1) and
	 * set triggerPending = 1 in response to a key/jack edge.  Without
	 * the snapshot we'd read it twice and could observe (0, 1) — fire
	 * set_pitch_hz first and then the batched trigger second, leaking
	 * pitch ahead of the seed.  The snapshot is consistent: any flag
	 * the audio ISR sets after the snapshot just gets picked up next
	 * OSC_TIM tick (~555 µs latency, inaudible). */
	uint8_t pending_chans[NUM_CHANNELS];
	uint8_t num_pending = 0;
	uint8_t triggering[NUM_CHANNELS];
	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		triggering[c] = wt_osc.halo_state[c].triggerPending;
		if (triggering[c])
			pending_chans[num_pending++] = c;
	}

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

		o_halo *rs = &wt_osc.halo_state[chan];

		/* Drive the noise envelope from whichever envelope is active on
		 * this channel:
		 *   - LPG mode (mode==lfot_LPG && to_vca): LPG vactrol level
		 *   - LFO-VCA (to_vca set, mode==lfot_LFO): LFO shape 0..1
		 *   - Neither: fall through to constant 1.0 so the string is
		 *     continuously excited by noise at its user-set level.
		 *     (An LFO that's running but not routed to the VCA must
		 *     NOT secretly modulate the noise — this would be confusing
		 *     behaviour since the user has explicitly disengaged it.)
		 * lfos.out_lpf[] is updated by envout_pwm.c at the PWM update
		 * rate. The Halo physics reads this field once per cycle
		 * so the envelope appears smoothed by the cycle period. */
		float ext_env = 1.0f;
		if (lfos.to_vca[chan]) {
			ext_env = lfos.out_lpf[chan];
			if (ext_env < 0.0f) ext_env = 0.0f;
			if (ext_env > 1.0f) ext_env = 1.0f;
		}
		rs->externalEnvLevel = ext_env;
		halo_set_external_env(chan, ext_env);

		/* Pitch tracking: only push the current pitch when this
		 * channel is NOT about to fire a trigger this tick (per the
		 * snapshot taken above).  If we wrote pitch here AND
		 * halo_trigger overwrites it below (atomically with the
		 * new seed), the audio ISR can preempt between the two writes
		 * and play the OLD seed at the NEW pitch for a block —
		 * audible as "pitch jumps right before the trigger lands".
		 * Skipping when triggering[] is set lets the trigger commit
		 * pitch atomically with the seed, so audio sees a single
		 * coherent transition. */
		if (!triggering[chan]) {
			halo_set_pitch_hz(chan, calc_params.pitch[chan]);
		}
	}

	/* ── Batched trigger commit ──────────────────────────────────────
	 * Two-phase batch retrigger: PREPARE every pending voice with
	 * audio interrupts ENABLED (heavy work — resample, seed, DC
	 * remove, pre-smooth — written into per-voice staging buffers
	 * that audio doesn't read), then COMMIT every voice inside a
	 * single __disable_irq() block so audio sees the entire chord's
	 * seed/pitch swap on the same audio sample.
	 *
	 * Why split (vs. running the whole trigger inside __disable_irq()
	 * as before): on a six-voice chord retrigger the heavy work was
	 * ~280 µs of IRQ-blocked time, which exceeded the SAI DMA's
	 * audio-block deadline (500 µs/24-sample block at 48 kHz minus
	 * the ~250 µs the audio ISR itself takes).  The DMA underruns
	 * showed up as a "burst of noise" click that was independent of
	 * the voice's actual noise level — purely a side-effect of audio
	 * being preempted past its block budget.  Splitting the work
	 * leaves only memcpy + state reset (~30-50 µs total for 6 voices)
	 * inside __disable_irq(), well below any deadline.
	 *
	 * Atomicity is preserved: every voice's q_/wt_orig_/M_/read_head_
	 * swap still lands within the same __disable_irq() window, so
	 * the audio ISR observes the chord transition as a single
	 * coherent step (the equal-power crossfade then masks that step
	 * over kXfadeLen samples). */
	if (num_pending > 0) {
		uint32_t batch_start = DWT->CYCCNT;
		uint32_t arm_cycles[NUM_CHANNELS];
		for (uint8_t k = 0; k < num_pending; k++)
			arm_cycles[k] = diag_trigger_arm_cycle[pending_chans[k]];

		/* PHASE 1 — prepare staging for every pending voice with
		 * IRQs enabled.  Audio is still rendering the OLD voice via
		 * its live q_/wt_orig_/M_, completely unaffected by our
		 * writes to the per-voice staging buffers. */
		for (uint8_t k = 0; k < num_pending; k++) {
			uint8_t chan = pending_chans[k];

			/* Resolve cache slots.  An in-flight flash DMA leaves
			 * active_seed_idx == 0xFFFF; pass NULL so the seed
			 * builder skips that endpoint instead of reading half-
			 * written DMA data. */
			float pos = wt_osc.pending_seed_pos[chan];
			if (pos < 0.0f) pos = 0.0f;
			if (pos > (float)(NUM_WAVEFORMS_IN_SPHERE - 1))
				pos = (float)(NUM_WAVEFORMS_IN_SPHERE - 1);
			float frac = pos - (float)(uint16_t)pos;
			const int16_t *seed_a = (wt_osc.active_seed_idx[chan][0] != 0xFFFF)
			                        ? wt_osc.seed_cache[chan][0] : (const int16_t *)0;
			const int16_t *seed_b = (wt_osc.active_seed_idx[chan][1] != 0xFFFF)
			                        ? wt_osc.seed_cache[chan][1] : (const int16_t *)0;

			static float seed_wave[WT_TABLELEN];
			halo_build_seed_wave(seed_a, seed_b, frac, seed_wave);

			halo_prepare_trigger(chan, seed_wave, calc_params.pitch[chan]);
		}

		/* PHASE 2 — atomic commit for every prepared voice. */
		__disable_irq();
		for (uint8_t k = 0; k < num_pending; k++) {
			uint8_t chan = pending_chans[k];
			o_halo *rs = &wt_osc.halo_state[chan];

			halo_commit_trigger(chan);
			rs->phys_N = halo_phys_n(chan);
			rs->triggerPending = 0;

			if (lfos.lfo_reset_pending[chan]) {
				if (!lfos.locked[chan] && lfos.to_vca[chan] && lfos.mode[chan] != lfot_LPG) {
					/* Re-derive this channel's phase offset from the
					 * unified phase_spread (clock-period units). */
					lfos.phase_id[chan] = phase_id_from_spread(chan);
					lfos.phase[chan] = calc_lfo_phase(lfos.phase_id[chan]);
					lfos.trig_armed[chan] = 0;
				}
				lfos.lfo_reset_pending[chan] = 0;
			}
		}
		__enable_irq();

		uint32_t batch_dur = DWT->CYCCNT - batch_start;
		if (batch_dur > diag_osc_ringtick_peak_cycles)
			diag_osc_ringtick_peak_cycles = batch_dur;

		for (uint8_t k = 0; k < num_pending; k++) {
			if (arm_cycles[k]) {
				uint32_t delta = DWT->CYCCNT - arm_cycles[k];
				uint8_t chan = pending_chans[k];
				if (delta > diag_retrigger_peak_cycles[chan])
					diag_retrigger_peak_cycles[chan] = delta;
				diag_trigger_arm_cycle[chan] = 0;
			}
		}
	}

	{
		uint32_t chanloop_dur = DWT->CYCCNT - chanloop_start;
		if (chanloop_dur > diag_osc_chanloop_peak_cycles)
			diag_osc_chanloop_peak_cycles = chanloop_dur;
	}

	if (ui_mode == REVERB_EDIT)
		update_reverb_edit_sliders();

	/* ── Temporary: commit OSC_TIM peak duration. ── */
	{
		uint32_t dur = DWT->CYCCNT - osc_tim_start_cycles;
		if (dur > diag_osc_tim_peak_cycles)
			diag_osc_tim_peak_cycles = dur;
		diag_log(DIAG_EVT_OSCTIM, dur);
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

		wt_osc.plaits_last_cv_input[i] = 0.0f;
		wt_osc.plaits_refractory_timer[i] = 0;

		// Init Halo state — the active physics engine lives in the
		// cpp-class voice array (halo_init_all below).  The o_halo
		// struct is kept as a parameter container for led_cont reads
		// and the triggerPending flag.
		halo_init(&wt_osc.halo_state[i], wt_osc.mc[wt_osc.buffer_sel[i]][i]);
		memcpy(wt_osc.mc[wt_osc.buffer_sel[i] ^ 1][i],
		       wt_osc.mc[wt_osc.buffer_sel[i]][i],
		       sizeof(wt_osc.mc[0][0]));
		halo_arm_envelope(&wt_osc.halo_state[i]);

		// Init dual seed cache. 0xFFFF forces a load on the first
		// refresh_ring_seed_caches() tick; until then the two cache
		// slots look "empty" to halo_seed_lerp (NULL pointer
		// path), which leaves the sine primed above untouched.
		wt_osc.pending_seed_pos[i]     = 0.0f;
		wt_osc.active_seed_idx[i][0]   = 0xFFFF;
		wt_osc.active_seed_idx[i][1]   = 0xFFFF;
		wt_osc.active_seed_bank[i][0]  = 0xFFFF;
		wt_osc.active_seed_bank[i][1]  = 0xFFFF;
		memset(wt_osc.seed_cache[i], 0, sizeof(wt_osc.seed_cache[i]));

		// Crossfade state: idle until the first flip event arms it.
		wt_osc.xfade_remaining[i]    = 0;
		wt_osc.xfade_prev_buffer[i]  = 0;
		wt_osc.xfade_prev_head[i]    = 0.0f;
		wt_osc.xfade_prev_M[i]       = RS_N;
		wt_osc.xfade_prev_inc[i]     = 0.0f;
	}

	/* Initialise the active streaming-physics engine.  Each voice
	 * starts with a sine seed and default damping/noise params so
	 * the engine is audible from cycle zero before any encoder/CV
	 * writes. */
	halo_init_all();

	Shim_LPG_Init();
	Reverb_Init();
}
