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
#include "globals.h"
#include "analog_conditioning.h"
#include "compressor.h"
#include "system_settings.h"
#include "codec_sai.h"
#include "timekeeper.h"
#include "diag_log.h"
#include "diag_fsk.h"
#include "led_cont.h"
#include "drum_ui.h"

extern o_systemSettings	system_settings;
extern o_led_cont 		led_cont;

void process_audio_block_codec(int32_t * __restrict__ src, int32_t * __restrict__ dst)
{
	extern volatile uint32_t diag_audio_isr_peak_cycles;
	uint32_t audio_isr_start_cycles = DWT->CYCCNT;

	int16_t 		i_sample;
	uint8_t 		chan;

	int32_t			outL, outR;
	float			output_buffer_evens[MONO_BUFSZ] = {0.f};
	float			output_buffer_odds[MONO_BUFSZ] = {0.f};

	static float 	prev_level[NUM_CHANNELS] = {0.f};
	float 			interpolated_level, level_inc;

	/* Per-channel drum render scratch. */
	float			temp_buffer[MONO_BUFSZ];

	(void)src;

	for (chan = 0; chan < NUM_CHANNELS; chan++)
	{
		/* accent_gain folds Grids' accent flag into the same smoothed
		 * level ramp below -- it only steps at a trigger, but the
		 * existing per-block interpolation already exists to avoid
		 * zipper noise on slider moves, so an accent's gain change
		 * rides it for free instead of clicking. */
		float level = drum_chan[chan].level * drum_chan[chan].accent_gain;
		level_inc = (level - prev_level[chan]) / MONO_BUFSZ;
		interpolated_level = prev_level[chan];
		prev_level[chan] = level;

		drum_render_channel(chan, temp_buffer, MONO_BUFSZ);

		/* Drums are mono per channel and panned by channel parity so a
		 * kit spreads across the stereo pair without needing the old
		 * per-channel pan control. Open hat borrows closed hat's parity
		 * for this rather than its own -- they're one physical cymbal
		 * (they already choke each other, see fire() in drum_ui.c) and
		 * should sit in the same place in the stereo field. Kick stays
		 * dead center regardless of parity -- it anchors the kit and
		 * needs to hit evenly in both ears, not lean to one side. */
		uint8_t pan_chan = (chan == DRUM_CAT_OPEN_HAT) ? DRUM_CAT_CLOSED_HAT : chan;
		float pan = (chan == DRUM_CAT_KICK) ? 0.5f : ((pan_chan & 1) ? 0.33f : 0.66f);

		float g = interpolated_level;
		const float g_step = level_inc * 4.0f;

		for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample += 4) {
			float gL = g * pan;
			float gR = g - gL;
			for (int j = 0; j < 4; j++) {
				float t = temp_buffer[i_sample + j];
				output_buffer_evens[i_sample + j] += t * gL;
				output_buffer_odds [i_sample + j] += t * gR;
			}
			g += g_step;
		}
	}

	// Apply Global VCA from LFO CV jack (only when jack is plugged)
	if (analog_jack_plugged(LFO_CV) && lfos.global_vca_level < 1.0f) {
		arm_scale_f32(output_buffer_evens, lfos.global_vca_level, output_buffer_evens, MONO_BUFSZ);
		arm_scale_f32(output_buffer_odds, lfos.global_vca_level, output_buffer_odds, MONO_BUFSZ);
	}

	uint8_t fsk_out_active = diag_log_enabled
	                      && (led_cont.ongoing_display == ONGOING_DISPLAY_CPU_USAGE);

	/* Drum voices were each individually gain-trimmed to avoid clipping
	 * on their own, then further attenuated by the per-channel level
	 * (default 0.8, not 1.0) and by the constant-sum pan law above
	 * (~0.74x power for a split channel) -- the combined result sits
	 * well under the full-scale level master_gain was calibrated for,
	 * so the whole kit reads quiet even before a single voice is close
	 * to clipping. compress() above is a soft asymptotic limiter, not a
	 * hard clip, so it's safe to make up most of that headroom here;
	 * re-tune by ear if a very dense multi-channel pattern audibly
	 * pumps against the limiter. */
	#define DRUM_MIX_MAKEUP_GAIN	1.8f
	const float dac_scale = system_settings.master_gain * DRUM_MIX_MAKEUP_GAIN * (32768.0f * 4095.0f);
	for (i_sample = 0; i_sample < MONO_BUFSZ; i_sample++)
	{
		outL = (int32_t)(output_buffer_evens[i_sample] * dac_scale);
		outR = (int32_t)(output_buffer_odds[i_sample] * dac_scale);

		*dst++ = compress(outL);
		if (fsk_out_active) {
			*dst++ = diag_fsk_next_sample();
		} else {
			*dst++ = compress(outR);
		}
	}

	{
		uint32_t dur = DWT->CYCCNT - audio_isr_start_cycles;
		if (dur > diag_audio_isr_peak_cycles)
			diag_audio_isr_peak_cycles = dur;
		diag_log(DIAG_EVT_AUDIOISR, dur);
	}
}


void update_oscillators(void){

	extern volatile uint32_t diag_osc_tim_peak_cycles;
	uint32_t osc_tim_start_cycles = DWT->CYCCNT;

	/* Pattern advance + trigger arming lives here rather than in the
	 * main loop: OSC_TIM's fixed ~1.8 kHz tick bounds the jitter between
	 * a clock edge and the hit landing, which the superloop (variable
	 * period, flash I/O in it) cannot. */
	update_drum_triggers();

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

