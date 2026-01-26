/*
 * params_lfo_clk.c
 *
 * Author: Dan Green (danngreen1@gmail.com), Hugo Paris (hugoplho@gmail.com)
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
#include <math.h>
#include "params_lfo.h"
#include "params_lfo_clk.h"
#include "params_lfo_period.h"
#include "gpio_pins.h"
#include "UI_conditioning.h"
#include "math_util.h"
#include "led_cont.h"
#include "params_update.h"
#include "system_settings.h"

extern o_led_cont 		led_cont;
extern o_lfos 			lfos;
extern o_systemSettings	system_settings;

void read_ext_clk(void)
{
	static uint32_t prev_clock_cycles = 0;
	static uint8_t 	clock_up;
	uint32_t cur_cycles;
	float test_period;
	float clock_difference;

	uint8_t		clock_state;

	// Determine source of external clock, and read it
	if (jack_plugged(CLK_SENSE))
		clock_state = CLK_IN();

	else if (system_settings.allow_bus_clock == 1)
		clock_state = BUS_CLK();

	else
		clock_state = 0;

	// Use DWT cycle counter for high-fidelity timing (immune to Systick jitter/stall)
	cur_cycles = DWT->CYCCNT;

		if (clock_state && !clock_up)
		{
			clock_up = 1;

			led_cont.waiting_for_clockin = 0;
			
			// 1 tick = 1/8000 sec = 216,000,000 / 8,000 = 27,000 cycles
			// Use float for high precision
			test_period = (float)(cur_cycles - prev_clock_cycles) / 27000.0f;

			if (prev_clock_cycles && test_period > 0.0f)
			{
				//Divide down audio-rate incoming clocks
				while (test_period < F_LFO_AUDIO_RANGE_PERIOD_L) test_period *= 2.0f;

				clock_difference = fabsf(test_period - lfos.period[REF_CLK]);

				if (clock_difference > 0.0f)
				{
					//Re-sync phases if clock changes by more than 1%
					if ( ((float)clock_difference / (float)lfos.period[REF_CLK]) > 0.01 )
						stage_resync_lfos();

					if (lfos.period[REF_CLK] == 0)
						lfos.period[REF_CLK] = test_period;
					else
						lfos.period[REF_CLK] = (lfos.period[REF_CLK] * 7.0f + (float)test_period) * 0.125f;

					lfos.cycle_pos[REF_CLK] = 0;
					
					//lfos.inc[REF_CLK] = calc_lfo_inc(lfos.period[REF_CLK]);
					flag_all_lfos_recalc();
				}
				lfos.use_ext_clock = 1;
			}

			prev_clock_cycles = cur_cycles;
		}
	// }

 	if (!clock_state)
 	{
 		clock_up = 0;

 		// CLOCK DETECTION TIMEOUT
		uint32_t noclock_elapsed_ticks = (cur_cycles - prev_clock_cycles) / 27000;
		if ( noclock_elapsed_ticks > (lfos.period[REF_CLK]*2))
		{
			// prev_clock_cycles = 0;
			led_cont.waiting_for_clockin = 0;
			lfos.use_ext_clock = 0;
			led_cont.clockin_wait_progress = 0;
		}
		else if ((noclock_elapsed_ticks > lfos.period[REF_CLK]) && lfos.use_ext_clock)
		{
			led_cont.waiting_for_clockin = 1;
			led_cont.clockin_wait_progress = ((float)(noclock_elapsed_ticks)) / (2.0*(float)(lfos.period[REF_CLK]));
		}
	}
}
