/*
 * envout_pwm.c - PWM output for the channel ENV OUT jacks
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

#include "envout_pwm.h"
#include "globals.h"
#include "params_update.h"
#include "params_lfo.h"
#include "led_cont.h"
#include "gpio_pins.h"
#include "timekeeper.h"
#include "hal_handlers.h"
#include "lfo_wavetable_bank.h"
#include "oscillator.h"
#include "ui_modes.h"
#include "drum_ui.h"

extern o_params 	params;
extern o_lfos 		lfos;
extern o_led_cont 	led_cont;
extern enum UI_Modes ui_mode;

uint32_t LFOMON;
uint32_t TRIGMON;

float voltoct_pwm_tracking = 1.0f;

// Private function:
void update_envout_pwm(void);

TIM_HandleTypeDef	timAB;
TIM_HandleTypeDef	timC;
TIM_HandleTypeDef	timDEF;


void init_envout_pwm(void)
{

	uint8_t 			i;
	GPIO_InitTypeDef 	gpio;
	TIM_OC_InitTypeDef	tim_oc;

	//Initialize the values
	for (i=0;i<NUM_CHANNELS;i++)		lfos.envout_pwm[i]=0;


	ENVOUT_PWM_RCC_ENABLE();

	//
	// Setup GPIO for timer output pins
	//
	ENVOUT_PWM_TIM_GPIO_RCC_ENABLE();

	gpio.Mode 	= GPIO_MODE_AF_PP;
	gpio.Pull 	= GPIO_PULLUP;
	gpio.Speed 	= GPIO_SPEED_FREQ_HIGH;

	//Jacks A and B
	gpio.Alternate 	= ENVOUT_PWM_TIM_AB_AF;
	gpio.Pin 		= ENVOUT_PWM_pins_AB;
	HAL_GPIO_Init(ENVOUT_PWM_GPIO_AB, &gpio);

	//Jack C
	gpio.Alternate 	= ENVOUT_PWM_TIM_C_AF;
	gpio.Pin 		= ENVOUT_PWM_pins_C;
	HAL_GPIO_Init(ENVOUT_PWM_GPIO_C, &gpio);

	//Jacks D, E, and F
	gpio.Alternate 	= ENVOUT_PWM_TIM_DEF_AF;
	gpio.Pin 		= ENVOUT_PWM_pins_DEF;
	HAL_GPIO_Init(ENVOUT_PWM_GPIO_DEF, &gpio);

	// Initialize the Timer peripherals (period determines resolution and frequency)

	//This timer runs 2x as fast as the other two, because it's on the APB1 bus
	timAB.Instance 				 	= ENVOUT_PWM_TIM_AB;
	timAB.Init.Prescaler         	= 0;
	timAB.Init.Period            	= PWM_MAX; //216M / 1 / 256 = 420kHz;
	timAB.Init.ClockDivision     	= 0;
	timAB.Init.CounterMode       	= TIM_COUNTERMODE_UP;
	timAB.Init.RepetitionCounter 	= 0;
	timAB.Init.AutoReloadPreload 	= TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&timAB) != HAL_OK) _Error_Handler(__FILE__, __LINE__);


	timC.Instance 				 	= ENVOUT_PWM_TIM_C;
	timC.Init.Prescaler         	= 0;
	timC.Init.Period            	= PWM_MAX; //216M / 2 / 256 = 210kHz;
	timC.Init.ClockDivision     	= 0;
	timC.Init.CounterMode       	= TIM_COUNTERMODE_UP;
	timC.Init.RepetitionCounter 	= 0;
	timC.Init.AutoReloadPreload 	= TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&timC) != HAL_OK) _Error_Handler(__FILE__, __LINE__);

	timDEF.Instance 				= ENVOUT_PWM_TIM_DEF;
	timDEF.Init.Prescaler         	= 0;
	timDEF.Init.Period            	= PWM_MAX; //216M / 2 / 256 = 210kHz;
	timDEF.Init.ClockDivision     	= 0;
	timDEF.Init.CounterMode       	= TIM_COUNTERMODE_UP;
	timDEF.Init.RepetitionCounter 	= 0;
	timDEF.Init.AutoReloadPreload 	= TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&timDEF) != HAL_OK) _Error_Handler(__FILE__, __LINE__);


	// Configure each TIMx peripheral's Output Compare units.
	// Each channel (CCRx) needs to be enabled for each TIMx that we're using

	//Common configuration for all channels
	tim_oc.OCMode       = TIM_OCMODE_PWM1;
	tim_oc.OCPolarity   = TIM_OCPOLARITY_HIGH;
	tim_oc.OCFastMode   = TIM_OCFAST_DISABLE;
	tim_oc.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	tim_oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	tim_oc.OCIdleState  = TIM_OCIDLESTATE_RESET;
	tim_oc.Pulse 		= 0;

	if (HAL_TIM_PWM_ConfigChannel(&timAB, &tim_oc, 	ENVOUT_PWM_CHAN_A) != HAL_OK)		_Error_Handler(__FILE__, __LINE__);
	if (HAL_TIM_PWM_ConfigChannel(&timAB, &tim_oc, 	ENVOUT_PWM_CHAN_B) != HAL_OK)		_Error_Handler(__FILE__, __LINE__);
	if (HAL_TIM_PWM_ConfigChannel(&timC,  &tim_oc, 	ENVOUT_PWM_CHAN_C) != HAL_OK)		_Error_Handler(__FILE__, __LINE__);
	if (HAL_TIM_PWM_ConfigChannel(&timDEF, &tim_oc, ENVOUT_PWM_CHAN_D) != HAL_OK)		_Error_Handler(__FILE__, __LINE__);
	if (HAL_TIM_PWM_ConfigChannel(&timDEF, &tim_oc, ENVOUT_PWM_CHAN_E) != HAL_OK)		_Error_Handler(__FILE__, __LINE__);
	if (HAL_TIM_PWM_ConfigChannel(&timDEF, &tim_oc, ENVOUT_PWM_CHAN_F) != HAL_OK)		_Error_Handler(__FILE__, __LINE__);

	//
	// Start PWM signals generation
	//
 	if (HAL_TIM_PWM_Start(&timAB, 	ENVOUT_PWM_CHAN_A) != HAL_OK)						_Error_Handler(__FILE__, __LINE__);
 	if (HAL_TIM_PWM_Start(&timAB, 	ENVOUT_PWM_CHAN_B) != HAL_OK)						_Error_Handler(__FILE__, __LINE__);
 	if (HAL_TIM_PWM_Start(&timC, 	ENVOUT_PWM_CHAN_C) != HAL_OK)						_Error_Handler(__FILE__, __LINE__);
 	if (HAL_TIM_PWM_Start(&timDEF, 	ENVOUT_PWM_CHAN_D) != HAL_OK)						_Error_Handler(__FILE__, __LINE__);
 	if (HAL_TIM_PWM_Start(&timDEF, 	ENVOUT_PWM_CHAN_E) != HAL_OK)						_Error_Handler(__FILE__, __LINE__);
 	if (HAL_TIM_PWM_Start(&timDEF, 	ENVOUT_PWM_CHAN_F) != HAL_OK)						_Error_Handler(__FILE__, __LINE__);

}

	

void start_envout_pwm(void)
{
	start_timer_IRQ(PWM_OUTS_TIM_number, &update_envout_pwm);
}


void update_envout_pwm(void){

	/* Temporary: PWM_OUTS_TIM runs at 7.2 kHz (138 µs period) at priority
	 * 0,3 — higher than OSC_TIM — so anything here preempts the physics
	 * tick.  A rogue blocking call here would show on inner LED 3 as a
	 * per-call peak.  Remove once the 200 ms main-loop spike is
	 * diagnosed. */
	extern volatile uint32_t diag_pwm_out_peak_cycles;
	uint32_t pwm_out_start_cycles = DWT->CYCCNT;

	uint8_t j;
	uint32_t envout_buf;

	extern float lfo_phase_multiplier;
	update_lfos(lfo_phase_multiplier);

	/* Compute REAL elapsed PWM ticks since the last ISR call.
	 *
	 * NVIC_PRIORITYGROUP_2 puts SAI (0,0) and PWM_OUTS_TIM (0,3) in
	 * the same preemption level, so SAI cannot be preempted by us
	 * but it CAN delay us — and SAI runs ~250-300 µs every 500 µs,
	 * which is longer than the 138.9 µs PWM tick.  When SAI runs
	 * over two consecutive PWM tick boundaries the timer's update-
	 * interrupt flag stays asserted and the NVIC pending bit
	 * coalesces the two events into a single deferred ISR call;
	 * the second tick is silently lost.
	 *
	 * If we naively decrement lfos.lpg_trigger_delay[] by 1 per ISR
	 * call, those lost ticks make the strum schedule lag by 1-2
	 * ticks (~140-280 µs) at random points.  Whether that lag falls
	 * between voice-N firing and voice-(N+1) firing depends on
	 * where SAI happens to be in its block — bar-to-bar dependent —
	 * so the inter-voice spacing audibly speeds up and slows down
	 * even with a perfectly stable Pam's clock.
	 *
	 * Fix: read the free-running DWT cycle counter (216 MHz, immune
	 * to ISR scheduling), divide by the known PWM period of 30000
	 * cycles, and decrement by the *actual* number of ticks elapsed.
	 * Voices whose deadline is now in the past fire on this call;
	 * voices still ahead of the deadline get the precise wall-clock
	 * decrement.  Wall-clock spacing between voices then depends
	 * only on the schedule that was committed in read_freq() and
	 * the (rock-stable) hardware timer — not on which ISRs ran in
	 * between. */
	static uint32_t prev_pwm_cycles      = 0;
	static uint8_t  pwm_elapsed_inited   = 0;
	uint32_t now_cycles    = DWT->CYCCNT;
	uint32_t elapsed_ticks;
	if (!pwm_elapsed_inited) {
		/* First call: we have no prior reference.  Treat as a
		 * single normal tick so any boot-time pending delay (there
		 * shouldn't be one) progresses by 1 instead of by some wild
		 * value derived from CYCCNT == 0 at reset. */
		elapsed_ticks      = 1u;
		pwm_elapsed_inited = 1;
	} else {
		uint32_t elapsed_cycles = now_cycles - prev_pwm_cycles; /* uint32 wrap is fine */
		elapsed_ticks = elapsed_cycles / 30000u;        /* 216 MHz / 7.2 kHz */
		if (elapsed_ticks == 0u) elapsed_ticks = 1u;    /* never go backwards */
		if (elapsed_ticks > 65535u) elapsed_ticks = 65535u;
	}
	prev_pwm_cycles = now_cycles;

	for (j=0;j<NUM_CHANNELS;j++)
	{
		/* ENV OUT jacks are gate outputs for the drum station: full
		 * scale for DRUM_GATE_TICKS after that channel fires (set in
		 * drum_ui.c's fire(), called from OSC_TIM), rather than the
		 * old LFO/LPG waveform. */
		if (drum_gate_ticks[j] > 0) {
			drum_gate_ticks[j]--;
			envout_buf = PWM_MAX;
		} else {
			envout_buf = 0;
		}

		lfos.envout_pwm[j] = envout_buf;
		lfos.out_lpf[j]  = (float)(lfos.envout_pwm[j]) / (float)(PWM_MAX);
	}

	//ENVOUTs A,B are higher res (12 bits)
	ENVOUT_PWM_TIM_AB->ENVOUT_PWM_CC_A 	= lfos.envout_pwm[0];
	ENVOUT_PWM_TIM_AB->ENVOUT_PWM_CC_B 	= lfos.envout_pwm[1];
	ENVOUT_PWM_TIM_C->ENVOUT_PWM_CC_C 	= lfos.envout_pwm[2];
	ENVOUT_PWM_TIM_DEF->ENVOUT_PWM_CC_D = lfos.envout_pwm[3];
	ENVOUT_PWM_TIM_DEF->ENVOUT_PWM_CC_E = lfos.envout_pwm[4];
	ENVOUT_PWM_TIM_DEF->ENVOUT_PWM_CC_F = lfos.envout_pwm[5];

	/* ── Temporary: commit PWM_OUTS_TIM peak duration. ── */
	{
		uint32_t dur = DWT->CYCCNT - pwm_out_start_cycles;
		if (dur > diag_pwm_out_peak_cycles)
			diag_pwm_out_peak_cycles = dur;
	}
}
