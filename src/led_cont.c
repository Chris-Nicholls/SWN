/*
 * led_cont.c - handles SWN LEDs
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


#include "globals.h"
#include "led_cont.h"
#include "led_colors.h"
#include "envout_pwm.h"
#include "params_update.h"
#include "params_lfo.h"
#include "params_lfo_period.h"
#include "gpio_pins.h"
#include "flash_params.h"
#include "analog_conditioning.h"
#include "UI_conditioning.h"
#include "preset_manager.h"
#include "preset_manager_UI.h"
#include "math_util.h"
#include "drivers/mono_led_driver.h"
#include "system_settings.h"
#include "ui_modes.h"
#include "key_combos.h"
#include "drum_ui.h"
#include "drum_preset.h"

#include "drivers/leds_pwm.h"
#include "quantz_scales.h"
#include "calibrate_voct.h"
#include "flash_params.h"
#include "timekeeper.h"
#include "ui_modes.h"
#include "oscillator.h"
#include "hardware_controls.h"
#include "oscillator.h"
#include "lfo_wavetable_bank.h"

extern SystemCalibrations *system_calibrations;

// UI
extern enum UI_Modes 			ui_mode;

// LEDs
o_led_cont				 		led_cont;
extern const o_rgb_led			RGB_LED_OFF;

// Params
extern 		o_params 			params;
extern		o_calc_params		calc_params;
extern 		o_lfos				lfos;
extern		o_preset_manager	preset_mgr;
extern		o_systemSettings	system_settings;

// Hardware
extern 		o_monoLed   		monoLed[NUM_MONO_LED];
extern 		o_macro_states		macro_states;


// LED maps
extern const uint8_t	led_button_map[NUM_BUTTONS];
extern const uint8_t	led_rotary_map[NUM_LED_ROTARIES];
extern const uint8_t 	ledstring_map[NUM_CHANNELS+1];
extern const uint8_t 	led_outring_map[NUM_LED_OUTRING];
extern const uint8_t 	led_inring_map[NUM_LED_INRING];

const uint8_t OCT_OUTRING_MAP[NUM_LED_OUTRING] 		    	= { 9, 10, 11, 12, 13, 14, 15, 16, 17, 0, 1, 2, 3, 4, 5, 6, 7, 8 };

// Color palettes
extern uint32_t colorPalette[NUM_LED_COLORS][NUM_PALETTE_COLORS];
extern const enum colorCodes qtz_scale_colors[NUM_QTZ_SCALES];

// TABLES
extern const float 	exp_1voct_10_41V[4096];

const uint16_t CH_COLOR_MAP[6][3] = {
	{ 1		, 600	, 954	},
	{ 1		, 12	, 954	},
	{ 941  	, 366	, 954	},
	{ 941 	, 35 	, 947	},
	{ 954 	, 176	, 21 	},
	{ 800 	, 1	 	, 50	}
};

const uint16_t LFO_BANK_COLOR[25][3]= {
	{ 1		, 600	, 954	},				// Shades of Blue
	{ 1		, 318	, 947	},
	{ 1		, 94 	, 950	},
	{ 1		, 12	, 954	},
	{ 1		, 1		, 379	},

	{ 941  	, 366	, 954	},				// Shades of Pink
	{ 935 	, 116	, 928	},
	{ 941 	, 35 	, 947	},
	{ 954 	, 1		, 282 	},
	{ 904 	, 1	 	, 126	},

	{ 502	, 309 	, 43 	},				// Shades of Yellow/Orange
	{ 947 	, 388	, 21	},
	// { 949 	, 256	, 21 	},
	// { 954 	, 176	, 21 	},
	// { 954 	, 130	, 22	},
	// { 954 	, 55	, 21	},

	{ 100  	, 100	, 100	},				// Shade of white

	{  588	, 928	, 199	},				// Shades of Green
	{  274	, 954	, 67	},
	{  83	, 949	, 1		},
	{  1	, 239	, 1		},
	{  1	, 101	, 9		},
	{  1	, 25	, 4		},

	{ 941  	, 366	, 100	},				// Shades of Red
	{ 935 	, 116	, 80	},
	{ 941 	, 35 	, 60	},
	{ 954 	, 1		, 40 	},
	{ 904 	, 1	 	, 20	},
	{ 800 	, 1	 	, 0	    }
};

enum colorCodes key_sw_mode_colors[NUM_MUTE_NOTE_KEY_STATES];

void update_pwm_leds(void);

void update_pwm_leds(void)
{
	update_display_at_encoder_press();
	update_led_flash();
	update_button_leds();
	update_encoder_leds();
	update_array_leds();
	update_clockin_led();
	update_audioin_led();
	update_LED_rings();
}

void start_led_display(void)
{
	start_timer_IRQ(LED_UPDATE_TIM_number, &update_pwm_leds);
}

void init_led_cont_ongoing_display(void)
{
	led_cont.ongoing_display	= ONGOING_DISPLAY_NONE;
	led_cont.ongoing_timeout	= 0;
}

void init_led_cont(void)
{
	uint8_t i;

	// BUTTONS
	for (i=0; i<NUM_BUTTONS; i++)
	{
		if (i< NUM_CHANNELS)
			set_rgb_color_brightness(&led_cont.button[i], ledc_WHITE, 1.0/1.7);
		else
			set_rgb_color_brightness(&led_cont.button[i], ledc_PINK, 1.0/2.5);
	}

	// LFOs
	for (i=0; i<NUM_CHANNELS; i++)
		set_rgb_color_brightness(&led_cont.array[i], ledc_WHITE, 1.0/2.0);

	set_rgb_color_brightness(&led_cont.array[GLO_CLK], ledc_WHITE, 1.0/2.0);

	// Flags (LFO)
	for (i=0; i<NUM_CHANNELS; i++)
	{
		led_cont.ongoing_lfoshape[i] = 0;
		led_cont.lfoshape_timeout[i] = 0;
	}


	key_sw_mode_colors[ksw_MUTE] = ledc_WHITE;
	key_sw_mode_colors[ksw_NOTE] = ledc_PINK;
	key_sw_mode_colors[ksw_KEYS] = ledc_PURPLE;
}

void update_display_at_encoder_press(void)
{
	if (rotary_pressed(rotm_TRANSPOSE))
	{
		if (switch_pressed(FINE_BUTTON))
			start_ongoing_display_finetune();
		else
			start_ongoing_display_transpose();
	}

	if (rotary_pressed(rotm_OCT) && !switch_pressed(FINE_BUTTON) && (led_cont.ongoing_display != ONGOING_DISPLAY_SCALE))
		start_ongoing_display_octave();

	else if (rotary_pressed(rotm_OCT) && switch_pressed(FINE_BUTTON))
		start_ongoing_display_scale();

	static uint8_t cpu_toggle_handled = 0;
	if (rotary_pressed(rotm_LFOSPEED) == SHORT_PRESSED) {
		if (!cpu_toggle_handled) {
			cpu_toggle_handled = 1;
			if (led_cont.ongoing_display == ONGOING_DISPLAY_CPU_USAGE)
				led_cont.ongoing_display = ONGOING_DISPLAY_NONE;
			else
				start_ongoing_display_cpu_usage();
		}
	} else {
		cpu_toggle_handled = 0;
	}
}

void update_led_flash(void)
{
	led_cont.flash_state = (HAL_GetTick()/TICKS_PER_MS) & 0x080; //128ms flash period

	/* Age the per-channel hit flashes here rather than in the button
	 * renderer, so they still expire while an ongoing-display overlay
	 * owns the button LEDs. */
	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		if (drum_trig_flash[c])
			drum_trig_flash[c]--;
	}
}

void update_button_leds(void){

	uint8_t 					i;
	uint8_t 					color;
	enum VoctCalStates 			voct_state;

	int32_t 					tri_period, tri_phase;
	float 						tri_wave, brightness, lock_brightness;

	uint32_t					now = (HAL_GetTick()/TICKS_PER_MS);
	static uint32_t				animation_phase = 0;
	static enum ongoingDisplays	display_cache = ONGOING_DISPLAY_NONE;
	int16_t oct;

	for (i = 0; i < NUM_BUTTONS; i++){

		if (ui_mode == PLAY){

			if (i<NUM_CHANNELS){

				if(led_cont.ongoing_display){

					if (params.osc_param_lock[i] && lock_flash_state())
						lock_brightness = 0;
					else
						lock_brightness = F_MAX_BRIGHTNESS;

					if (params.key_sw[i]==ksw_MUTE) {
						if (!params.note_on[i])
							lock_brightness = F_MAX_BRIGHTNESS - lock_brightness;
					}

					if ( (led_cont.ongoing_display == ONGOING_DISPLAY_SCALE)){
						set_rgb_color_brightness(&led_cont.button[i], qtz_scale_colors[ params.indiv_scale[i] ], lock_brightness);
					}

					else if( (led_cont.ongoing_display == ONGOING_DISPLAY_TRANSPOSE) ){
						set_rgb_color_by_array(&led_cont.button[i], CH_COLOR_MAP[i], lock_brightness);
					}

					else if( (led_cont.ongoing_display == ONGOING_DISPLAY_SPHERE_SEL) ){
						led_cont.button[i].brightness = lock_brightness*4.0;
						get_wt_color(params.wt_bank[i], &led_cont.button[i]);
					}

					else if( (led_cont.ongoing_display == ONGOING_DISPLAY_FINETUNE) ){
						set_rgb_color_by_array(&led_cont.button[i], CH_COLOR_MAP[i], lock_brightness);
					}

					else if ( led_cont.ongoing_display == ONGOING_DISPLAY_OCTAVE){
						oct = _CLAMP_I16(params.oct[i], MIN_OCT , MAX_OCT) - MIN_OCT;
						oct = _CLAMP_I16(oct, 0, NUM_LED_OUTRING-1);
						led_cont.button[i].c_red  	= led_cont.outring[OCT_OUTRING_MAP[oct]].c_red;
						led_cont.button[i].c_green 	= led_cont.outring[OCT_OUTRING_MAP[oct]].c_green;
						led_cont.button[i].c_blue  	= led_cont.outring[OCT_OUTRING_MAP[oct]].c_blue;
						led_cont.button[i].brightness  	= F_MAX_BRIGHTNESS * lock_brightness;
					}

					else if ( led_cont.ongoing_display == ONGOING_DISPLAY_LFO_TOVCA){
						if (lfos.to_vca[i]) {
							if ( display_cache != ONGOING_DISPLAY_LFO_TOVCA ){
								animation_phase = 0xFFFFFFFF - now + 1; //starting phase: results in tri_phase starting at 0
								display_cache = ONGOING_DISPLAY_LFO_TOVCA;
							}
							tri_period = LFO_TOVCA_FLASH_PERIOD;
							tri_phase = (now + animation_phase) % (tri_period*2);
							tri_wave = _FOLD_F(tri_phase, tri_period);
							brightness = (float)(tri_wave/tri_period);
							set_rgb_color_brightness(&led_cont.button[i], ledc_LIGHT_GREEN, brightness);
						}
						else
							set_rgb_color(&led_cont.button[i], ledc_DIM_YELLOW);
					}

					else if ( led_cont.ongoing_display == ONGOING_DISPLAY_LFO_MODE){
						if (params.key_sw[i] == ksw_MUTE) {
							if (display_cache != ONGOING_DISPLAY_LFO_MODE){
								display_cache =  ONGOING_DISPLAY_LFO_MODE;
								animation_phase = 0xFFFFFFFF - now + 1; //starting phase: results in tri_phase starting at 0
							}

							tri_period = LFO_MODE_FLASH_PERIOD;
							tri_phase = (now + animation_phase + ((tri_period*2*(NUM_CHANNELS-i))/NUM_CHANNELS)) % (tri_period*2);

							if (lfos.mode[i]==lfot_LPG){
								// LPG mode: show envelope state - bright flash on trigger, decay
								brightness = lfos.out_lpf[i];
								color = ledc_FUSHIA;
							}
							else{ //lfot_LFO (standard LFO mode)
								tri_wave = _FOLD_F(tri_phase, tri_period);
								brightness = (float)(tri_wave/tri_period);
								color = ledc_BLUE;
							}
							if ((animation_phase + now) < 250) brightness = 0;

						} else {
							brightness = 0.2;
							color = (lfos.mode[i]==lfot_LPG) ? ledc_FUSHIA : ledc_BLUE;
						}
						set_rgb_color_brightness(&led_cont.button[i], color, brightness);
					}
					else if ( led_cont.ongoing_display == ONGOING_DISPLAY_SELBUS ) {
						//nothing
					}
				}

				else { //no ongoing_display

					/* Drum play: full brightness while the channel is
					 * flashing from a hit, a steady mid glow for the
					 * edit-focus channel, dim otherwise. */
					if (drum_trig_flash[i])
						brightness = F_MAX_BRIGHTNESS;
					else if (i == drum_selected_chan)
						brightness = 0.35f;
					else
						brightness = 0.08f;

					set_rgb_color_by_array(&led_cont.button[i], CH_COLOR_MAP[i], brightness);
				}
			}

			//LFO buttons:
			else
			{
				if (led_cont.ongoing_display == ONGOING_DISPLAY_SELBUS) {
					color = (system_settings.selbus_can_recall == SELBUS_RECALL_ENABLED) ? ledc_MED_GREEN : ledc_DIM_GREEN;
					set_rgb_color(&led_cont.button[butm_LFOVCA_BUTTON], color);
					color = (system_settings.selbus_can_save == SELBUS_SAVE_ENABLED) ? ledc_MED_RED : ledc_DIM_RED;
					set_rgb_color(&led_cont.button[butm_LFOMODE_BUTTON], color);
				}
				else 
				{
					if (!calc_params.keymode_pressed)
					{
						//No channels are in mute mode ==> dim purple
						if ((params.key_sw[0] != ksw_MUTE) &&
							(params.key_sw[1] != ksw_MUTE) &&
							(params.key_sw[2] != ksw_MUTE) &&
							(params.key_sw[3] != ksw_MUTE) &&
							(params.key_sw[4] != ksw_MUTE) &&
							(params.key_sw[5] != ksw_MUTE) )					set_rgb_color_brightness(&led_cont.button[i], ledc_PURPLE, 0.1);
						else if (!button_pressed(i))							set_rgb_color(&led_cont.button[i], ledc_PINK);
						else if (calc_params.button_safe_release[i - NUM_CHANNELS])	set_rgb_color(&led_cont.button[i], ledc_CORAL);
						else													set_rgb_color(&led_cont.button[i], ledc_MED_BLUE);
					}
					else
					{
						if(calc_params.button_safe_release[i - NUM_CHANNELS])		set_rgb_color(&led_cont.button[i], ledc_CORAL);
						else													set_rgb_color(&led_cont.button[i], ledc_PURPLE);
					}
				}
			}

		} //if (ui_mode==PLAY)

		else if (ui_mode==VOCT_CALIBRATE)
		{
			color = ledc_OFF;

			//Set the color of the channel buttons to the state of the Voct Calibration
			//(the Transpose calibration state is displayed on the led array)
			if (i<NUM_CHANNELS)
			{
				voct_state = get_voctcal_state(i);
				if (voct_state == VOCTCAL_READING_C1)		color = ledc_BLUE;
				if (voct_state == VOCTCAL_READING_C3)		color = ledc_RED;
				if (voct_state == VOCTCAL_CALIBRATED)		color = ledc_WHITE;
			}
			set_rgb_color(&led_cont.button[i], color);
		}

		set_pwm_led(led_button_map[i], &led_cont.button[i]);
	}
}

void update_encoder_leds(void){

	uint8_t i, color, set_color;

	if (led_cont.ongoing_display == ONGOING_DISPLAY_RECORD)
		color = ledc_FUSHIA;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_TRANSPOSE)
		color = ledc_FUSHIA;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_FINETUNE)
		color = ledc_MED_BLUE;

	else
		color = ledc_PURPLE;

	for (i = ledrotm_DEPTH; i < NUM_LED_ROTARIES; i++)
	{
		if (color == ledc_PURPLE && rotary_pressed(i))
			set_color = ledc_CORAL;
		else
			set_color = color;

		set_rgb_color(&led_cont.encoder[i], set_color);
		set_pwm_led(led_rotary_map[i], &led_cont.encoder[i]);
	}
}

void update_mono_leds(void){
	uint8_t i;
	static uint32_t slider_pwm=0;
	float exp;

	// SLIDERS

	if (ui_mode==RGB_COLOR_ADJUST)
	{
		mono_led_off(mledm_SLIDER_A);
		mono_led_off(mledm_SLIDER_B);
		mono_led_off(mledm_SLIDER_C);
		mono_led_off(mledm_SLIDER_D);
		mono_led_off(mledm_SLIDER_E);
		mono_led_off(mledm_SLIDER_F);
	}
	else {

		if (slider_pwm-- == 0)
		{
			slider_pwm=32;		//32 steps of brightness

			for (i=0;i<NUM_CHANNELS;i++){
				mono_led_off(i);
			}
		}
		else{
			for (i=0;i<NUM_CHANNELS;i++) {
				uint32_t level = (uint32_t)calc_params.level[i];

				if (calc_params.adjusting_pan_state[i] == pan_CACHED_LEVEL && cached_param_flash_state())
					level = level < 3000 ? 4095 : 0;

				exp = exp_1voct_10_41V[level] * system_settings.global_brightness;
				if ((level > 50) && (exp > (slider_pwm*43))){
					mono_led_on(i);
				}
			}
		}
	}
}

void update_array_leds(void)
{
	uint8_t 	i;
	uint8_t		color;
	enum VoctCalStates voct_state;

	if (ui_mode != VOCT_CALIBRATE)
	{
		calculate_lfo_leds();
		calculate_lfocv_led();

	}
	else if (ui_mode == VOCT_CALIBRATE)
	{
		color = ledc_OFF;

		//Get voct calibration state of Transpose jack
		voct_state = get_voctcal_state(NUM_VOCT_CHANNELS-1);
		if (voct_state == VOCTCAL_READING_C1)		color = ledc_BLUE;
		if (voct_state == VOCTCAL_READING_C3)		color = ledc_RED;
		if (voct_state == VOCTCAL_CALIBRATED)		color = ledc_WHITE;

		//Set all array LEDs to the same color
		for (i = 0; i < NUM_CHANNELS+1; i++)
			set_rgb_color(&led_cont.array[i], color);
	}


	for (i=0; i<NUM_CHANNELS +1; i++)
	{
		set_pwm_led(ledstring_map[i], &led_cont.array[i]);
	}
}

void update_clockin_led(void)
{
	if (jack_plugged(CLK_SENSE)) {
		if (led_cont.waiting_for_clockin)
			set_single_pwm_led(singleledm_CLKIN, 0);
		if (lfos.use_ext_clock && lfos.cycle_pos[REF_CLK] < 0.5)
			set_single_pwm_led(singleledm_CLKIN, 1000);
		else
			set_single_pwm_led(singleledm_CLKIN, 0);
	}
	else if (system_settings.allow_bus_clock) {
		if (BUS_CLK())	set_single_pwm_led(singleledm_CLKIN, 100);
		else			set_single_pwm_led(singleledm_CLKIN, 0);
	}
	else
		set_single_pwm_led(singleledm_CLKIN, 0);
}

void update_audioin_led(void)
{

	o_rgb_led rgb;

	set_rgb_color(&rgb, ledc_OFF);

	set_pwm_led(ledm_AUDIOIN, &rgb);

}

void calculate_lfocv_led(void)
{
	if ( 	(params.key_sw[0] == ksw_MUTE) ||
			(params.key_sw[1] == ksw_MUTE) ||
			(params.key_sw[2] == ksw_MUTE) ||
			(params.key_sw[3] == ksw_MUTE) ||
			(params.key_sw[4] == ksw_MUTE) ||
			(params.key_sw[5] == ksw_MUTE)	){

		if (led_cont.waiting_for_clockin){
			led_cont.array[GLO_CLK].c_red 		= 1023 * (1.0 - led_cont.clockin_wait_progress);
			led_cont.array[GLO_CLK].c_green 	= 1023 * led_cont.clockin_wait_progress;
			led_cont.array[GLO_CLK].c_blue 		= 200;
		}
		else if (lfos.period[GLO_CLK] > F_LFO_AUDIO_RANGE_PERIOD_L){
			set_rgb_color(&led_cont.array[GLO_CLK], ledc_WHITE);
		}
		else{
			set_rgb_color(&led_cont.array[GLO_CLK], ledc_PURPLE);
		}
	}
	else{
		set_rgb_color(&led_cont.array[GLO_CLK], ledc_CORAL);
	}

	if (lfos.cycle_pos[GLO_CLK] < 0.5)
		led_cont.array[GLO_CLK].brightness = F_MAX_BRIGHTNESS;
	else
		led_cont.array[GLO_CLK].brightness = 0;
}

void calculate_lfo_leds(void)
{
	uint8_t chan=0;
	float brightness;

	for (chan = 0; chan < NUM_CHANNELS; chan++)
	{
		// Audio rate and shape selection- -> LFO static brightness
		if  ( (params.key_sw[chan] == ksw_MUTE) && (lfos.audio_mode[chan] || led_cont.lfoshape_timeout[chan]) ){
			brightness 	= lfos.gain[chan];
		}
		else
			brightness = lfos.out_lpf[chan];

		set_rgb_color_by_array(&led_cont.array[chan], LFO_BANK_COLOR[lfos.shape[chan]], brightness);
	}
}

void update_LED_rings(void)
{
	uint8_t i;

	calculate_led_ring();

	for (i = 0; i < NUM_LED_OUTRING; i++){
		set_pwm_led(led_outring_map[i], &led_cont.outring[i]);
	}

	for (i = 0; i < NUM_LED_INRING; i++){
		set_pwm_led(led_inring_map[i], &led_cont.inring[i]);
	}
}


void calculate_led_ring(void){
	uint8_t i;

	update_ongoing_display_timers();

	if (ui_mode==VOCT_CALIBRATE) {
		turn_outring_off();

		for (i = 0; i < NUM_LED_INRING; i++){
			set_rgb_color(&led_cont.inring[i], ledc_OFF);
		}
	}
	else {
		switch (led_cont.ongoing_display)
		{
			case ONGOING_DISPLAY_TRANSPOSE:
				display_transpose();
				break;

			case ONGOING_DISPLAY_FINETUNE:
				display_finetune();
				break;

			case ONGOING_DISPLAY_OCTAVE:
				display_octave();
				break;

			case ONGOING_DISPLAY_PRESET:
				display_preset();
				break;

			case ONGOING_DISPLAY_CPU_USAGE:
				display_cpu_usage();
				break;

			case ONGOING_DISPLAY_DRUM_PARAM:
				display_drum_param();
				break;

			case ONGOING_DISPLAY_DRUM_PRESET:
				display_drum_preset();
				break;

			default:
				display_drum_pattern();
				break;
		}
	}
}

/* Outer ring = the selected channel's euclidean pattern, 1 LED per step:
 * ring position i is step i for i<n, and stays off for i>=n (e.g. an
 * 8-step pattern lights only 8 of the 18 LEDs -- no stretching to fill
 * the ring).  Active steps lit dim, playhead bright.  Inner ring = one
 * LED per channel, lit while that channel is flashing from a hit. */
void display_drum_pattern(void)
{
	const EuclidChannelState *e = &drum_chan[drum_selected_chan].euclid;
	uint8_t i;

	for (i = 0; i < NUM_LED_OUTRING; i++) {
		uint8_t j = rotate_origin(i, NUM_LED_OUTRING);
		float bri = 0.0f;
		enum ledColors color = ledc_AQUA;

		if (i < e->n) {
			if (i == e->current_step) {
				bri = F_MAX_BRIGHTNESS;
				color = ledc_WHITE;
			} else if (euclid_step_active(e, i)) {
				bri = F_MAX_BRIGHTNESS * 0.25f;
			} else {
				bri = F_MAX_BRIGHTNESS * 0.02f;
			}
		}

		set_rgb_color_brightness(&led_cont.outring[j], color, bri);
	}

	for (i = 0; i < NUM_LED_INRING; i++) {
		uint8_t j = rotate_origin(i, NUM_LED_INRING);
		float bri = 0.0f;
		if (i < NUM_CHANNELS)
			bri = drum_trig_flash[i] ? F_MAX_BRIGHTNESS : 0.05f;
		set_rgb_color_brightness(&led_cont.inring[j],
		                         (i == drum_selected_chan) ? ledc_WHITE : ledc_AQUA,
		                         bri);
	}
}

/* Outer ring bar-graph for whichever of filter/decay/other is being
 * turned: floor(value*18) LEDs solid, the next one partially lit for
 * sub-step resolution, colour-coded per param so Depth/Latitude/
 * Longitude are visually distinct at a glance. */
void display_drum_param(void)
{
	const o_drum_chan *d = &drum_chan[drum_selected_chan];
	float value;
	enum ledColors color;
	uint8_t i;

	switch (led_cont.ongoing_drum_param) {
		case DRUM_PARAM_DISP_DECAY: value = d->decay; color = ledc_YELLOW; break;
		case DRUM_PARAM_DISP_OTHER: value = d->other; color = ledc_PURPLE; break;
		case DRUM_PARAM_DISP_SPEED:
			/* clock_divmult_id ranges LFO_MIN_DIVMULT_ID..LFO_MAX_DIVMULT_ID,
			 * not 0..1 like the other three -- normalize it so the same
			 * bar-graph code works unchanged. Unity (LFO_UNITY_DIVMULT_ID)
			 * lands well left of center since the multiply range above it
			 * is much wider than the divide range below it. */
			value = (d->clock_divmult_id - LFO_MIN_DIVMULT_ID) / (float)(LFO_MAX_DIVMULT_ID - LFO_MIN_DIVMULT_ID);
			color = ledc_MED_GREEN;
			break;
		default:                    value = d->filter; color = ledc_AQUA;  break;
	}

	float lit_f = _CLAMP_F(value, 0.0f, 1.0f) * NUM_LED_OUTRING;
	uint8_t lit_full = (uint8_t)lit_f;
	float   lit_frac = lit_f - (float)lit_full;

	for (i = 0; i < NUM_LED_OUTRING; i++) {
		uint8_t j = rotate_origin(i, NUM_LED_OUTRING);
		float bri;

		if (i < lit_full)
			bri = F_MAX_BRIGHTNESS;
		else if (i == lit_full)
			bri = F_MAX_BRIGHTNESS * lit_frac;
		else
			bri = 0.0f;

		set_rgb_color_brightness(&led_cont.outring[j], color, bri);
	}

	for (i = 0; i < NUM_LED_INRING; i++)
		set_rgb_color_brightness(&led_cont.inring[rotate_origin(i, NUM_LED_INRING)], color, 0.0f);
}

void start_ongoing_display_drum_param(enum drumParamDisplay which)
{
	led_cont.ongoing_display     = ONGOING_DISPLAY_DRUM_PARAM;
	led_cont.ongoing_drum_param  = which;
	led_cont.ongoing_timeout     = DRUM_PARAM_DISPLAY_TIMER_LIMIT;
}

/* Outer ring = preset slots, one LED per slot (DRUM_PRESET_NUM_SLOTS=16
 * fits directly onto the 18-LED ring with 2 spare, unlit positions):
 * filled slots dim yellow, the selected slot bright -- white while just
 * browsing, green right after a load, red right after a save. */
void display_drum_preset(void)
{
	uint8_t selected = drum_preset_selected_slot();
	uint8_t i;

	enum ledColors hover_color;
	switch (led_cont.ongoing_drum_preset_activity) {
		case DRUM_PRESET_DISP_LOADED: hover_color = ledc_MED_GREEN; break;
		case DRUM_PRESET_DISP_SAVED:  hover_color = ledc_RED;       break;
		default:                      hover_color = ledc_WHITE;     break;
	}

	for (i = 0; i < NUM_LED_OUTRING; i++) {
		uint8_t j = rotate_origin(i, NUM_LED_OUTRING);
		float bri = 0.0f;
		enum ledColors color = ledc_YELLOW;

		if (i == selected) {
			bri = F_MAX_BRIGHTNESS;
			color = hover_color;
		} else if (i < DRUM_PRESET_NUM_SLOTS && drum_preset_slot_filled(i)) {
			bri = F_MAX_BRIGHTNESS * 0.2f;
		}

		set_rgb_color_brightness(&led_cont.outring[j], color, bri);
	}

	for (i = 0; i < NUM_LED_INRING; i++)
		set_rgb_color_brightness(&led_cont.inring[rotate_origin(i, NUM_LED_INRING)], hover_color, 0.0f);
}

void start_ongoing_display_drum_preset(enum drumPresetDisplay activity)
{
	led_cont.ongoing_display              = ONGOING_DISPLAY_DRUM_PRESET;
	led_cont.ongoing_drum_preset_activity = activity;
	led_cont.ongoing_timeout              = DRUM_PRESET_DISPLAY_TIMER_LIMIT;
}

void turn_outring_off(void)
{
	uint8_t i;

	for (i =0; i< NUM_LED_OUTRING; i++){
		led_cont.outring[i].brightness 	= 0;
	}
}

void display_wtpos_inring(void)
{
	uint8_t 	i, j;
	uint16_t 	scaled_wt_pos[3];

	for ( i = 0 ; i < NUM_CHANNELS ; i++)
	{
		scaled_wt_pos[0] = _SCALE_F2U16(calc_params.wt_pos[0][i], 0, 2, 2048, 4095);
		scaled_wt_pos[1] = _SCALE_F2U16(calc_params.wt_pos[1][i], 0, 2, 2048, 3900);
		scaled_wt_pos[2] = _SCALE_F2U16(calc_params.wt_pos[2][i], 0, 2, 2048, 4095);

		j = rotate_origin(i, NUM_CHANNELS);
		led_cont.inring[j].c_red 		= 3 * exp_1voct_10_41V[scaled_wt_pos[0]];
		led_cont.inring[j].c_green 		= 	  exp_1voct_10_41V[scaled_wt_pos[1]];
		led_cont.inring[j].c_blue 		= 3 * exp_1voct_10_41V[scaled_wt_pos[2]];
		led_cont.inring[j].brightness 	= F_MAX_BRIGHTNESS;
	}
}

void flash_wt_lock(void)
{
	uint8_t chan, led;

	for ( chan = 0; chan < NUM_CHANNELS; chan++)
	{
		led = rotate_origin(chan, NUM_CHANNELS);
		if (params.osc_param_lock[chan] && lock_flash_state())
		{
			led_cont.outring[led * 3    ].brightness = 0;
			led_cont.outring[led * 3 + 1].brightness = 0;
			led_cont.outring[led * 3 + 2].brightness = 0;
			led_cont.inring[led].brightness = 0;
		}
	}
}

void get_wt_color(uint8_t wt_num, o_rgb_led *rgb)
{
	float fade, inv_fade;
	uint16_t scaled_wt_num;

	if (wt_num < NUM_FACTORY_SPHERES) {
		scaled_wt_num = _SCALE_U2U(wt_num, 0, NUM_FACTORY_SPHERES, 1024, 4095);
		fade = exp_1voct_10_41V[scaled_wt_num] / 1370.0;
		inv_fade = exp_1voct_10_41V[4095-scaled_wt_num] / 1370.0;

		rgb->c_red 		= (2048.0 * inv_fade) + 150.0;
		rgb->c_green  	= 0;
		rgb->c_blue 	= (200.0 * fade);
	}
	else if (wt_num < MAX_TOTAL_SPHERES) {
		scaled_wt_num = _SCALE_U2U((wt_num-NUM_FACTORY_SPHERES) % 18, 0, 17, 1024, 4095);
		//fade = exp_1voct_10_41V[scaled_wt_num] / 1370.0;
		inv_fade = exp_1voct_10_41V[4095-scaled_wt_num+1024] / 1370.0;
		fade = 1.0-inv_fade;

		if (wt_num < (NUM_FACTORY_SPHERES + 18*1)){
			rgb->c_red  	= (300.0 * fade)+0; //fade=.726 ->463
			rgb->c_green 	= (2048.0 * inv_fade) + 50; //inv fade = 0.0058 ->62
			rgb->c_blue 	= 0;
		}
		else if (wt_num < (NUM_FACTORY_SPHERES + 18*2)){
			rgb->c_red  	= 0;
			rgb->c_green 	= (100.0 * fade)+50;
			rgb->c_blue 	= (2048.0 * inv_fade) + 50;
		}
		else if (wt_num < (NUM_FACTORY_SPHERES + 18*3)){
			rgb->c_red 		= (2048.0 * inv_fade) + 150;
			rgb->c_green 	= (50.0 * fade)+0;
			rgb->c_blue 	= (2048.0 * inv_fade) + 50;
		}
		else if (wt_num < (NUM_FACTORY_SPHERES + 18*4)){
			rgb->c_red 		= (200.0 * fade);
			rgb->c_green 	= (2048.0 * inv_fade) + 50;
			rgb->c_blue 	= (2048.0 * inv_fade) + 50;
		}
		else if (wt_num < (NUM_FACTORY_SPHERES + 18*5)){
			rgb->c_red 		= (2800.0 * inv_fade) + 50;
			rgb->c_green 	= (1600.0 * inv_fade) + 50;
			rgb->c_blue 	= (100.0 * fade);
		}
		else{
			rgb->c_red 		= (2048.0 * inv_fade) + 50;
			rgb->c_green 	= (2048.0 * inv_fade) + 50;
			rgb->c_blue 	= (2048.0 * inv_fade) + 50;
		}
	}
	else {
		set_rgb_color(rgb, ledc_OFF);
	}
}

void display_wt_pos(void)
{
	uint8_t i, j;
	int8_t chan =-1;
	float folded_wt_pos;
	uint16_t scaled_wt_pos[3];

	for ( i = 0 ; i < NUM_LED_OUTRING ; i++)
	{
		if (!i || !(i%3))
		{
			chan++;
			folded_wt_pos 	 =_FOLD_F(calc_params.wt_pos[0][chan], 1.5);
			scaled_wt_pos[0] = _SCALE_F2U16(folded_wt_pos, 0, 1.5, 2048, 4095);

			folded_wt_pos 	 =_FOLD_F(calc_params.wt_pos[1][chan], 1.5);
			scaled_wt_pos[1] = _SCALE_F2U16(folded_wt_pos, 0, 1.5, 2048, 3900);

			folded_wt_pos 	 =_FOLD_F(calc_params.wt_pos[2][chan], 1.5);
			scaled_wt_pos[2] = _SCALE_F2U16(folded_wt_pos, 0, 1.5, 2048, 4095);
		}

		j = rotate_origin(i, NUM_LED_OUTRING);
		led_cont.outring[j].c_red 		= 3 * exp_1voct_10_41V[scaled_wt_pos[0]];
		led_cont.outring[j].c_green 	= 	  exp_1voct_10_41V[scaled_wt_pos[1]];
		led_cont.outring[j].c_blue 		= 3 * exp_1voct_10_41V[scaled_wt_pos[2]];
		led_cont.outring[j].brightness 	= F_MAX_BRIGHTNESS;
	}

	for ( i = 0; i < NUM_CHANNELS; i++)
	{
		j = rotate_origin(i, NUM_CHANNELS);
		led_cont.inring[j].brightness = F_MAX_BRIGHTNESS;
		get_wt_color(params.wt_bank[i], &led_cont.inring[j]);
	}
}

void display_wt_seed_pos(void)
{
	uint8_t i, j;

	/* Start with all outring LEDs dark, then accumulate per-channel
	 * split cursors.  Saturate at 4095 to avoid wrap on overlap. */
	for (i = 0; i < NUM_LED_OUTRING; i++) {
		led_cont.outring[i].c_red   = 0;
		led_cont.outring[i].c_green = 0;
		led_cont.outring[i].c_blue  = 0;
		led_cont.outring[i].brightness = F_MAX_BRIGHTNESS;
	}

	for (i = 0; i < NUM_CHANNELS; i++) {
		/* Read the live glided position (updated every OSC_TIM tick
		 * by the morph bed) rather than wt_osc.pending_seed_pos[]
		 * (only refreshed once per ~16-tick dispatcher pass) — gives
		 * continuous visual tracking as the encoder turns. */
		float pos = params.wt_browse_step_pos_enc[i];
		/* Wrap into [0, NUM_WAVEFORMS_IN_SPHERE) defensively.  The
		 * PLAY-mode browse handler wraps on encoder turn, but a CV
		 * offset or a mode transition could briefly push pos outside
		 * the range. */
		while (pos < 0.0f)
			pos += (float)NUM_WAVEFORMS_IN_SPHERE;
		while (pos >= (float)NUM_WAVEFORMS_IN_SPHERE)
			pos -= (float)NUM_WAVEFORMS_IN_SPHERE;

		/* Map 0..NUM_WAVEFORMS_IN_SPHERE → 0..NUM_LED_OUTRING with a
		 * float index.  Using the un-decremented counts on both sides
		 * (27 and 18) means 1.5 wavetable positions per LED and
		 * wrap closes cleanly: pos=27 ≡ pos=0 → LED 18 ≡ LED 0. */
		float led_f = pos * (float)NUM_LED_OUTRING
		              / (float)NUM_WAVEFORMS_IN_SPHERE;
		if (led_f < 0.0f) led_f = 0.0f;
		if (led_f >= (float)NUM_LED_OUTRING)
			led_f -= (float)NUM_LED_OUTRING;

		uint8_t led_a = (uint8_t)led_f;
		if (led_a >= NUM_LED_OUTRING) led_a = 0;
		uint8_t led_b = (uint8_t)(led_a + 1);
		if (led_b >= NUM_LED_OUTRING) led_b = 0;
		float fled = led_f - (float)led_a;

		o_rgb_led ch_color;
		get_wt_color(params.wt_bank[i], &ch_color);

		/* Distribute the cursor colour between the two adjacent LEDs
		 * in proportion to (1-fled) and fled.  At integer positions
		 * (fled=0) led_a gets the full colour and led_b gets nothing,
		 * matching the old discrete-cursor look.  Mid-glide the
		 * cursor visibly slides between LEDs. */
		float wa = 1.0f - fled;
		float wb = fled;

		uint8_t ja = rotate_origin(led_a, NUM_LED_OUTRING);
		uint8_t jb = rotate_origin(led_b, NUM_LED_OUTRING);

		uint32_t ra = (uint32_t)led_cont.outring[ja].c_red
		              + (uint32_t)((float)ch_color.c_red   * wa);
		uint32_t ga = (uint32_t)led_cont.outring[ja].c_green
		              + (uint32_t)((float)ch_color.c_green * wa);
		uint32_t ba = (uint32_t)led_cont.outring[ja].c_blue
		              + (uint32_t)((float)ch_color.c_blue  * wa);
		if (ra > 4095) ra = 4095;
		if (ga > 4095) ga = 4095;
		if (ba > 4095) ba = 4095;
		led_cont.outring[ja].c_red   = (uint16_t)ra;
		led_cont.outring[ja].c_green = (uint16_t)ga;
		led_cont.outring[ja].c_blue  = (uint16_t)ba;

		uint32_t rb = (uint32_t)led_cont.outring[jb].c_red
		              + (uint32_t)((float)ch_color.c_red   * wb);
		uint32_t gb = (uint32_t)led_cont.outring[jb].c_green
		              + (uint32_t)((float)ch_color.c_green * wb);
		uint32_t bb = (uint32_t)led_cont.outring[jb].c_blue
		              + (uint32_t)((float)ch_color.c_blue  * wb);
		if (rb > 4095) rb = 4095;
		if (gb > 4095) gb = 4095;
		if (bb > 4095) bb = 4095;
		led_cont.outring[jb].c_red   = (uint16_t)rb;
		led_cont.outring[jb].c_green = (uint16_t)gb;
		led_cont.outring[jb].c_blue  = (uint16_t)bb;
	}

	/* Inner ring: per-channel sphere color so each channel's bank is
	 * legible at a glance independent of seed-position cursors. */
	for (i = 0; i < NUM_CHANNELS; i++) {
		j = rotate_origin(i, NUM_CHANNELS);
		led_cont.inring[j].brightness = F_MAX_BRIGHTNESS;
		get_wt_color(params.wt_bank[i], &led_cont.inring[j]);
	}
}

/* Single-parameter bar-graph helper used by the RS param overlays.
 *   value: 0..1 fill ratio for the outring bar (channel 0 representative).
 *   per_channel: NUM_CHANNELS-long array of per-channel values for the
 *     inring tint (captures CV-offset divergence across channels).
 *   r_w/g_w/b_w: per-channel weights (0..255) for the bar color.
 */
void display_firmware_version(void)
{
	uint8_t i, j;

	set_rgb_color(&led_cont.encoder[ledrotm_DEPTH], ledc_OFF);
	set_rgb_color(&led_cont.encoder[ledrotm_LATITUDE], ledc_OFF);
	set_rgb_color(&led_cont.encoder[ledrotm_LONGITUDE], ledc_OFF);

	set_pwm_led(led_rotary_map[ledrotm_DEPTH], &led_cont.encoder[ledrotm_DEPTH]);
	set_pwm_led(led_rotary_map[ledrotm_LATITUDE], &led_cont.encoder[ledrotm_LATITUDE]);
	set_pwm_led(led_rotary_map[ledrotm_LONGITUDE], &led_cont.encoder[ledrotm_LONGITUDE]);

	for (i=0; i<NUM_BUTTONS; i++) {
		set_rgb_color(&led_cont.button[i], ledc_OFF);
		set_pwm_led(led_button_map[i], &led_cont.button[i]);
	}

	for (i=0; i<NUM_LED_ARRAY; i++) {
		set_rgb_color(&led_cont.array[i], ledc_OFF);
		set_pwm_led(ledstring_map[i], &led_cont.array[i]);
	}

	for (i =0; i< NUM_LED_OUTRING; i++)
		set_rgb_color(&led_cont.outring[i], ledc_OFF);

	for (i=0; i<NUM_CHANNELS; i++)
		set_rgb_color(&led_cont.inring[i], ledc_OFF);

	if (system_calibrations->major_firmware_version > 0) {
		j = system_calibrations->major_firmware_version - 1;
		set_rgb_color(&led_cont.inring[j], ledc_RED);
	}

	j = system_calibrations->minor_firmware_version;
	// j = (i>=9) ? (i-9) : (i+9); //set bottom-left as origin
	set_rgb_color(&led_cont.outring[j], ledc_BLUE);

	for (i =0; i< NUM_LED_OUTRING; i++)
		set_pwm_led(led_outring_map[i], &led_cont.outring[i]);

	for (i=0; i<NUM_CHANNELS; i++)
		set_pwm_led(led_inring_map[i], &led_cont.inring[i]);

}

void display_transpose(void)
{
	uint8_t i, j, chan;
	int32_t t_transpose;
	int8_t transpose_pos[NUM_CHANNELS];
	uint32_t num_wraps;

	uint8_t overlap[NUM_LED_OUTRING][NUM_CHANNELS];
	uint8_t overlap_num[NUM_LED_OUTRING];
	static uint8_t overlap_ctr[NUM_LED_OUTRING]={0};

	uint8_t do_advance_overlap = 0;
	static uint32_t last_advance_overlap_tmr=0;
	uint32_t now = HAL_GetTick()/TICKS_PER_MS;

	if ((now - last_advance_overlap_tmr) > 300) {
		do_advance_overlap = 1;
		last_advance_overlap_tmr = now;
	}

	for (i = 0; i < NUM_LED_OUTRING; i++)
	{
		led_cont.outring[i].brightness = 0;
		overlap_num[i] = 0;
		for (chan=0; chan<NUM_CHANNELS; chan++)
			overlap[i][chan] = 99;
	}

	// Create overlap[led_position][channels_occupying_position] = channel#
	for (i = 0; i < NUM_CHANNELS; i++)
	{
		t_transpose = _CLAMP_I16(calc_params.transpose[i], MIN_TRANSPOSE_WRAP, MAX_TRANSPOSE_WRAP);
		transpose_pos[i] = (t_transpose - MIN_TRANSPOSE_WRAP)  % NUM_LED_OUTRING;

		j = rotate_origin(i, NUM_CHANNELS);

		if (params.osc_param_lock[i] && lock_flash_state() ){
			set_rgb_color(&led_cont.inring[j], ledc_OFF);
		} else {
			num_wraps = (uint32_t)((float)(t_transpose - transpose_pos[i] - MIN_TRANSPOSE_WRAP) / (float)(MAX_TRANSPOSE_WRAP - MIN_TRANSPOSE_WRAP) * 4096.0);
			set_rgb_color_by_array(&led_cont.inring[j], CH_COLOR_MAP[i], ((exp_1voct_10_41V[num_wraps] / 500.0) + 0.017) / F_MAX_BRIGHTNESS);
		}

		overlap[transpose_pos[i]][ overlap_num[transpose_pos[i]] ] = i;
		overlap_num[transpose_pos[i]]++;
	}

	for (i = 0; i<NUM_LED_OUTRING; i++)
	{
		if (do_advance_overlap) {
			overlap_ctr[i]++;
			if (overlap_ctr[i] >= overlap_num[i]) overlap_ctr[i]=0;
		}

		chan = overlap[i][overlap_ctr[i]];
		if (chan<=NUM_CHANNELS)
		{
			if (overlap_num[i]==1 && params.osc_param_lock[chan] && lock_flash_state() )
				set_rgb_color(&led_cont.outring[i], ledc_OFF);
			else
				set_rgb_color_by_array(&led_cont.outring[i], CH_COLOR_MAP[chan], 1.0);
		}

	}
}


void display_finetune (void)
{
	uint8_t i, j;
	int16_t t_finetune;
	uint32_t triangle, saw;
	uint32_t tm;
	uint32_t period;
	uint8_t	detune_pos_i;
	float brightness;

	tm = HAL_GetTick()/TICKS_PER_MS;

	for (i = 0; i < NUM_LED_OUTRING; i++)
		set_rgb_color(&led_cont.outring[i], ledc_OFF);

	for (i = 0; i < NUM_CHANNELS; i++)
	{
		//Inner ring: shows channel color (solid, full brightness = unlocked; flashing=locked)
		if (params.osc_param_lock[i] && lock_flash_state())
			brightness 	= 0.0;
		else
			brightness 	= F_MAX_BRIGHTNESS;

		j = rotate_origin(i, NUM_CHANNELS);
		set_rgb_color_by_array(&led_cont.inring[j], CH_COLOR_MAP[i], brightness);

		t_finetune = _CLAMP_I16(params.finetune[i], MIN_FINETUNE_WRAP, MAX_FINETUNE_WRAP);

		if (t_finetune==0) {
			detune_pos_i = j*3+1;
			set_rgb_color(&led_cont.outring[detune_pos_i], ledc_WHITE);
			// led_cont.outring[detune_pos_i].brightness = F_MAX_BRIGHTNESS*0.25; //dim tuned channels?
		}
		else {
			if (t_finetune<0) {
				period = _CLAMP_I16(1000+(t_finetune*0.8f), 50, 1000);
				detune_pos_i = j*3;
				set_rgb_color(&led_cont.outring[detune_pos_i], ledc_BLUE);
			}
			else {
				period = _CLAMP_I16(1000-(t_finetune*0.8f), 50, 1000);
				detune_pos_i = j*3+2;
				set_rgb_color(&led_cont.outring[detune_pos_i], ledc_RED);
			}

			//Calculate triangle wave to make LED fade faster as it gets more detuned
			saw = ((float)(tm % period)/(float)period) * 8191.0;
			triangle = (saw>4095) ? (8191-saw) : saw;

			led_cont.outring[detune_pos_i].brightness = (exp_1voct_10_41V[triangle] / 1367.0) + 0.03;
		}
	}
}

// Phase spread multipliers for display (matches oscillator.c)


void display_unison(void)
{
	uint8_t i, j;

	uint8_t num_voices_lit;
	uint8_t voices;
	float spread;

	// Turn off all rings first
	turn_outring_off();
	for (i = 0; i < NUM_LED_INRING; i++) set_rgb_color(&led_cont.inring[i], ledc_OFF);

	// Determine which channel to display
	// If Global (no buttons pressed), display Channel 0 (or blend? usually we just pick one or show all)
	// If Individual, display the pressed channel
	int8_t active_chan = -1;
	
	if (macro_states.all_af_buttons_released) {
		active_chan = 0; // Display channel 0 parameters as proxy for global, or iterate?
						 // Let's display the max values found across channels, or just chan 0
	} else {
		for (i = 0; i < NUM_CHANNELS; i++) {
			if (button_pressed(i)) {
				active_chan = i;
				break;
			}
		}
	}
	if (active_chan == -1) active_chan = 0;

	// INNER RING: Voice Count (1-8)
	// 6 LEDs available.
	voices = params.unison_voice_count[active_chan];
	num_voices_lit = (voices > NUM_LED_INRING) ? NUM_LED_INRING : voices;

	for (i = 0; i < num_voices_lit; i++) {
		// Fill up
		// Change color for 7th and 8th voice equivalent
		enum colorCodes c = ledc_GOLD;
		if (i == (NUM_LED_INRING-1) && voices > NUM_LED_INRING) {
			if (voices == 7) c = ledc_CORAL;
			if (voices == 8) c = ledc_RED;
		}
		
		j = rotate_origin(i, NUM_LED_INRING);
		set_rgb_color(&led_cont.inring[j], c);
		
		// If editing global, pulse slightly?
		led_cont.inring[j].brightness = F_MAX_BRIGHTNESS;
	}

	// OUTER RING: Spread Amount (0.0 - 1.0)
	// 18 LEDs. Match to 0.0 - 1.0 range.
	spread = params.unison_spread_amt[active_chan];
	uint8_t spread_leds = (uint8_t)(spread * NUM_LED_OUTRING);
	if (spread > 0.0f && spread_leds == 0) spread_leds = 1; // Show at least one if > 0

	for (i = 0; i < spread_leds; i++) {
		j = rotate_origin(i, NUM_LED_OUTRING);
		set_rgb_color(&led_cont.outring[j], ledc_CORAL);
		led_cont.outring[j].brightness = 0.5f;
	}
}

void display_soft_clip(void) {
	uint32_t i;
	// Turn off all rings first
	turn_outring_off();
	// for (i = 0; i < NUM_LED_INRING; i++) set_rgb_color(&led_cont.inring[i], ledc_OFF); // Keep inner ring? or turn off? Logic says turn off

	// Display soft clip amount on Outer Ring
	// Range: 0.1 to 4.0. Default 1.0.
	// Map 0.1..4.0 to 0..NUM_LED_OUTRING LEDs
	float val = params.soft_clip_pregain;
	float norm_val = (val - MIN_SOFT_CLIP_PREGAIN) / (MAX_SOFT_CLIP_PREGAIN - MIN_SOFT_CLIP_PREGAIN);
	uint8_t num_leds = (uint8_t)(norm_val * NUM_LED_OUTRING);
	if (num_leds == 0) num_leds = 1;

	// Color gradient: Green (low gain) -> Orange (unity) -> Red (high gain/clip)
	enum colorCodes c;
	if (val < 0.9f) c = ledc_GREEN;
	else if (val < 1.2f) c = ledc_CORAL; // Around unity
	else c = ledc_RED; 

	for (i=0; i<num_leds; i++) {
        uint8_t led_idx = (i + 14) % NUM_LED_OUTRING; 
		set_rgb_color(&led_cont.outring[led_idx], c); // fixed index variable
	}
}



void display_preset(void)
{
	uint8_t slot_i, bank_i, led;

	uint16_t hover_bank, preset_i;
	uint8_t slot_color = ledc_OFF;

	hover_bank = preset_mgr.hover_num / NUM_LED_OUTRING;

	for (slot_i = 0; slot_i < NUM_LED_OUTRING; slot_i++)
	{
		led = rotate_origin(slot_i, NUM_LED_OUTRING);
		preset_i = slot_i + (hover_bank*NUM_LED_OUTRING);
		slot_color = animate_preset_ledring(slot_i, preset_i);

		set_rgb_color(&led_cont.outring[led], slot_color);
	}

	for ( bank_i = 0; bank_i < NUM_CHANNELS; bank_i++)
	{
		led = rotate_origin(bank_i, NUM_CHANNELS);
		if (bank_i==hover_bank)	slot_color = ledc_WHITE;
		else					slot_color = ledc_OFF;

		set_rgb_color(&led_cont.inring[led], slot_color);
	}
}



void display_octave(void)
{
	uint8_t i,j;
	int16_t oct;

	for (i = 0; i < NUM_LED_OUTRING; i++) {
		led_cont.outring[i].brightness 	= 0;
		led_cont.outring[i].c_red  		= 4032  - 4032 * (OCT_OUTRING_MAP[i])  / 18;
		led_cont.outring[i].c_green 	= 3800 * (OCT_OUTRING_MAP[i]) / 18;
		led_cont.outring[i].c_blue  	= 0;
	}

	// Light up LED ring positions corresponding to current indiv oct
	for (i=0; i<NUM_CHANNELS; i++){
		oct = _CLAMP_I16(params.oct[i], MIN_OCT , MAX_OCT) - MIN_OCT;
		oct = _CLAMP_I16(oct, 0, NUM_LED_OUTRING-1);

		j = rotate_origin(i, NUM_CHANNELS);
		set_rgb_color_by_rgb(&led_cont.inring[j], &led_cont.outring[OCT_OUTRING_MAP[oct]]);

		// flash locked channels
		if (led_cont.flash_state && params.osc_param_lock[i]){
			led_cont.outring[OCT_OUTRING_MAP[oct]].brightness = 0;
			led_cont.inring[j].brightness = 0;
		}
		else
		{
			if(macro_states.all_af_buttons_released) {
				led_cont.outring[OCT_OUTRING_MAP[oct]].brightness = F_MAX_BRIGHTNESS;
				led_cont.inring[j].brightness = F_MAX_BRIGHTNESS;
			}
			else
			{
				if (button_pressed(i) || (led_cont.ongoing_display == ONGOING_DISPLAY_FINETUNE)){ //Todo: What is FINETUNE doing here?
					led_cont.outring[OCT_OUTRING_MAP[oct]].brightness = F_MAX_BRIGHTNESS;
					led_cont.inring[j].brightness = F_MAX_BRIGHTNESS;
				}
				// idle channels are grey-ed
				else if (!button_pressed(i)){
					led_cont.outring[OCT_OUTRING_MAP[oct]].brightness = 0.1;
					led_cont.inring[j].brightness = 0.1;
				}
			}
		}
	}
}

void update_ongoing_display_timers(void){

	static uint32_t 	last_systick = 0;
	uint32_t 			elapsed_ticks;
	uint8_t				tick_down=0;
	uint32_t 			now = HAL_GetTick()/TICKS_PER_MS;

	elapsed_ticks = now - last_systick;
	last_systick = now;

	if (led_cont.ongoing_display == ONGOING_DISPLAY_NONE)
		return;

	if ( led_cont.ongoing_display == ONGOING_DISPLAY_OSC_PARAM_LOCK)
		tick_down = 1;

	else if ( led_cont.ongoing_display == ONGOING_DISPLAY_OCTAVE &&  macro_states.all_af_buttons_released )
		tick_down = 1;

	else if ( led_cont.ongoing_display == ONGOING_DISPLAY_SCALE && macro_states.all_af_buttons_released && !rotary_pressed(rotm_OCT) && !switch_pressed(FINE_BUTTON) )
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_TRANSPOSE && macro_states.all_af_buttons_released && !rotary_pressed(rotm_TRANSPOSE) )
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_FINETUNE && macro_states.all_af_buttons_released && !rotary_pressed(rotm_TRANSPOSE) && !switch_pressed(FINE_BUTTON) )
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_LFO_TOVCA)
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_LFO_MODE)
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_UNISON && !rotary_pressed(rotm_TRANSPOSE) && !rotary_pressed(rotm_WAVETABLE))
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_SOFT_CLIP && !rotary_pressed(rotm_WAVETABLE))
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_SPHERE_SEL)
		tick_down = 1;

	// CPU usage: no auto-timeout on release (it's a toggle now)
	else if (led_cont.ongoing_display == ONGOING_DISPLAY_CPU_USAGE)
		tick_down = 0;

	else if ( led_cont.ongoing_display == ONGOING_DISPLAY_FX &&  macro_states.all_af_buttons_released )
		tick_down = 1;

	else if ((led_cont.ongoing_display == ONGOING_DISPLAY_PRESET) && rotary_released(rotm_PRESET))
		tick_down = 1;

	else if ((led_cont.ongoing_display == ONGOING_DISPLAY_SPHERE_SAVE) && rotary_released(rotm_PRESET))
		tick_down = 1;

	else if ((led_cont.ongoing_display == ONGOING_DISPLAY_GLOBRIGHT) && rotary_released(rotm_PRESET))
		tick_down = 1;
	
	else if (led_cont.ongoing_display == ONGOING_DISPLAY_SELBUS)
		tick_down = 1;

	else if (led_cont.ongoing_display == ONGOING_DISPLAY_UNISON && !rotary_pressed(rotm_TRANSPOSE))
		tick_down = 1;

	/* Unconditional: start_ongoing_display_drum_param() re-arms the full
	 * timeout on every encoder tick, so sustained turning keeps this
	 * alive on its own; ticking down unconditionally just lets it fade
	 * out shortly after the user stops. */
	else if (led_cont.ongoing_display == ONGOING_DISPLAY_DRUM_PARAM)
		tick_down = 1;

	/* Same idiom: start_ongoing_display_drum_preset() re-arms this on
	 * every turn and on every load/save, so it only actually counts
	 * down once the user stops touching the PRESET encoder. */
	else if (led_cont.ongoing_display == ONGOING_DISPLAY_DRUM_PRESET)
		tick_down = 1;

	if (!tick_down)
		return;

	if (led_cont.ongoing_timeout)
	{
		if (led_cont.ongoing_timeout < elapsed_ticks)
			led_cont.ongoing_timeout = 0;
		else
			led_cont.ongoing_timeout -= elapsed_ticks;
	}

	if (!led_cont.ongoing_timeout)
	{
		led_cont.ongoing_display 	= ONGOING_DISPLAY_NONE;
		led_cont.ongoing_timeout 	= 0;
	}
}

void start_ongoing_display_finetune(void){
	led_cont.ongoing_display 	= ONGOING_DISPLAY_FINETUNE;
	led_cont.ongoing_timeout  	= FINETUNE_TIMER_LIMIT;
}

void start_ongoing_display_octave(void){
	led_cont.ongoing_display 	= ONGOING_DISPLAY_OCTAVE;
	led_cont.ongoing_timeout	= OCTAVE_TIMER_LIMIT;
}

void start_ongoing_display_scale(void){
	led_cont.ongoing_display 	= ONGOING_DISPLAY_SCALE;
	led_cont.ongoing_timeout 	= SCALE_TIMER_LIMIT;
}

void start_ongoing_display_transpose(void){
	led_cont.ongoing_display	= ONGOING_DISPLAY_TRANSPOSE;
	led_cont.ongoing_timeout  	= TRANSPOSE_TIMER_LIMIT;
}

void start_ongoing_display_lfo_tovca(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_LFO_TOVCA;
	led_cont.ongoing_timeout = LFO_TOVCA_TIMER_LIMIT;
}

void start_ongoing_display_lfo_mode(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_LFO_MODE;
	led_cont.ongoing_timeout = LFO_MODE_TIMER_LIMIT;
}

void start_ongoing_display_preset(void)
{
	if(led_cont.ongoing_display != ONGOING_DISPLAY_GLOBRIGHT){
		led_cont.ongoing_display 	= ONGOING_DISPLAY_PRESET;
		led_cont.ongoing_timeout 	= PRESET_TIMER_LIMIT;
	}
}

void start_ongoing_display_fx(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_FX;
	led_cont.ongoing_timeout = FX_TIMER_LIMIT;
}

void start_ongoing_display_sphere_save(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_SPHERE_SAVE;
	led_cont.ongoing_timeout = SPHERE_SAVE_TIMER_LIMIT;
}
void start_ongoing_display_sphere_play_export(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_SPHERE_PLAYEXPORT;
	led_cont.ongoing_timeout = SPHERE_SAVE_TIMER_LIMIT;
}

void start_ongoing_display_globright(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_GLOBRIGHT;
	led_cont.ongoing_timeout = GLOBRIGHT_TIMER_LIMIT;
}

void start_ongoing_display_soft_clip(void) {
	led_cont.ongoing_display = ONGOING_DISPLAY_SOFT_CLIP;
	led_cont.ongoing_timeout = FINETUNE_TIMER_LIMIT;
}

/* Cancel any transient parameter overlay so the caller's own display
 * (or the default drum-pattern view) takes over immediately. */
void stop_all_displays(void)
{
	led_cont.ongoing_display = ONGOING_DISPLAY_NONE;
	led_cont.ongoing_timeout = 0;
}

void start_ongoing_display_sphere_sel(void){
	led_cont.ongoing_display = ONGOING_DISPLAY_SPHERE_SEL;
	led_cont.ongoing_timeout = SPHERE_SEL_TIMER_LIMIT;
}

void start_ongoing_display_selbus(void) {
	led_cont.ongoing_display = ONGOING_DISPLAY_SELBUS;
	led_cont.ongoing_timeout = PRESET_TIMER_LIMIT;
}

void start_ongoing_display_unison(void) {
	led_cont.ongoing_display = ONGOING_DISPLAY_UNISON;
	led_cont.ongoing_timeout = FINETUNE_TIMER_LIMIT;
}

/* Calibration sweep state: on entry to CPU-usage mode, light the inner
 * ring LEDs one at a time in index order so the user can map their
 * clock-position observations to inring[0..5] unambiguously.  Set by
 * start_ongoing_display_cpu_usage(), consumed by display_cpu_usage(). */
static uint32_t cpu_usage_entry_tick = 0;
static uint8_t  cpu_usage_calibration_active = 0;
/* Step time in *real* milliseconds.  HAL_GetTick() returns values at
 * TICKS_PER_MS (8x) the rate of wall-clock ms, so the arithmetic below
 * multiplies by TICKS_PER_MS when comparing elapsed ticks. */
#define CPU_USAGE_CAL_STEP_MS 700u
#define CPU_USAGE_CAL_STEPS   NUM_LED_INRING

void start_ongoing_display_cpu_usage(void)
{
	led_cont.ongoing_display = ONGOING_DISPLAY_CPU_USAGE;
	led_cont.ongoing_timeout = 0;
	cpu_usage_entry_tick = HAL_GetTick();
	cpu_usage_calibration_active = 1;
}

void display_cpu_usage(void)
{
	/* Temporary diagnostic layout, refined second pass.
	 *
	 *   OUTER RING (18 LEDs) — main-loop iteration peak period.
	 *       Each LED = 10 ms. 0–4 = green, 5–9 = yellow, 10+ = red.
	 *
	 *   INNER RING (6 LEDs) — identified by calibration sweep as
	 *   starting at 1 o'clock (inring[0]) and running clockwise:
	 *     inring[0] ( 1 o'clock) : read_freq() peak (coarse).
	 *     inring[1] ( 3 o'clock) : OSC_TIM TOTAL peak — FINE thresholds
	 *                               (budget = 555 µs @ 1.8 kHz).
	 *     inring[2] ( 5 o'clock) : process_audio_block_codec() peak —
	 *                               FINE thresholds (budget = 1 ms).
	 *     inring[3] ( 7 o'clock) : halo_advance_cycle() peak —
	 *                               FINE thresholds.  Physics-only cost
	 *                               for a single channel (no trigger
	 *                               path).  Primary suspect for OSC_TIM
	 *                               overrun after refresh_ring_seed_caches
	 *                               was cleared.
	 *     inring[4] ( 9 o'clock) : OSC_TIM per-channel loop peak —
	 *                               FINE thresholds.  Covers the 6×
	 *                               update_pitch+halo_tick pass.
	 *     inring[5] (11 o'clock) : single halo_tick() peak —
	 *                               FINE thresholds.  Worst case one
	 *                               channel's physics + seed-lerp.
	 *                               Should track inring[3] closely in
	 *                               steady state; diverges on note change
	 *                               if the seed_lerp path is slow.
	 *
	 *   Main-loop peak (outer ring) and retrigger/recalc counts are
	 *   still tracked in globals for debugger inspection.
	 *
	 *   Coarse peak units (LED 0): off <1 ms / dim <5 ms / green
	 *   <20 ms / yellow <50 ms / red ≥50 ms.
	 *   Fine peak units  (LEDs 1–5): off <200 µs / dim <500 µs /
	 *   green <1 ms / yellow <2 ms / red ≥2 ms. */
	extern volatile uint32_t diag_main_loop_peak_cycles;
	extern volatile uint32_t diag_osc_tim_peak_cycles;
	extern volatile uint32_t diag_read_freq_peak_cycles;
	extern volatile uint32_t diag_audio_isr_peak_cycles;
	extern volatile uint32_t diag_reverb_peak_cycles;
	extern volatile uint32_t diag_osc_refresh_peak_cycles;
	extern volatile uint32_t diag_osc_chanloop_peak_cycles;
	extern volatile uint32_t diag_osc_ringtick_peak_cycles;
	extern volatile uint32_t diag_advance_cycle_peak_cycles;
	extern volatile uint32_t diag_seed_lerp_peak_cycles;

	/* ── Calibration sweep ──────────────────────────────────────────────
	 * For the first NUM_LED_INRING × 300 ms after entering CPU-usage
	 * mode, light ONE inner LED at a time in index order with a
	 * distinctive per-index colour.  Mapping key:
	 *   inring[0] → WHITE   (read_freq peak)
	 *   inring[1] → RED     (OSC_TIM peak)
	 *   inring[2] → GREEN   (audio ISR peak)
	 *   inring[3] → BLUE    (Reverb peak)
	 *   inring[4] → YELLOW  (starvation)
	 *   inring[5] → PURPLE  (chord recalc count)
	 * After the sweep, the normal diagnostic display takes over. */
	if (cpu_usage_calibration_active) {
		/* HAL_GetTick() counts at TICKS_PER_MS (8×) real ms, so
		 * scale the per-step duration accordingly. */
		uint32_t elapsed = HAL_GetTick() - cpu_usage_entry_tick;
		uint32_t step = elapsed / (CPU_USAGE_CAL_STEP_MS * TICKS_PER_MS);
		if (step >= CPU_USAGE_CAL_STEPS) {
			cpu_usage_calibration_active = 0;
			/* Reset all peak counters so the first real window
			 * isn't polluted by spikes during the sweep. */
			diag_main_loop_peak_cycles      = 0;
			diag_osc_tim_peak_cycles        = 0;
			diag_read_freq_peak_cycles      = 0;
			diag_audio_isr_peak_cycles      = 0;
			diag_reverb_peak_cycles         = 0;
			diag_osc_refresh_peak_cycles    = 0;
			diag_osc_chanloop_peak_cycles   = 0;
			diag_osc_ringtick_peak_cycles   = 0;
			diag_advance_cycle_peak_cycles  = 0;
			diag_seed_lerp_peak_cycles      = 0;
			/* fall through to normal display below */
		} else {
			static const uint8_t cal_colors[CPU_USAGE_CAL_STEPS] = {
				ledc_WHITE,   /* inring[0] → read_freq */
				ledc_RED,     /* inring[1] → OSC_TIM total */
				ledc_GREEN,   /* inring[2] → audio ISR */
				ledc_BLUE,    /* inring[3] → advance_cycle */
				ledc_YELLOW,  /* inring[4] → OSC_TIM chan-loop */
				ledc_PURPLE,  /* inring[5] → halo_tick single */
			};
			for (uint8_t c = 0; c < NUM_LED_INRING; c++) {
				if (c == step)
					set_rgb_color(&led_cont.inring[c], cal_colors[c]);
				else
					led_cont.inring[c].brightness = 0;
			}
			for (uint8_t i = 0; i < NUM_LED_OUTRING; i++)
				led_cont.outring[i].brightness = 0;
			return;
		}
	}

	/* Slow decay so a single spike lingers ≈1 s, but trends fade.
	 * Called at the LED refresh rate (≈500 Hz from timekeeper); decay
	 * only every 50 real ms (= 400 HAL ticks at TICKS_PER_MS=8) to make
	 * the decay rate independent of caller. */
	static uint32_t last_decay_tick = 0;
	uint32_t now = HAL_GetTick();
	if (now - last_decay_tick >= (50u * TICKS_PER_MS)) {
		last_decay_tick = now;
		diag_main_loop_peak_cycles      = (diag_main_loop_peak_cycles      * 63) >> 6;
		diag_osc_tim_peak_cycles        = (diag_osc_tim_peak_cycles        * 63) >> 6;
		diag_read_freq_peak_cycles      = (diag_read_freq_peak_cycles      * 63) >> 6;
		diag_audio_isr_peak_cycles      = (diag_audio_isr_peak_cycles      * 63) >> 6;
		diag_reverb_peak_cycles         = (diag_reverb_peak_cycles         * 63) >> 6;
		diag_osc_refresh_peak_cycles    = (diag_osc_refresh_peak_cycles    * 63) >> 6;
		diag_osc_chanloop_peak_cycles   = (diag_osc_chanloop_peak_cycles   * 63) >> 6;
		diag_osc_ringtick_peak_cycles   = (diag_osc_ringtick_peak_cycles   * 63) >> 6;
		diag_advance_cycle_peak_cycles  = (diag_advance_cycle_peak_cycles  * 63) >> 6;
		diag_seed_lerp_peak_cycles      = (diag_seed_lerp_peak_cycles      * 63) >> 6;
	}

	/* 1 ms at 216 MHz = 216 000 cycles. */
	const uint32_t CYCLES_PER_MS   = 216000u;
	const uint32_t CYCLES_PER_10MS = 2160000u;

	/* OUTER RING — main-loop peak period (1 LED per 10 ms). */
	uint16_t ml_fill = diag_main_loop_peak_cycles / CYCLES_PER_10MS;
	if (ml_fill > NUM_LED_OUTRING) ml_fill = NUM_LED_OUTRING;
	for (uint8_t i = 0; i < NUM_LED_OUTRING; i++) {
		if (i < ml_fill) {
			uint8_t color;
			if (i >= 10)      color = ledc_RED;
			else if (i >= 5)  color = ledc_YELLOW;
			else              color = ledc_LIGHT_GREEN;
			set_rgb_color(&led_cont.outring[i], color);
		} else {
			led_cont.outring[i].brightness = 0;
		}
	}

	/* INNER RING. LED 0 coarse; LEDs 1–5 fine sub-ms thresholds so
	 * we can tell "at budget" from "4x over budget". */
	uint32_t inring_vals[6];
	inring_vals[0] = diag_read_freq_peak_cycles;
	inring_vals[1] = diag_osc_tim_peak_cycles;
	inring_vals[2] = diag_audio_isr_peak_cycles;
	inring_vals[3] = diag_advance_cycle_peak_cycles;
	inring_vals[4] = diag_osc_chanloop_peak_cycles;
	inring_vals[5] = diag_osc_ringtick_peak_cycles;

	for (uint8_t c = 0; c < NUM_LED_INRING && c < 6; c++) {
		uint32_t v = inring_vals[c];
		uint8_t color;
		if (c == 0) {
			/* Coarse thresholds for read_freq (this call can
			 * legitimately take 1–50 ms on a chord recalc). */
			if      (v <  1u * CYCLES_PER_MS) color = ledc_OFF;
			else if (v <  5u * CYCLES_PER_MS) color = ledc_DIM_GREEN;
			else if (v < 20u * CYCLES_PER_MS) color = ledc_LIGHT_GREEN;
			else if (v < 50u * CYCLES_PER_MS) color = ledc_YELLOW;
			else                               color = ledc_RED;
		} else {
			/* Fine thresholds for all OSC_TIM/ISR sub-items. */
			if      (v < (CYCLES_PER_MS / 5))   color = ledc_OFF;        /* <200 µs */
			else if (v < (CYCLES_PER_MS / 2))   color = ledc_DIM_GREEN;  /* <500 µs */
			else if (v < (1u * CYCLES_PER_MS))  color = ledc_LIGHT_GREEN;/* <1 ms   */
			else if (v < (2u * CYCLES_PER_MS))  color = ledc_YELLOW;     /* <2 ms   */
			else                                color = ledc_RED;        /* ≥2 ms   */
		}
		if (color == ledc_OFF)
			led_cont.inring[c].brightness = 0;
		else
			set_rgb_color(&led_cont.inring[c], color);
	}
}
