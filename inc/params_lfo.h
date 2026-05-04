/*
 * params_lfo.h
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



#pragma once

#include "globals.h"

#define LFO_PHASE_TABLELEN				24
#define GLO_CLK							6
#define REF_CLK							7

#define WT_REC_RAMP_AMPLITUDE			254
#define WT_REC_TRIGGER_AMPLITUDE		254

// SCALING
#define F_SCALING_FINE_LFO_SPEED		0.1
#define F_SCALING_LFO_GAIN				0.05
#define F_SCALING_FINE_LFO_GAIN			0.005
#define SCALING_LFO_PHASE				32
#define F_SCALING_FINE_LFO_PHASE		(1.0/12.0) //0.0078125 // 128th note

/* Unified phase-spread (clock-period units).  See lfos.phase_spread.
 *
 * The encoder selects a discrete musical subdivision: each voice is
 * offset by 1/N of a clock period (or N clock periods, for indices
 * past the rhythmic threshold).  This gives 21 stops total — index 0
 * is unison, positive walks fast→strum→beat→sparse, negative mirrors. */
#define PHASE_SPREAD_NUM_STOPS_PER_SIDE  10
#define PHASE_SPREAD_IDX_MIN  (-PHASE_SPREAD_NUM_STOPS_PER_SIDE)
#define PHASE_SPREAD_IDX_MAX  ( PHASE_SPREAD_NUM_STOPS_PER_SIDE)
#define PHASE_SPREAD_LADDER_LEN  (2 * PHASE_SPREAD_NUM_STOPS_PER_SIDE + 1)

/* Per-step delay in clock-period units, indexed [-10..+10] mapped to
 * [0..20].  Defined in src/params_lfo.c. */
extern const float phase_spread_ladder[PHASE_SPREAD_LADDER_LEN];

// DISPLAY
#define LFO_TOVCA_TIMER_LIMIT			1200
#define LFO_TOVCA_FLASH_PERIOD			200

#define LFO_MODE_TIMER_LIMIT			1200
#define LFO_MODE_FLASH_PERIOD			400

#define LFOBANK_DISPLAYTMR				700


#define LFO_INIT_PERIOD					8000
#define LFO_INIT_GAIN					0.625


enum lfoModes{ 

	lfot_LFO,		// Standard LFO mode
	lfot_LPG,		// Low Pass Gate mode

	NUM_LFO_MODES
};

// Keep old name for compatibility during transition
#define lfot_SHAPE lfot_LFO


typedef struct o_lfos
{
	//Parameters
	float 			divmult_id			[NUM_CHANNELS + 2];
	float 			phase_id 			[NUM_CHANNELS];
	int8_t 			shape 				[NUM_CHANNELS];
	float 			gain 				[NUM_CHANNELS];
	uint8_t			locked 				[NUM_CHANNELS];
	enum lfoModes 	mode 				[NUM_CHANNELS];			//Shape/Trig/Gate
	uint8_t 		to_vca 				[NUM_CHANNELS];
	uint8_t 		muted		 		[NUM_CHANNELS];

	uint8_t			use_ext_clock;
	uint8_t			phase_switch;

	/* Unified phase-spread, shared between LFO-VCA mode and LPG-mode chord
	 * strums.  The encoder selects an integer index in
	 * [PHASE_SPREAD_IDX_MIN .. PHASE_SPREAD_IDX_MAX] which is mapped via
	 * phase_spread_ladder[] to a per-channel offset in clock-period units.
	 * The cached `phase_spread` float is the resolved offset (kept in
	 * sync with `phase_spread_idx` by apply_phase_spread()).
	 *
	 *   idx = 0  -> unison
	 *   idx = ±1..±5  -> tight strum  (1/96, 1/48, 1/24, 1/12, 1/6 of a beat)
	 *   idx = ±6..±7  -> wide strum   (1/3, 1/2 of a beat)
	 *   idx = ±8     -> rhythmic     (each voice on the next beat)
	 *   idx = ±9..±10 -> sparse rhythmic (2, 3 beats per voice)
	 *
	 * Sign controls direction: positive => low-channel-first, negative =>
	 * high-channel-first.
	 */
	int8_t			phase_spread_idx;
	float			phase_spread;       /* derived: phase_spread_ladder[idx] */

	// LPG-specific parameters (independent from LFO params)
	float			lpg_decay			[NUM_CHANNELS];		// LPG decay time (0-1), mapped to Speed encoder in LPG mode
	float			lpg_color			[NUM_CHANNELS];		// LPG color/resonance (0-1), mapped to Shape encoder in LPG mode
	float			lpg_gain			[NUM_CHANNELS];		// LPG peak level (0-1), mapped to Gain encoder in LPG mode

	// Global VCA from LFO CV jack
	float			global_vca_level;						// 0.0-1.0, applied to all outputs

	//Resultants
	float			divmult				[NUM_CHANNELS + 2];
	float			period 				[NUM_CHANNELS + 2];
	float 			inc					[NUM_CHANNELS + 2];
	float 			phase 				[NUM_CHANNELS];
	uint8_t			audio_mode			[NUM_CHANNELS];
	float 			divmult_id_global_locked[NUM_CHANNELS];

	//Running outputs
	float			cycle_pos			[NUM_CHANNELS + 2];		//0..1 cycle_pos is position within its cycle
	float 			wt_pos 				[NUM_CHANNELS]; 		//0..1 wt_pos = cycle_pos +/- phase
	uint8_t			div_cnt				[NUM_CHANNELS];			//number of base clocks passed, when dividing

	float 			preload 			[NUM_CHANNELS];
	uint32_t 		envout_pwm 			[NUM_CHANNELS];
	float 			out_lpf 			[NUM_CHANNELS];

	//Stashed values (for switching in and out of key/note mode)
	float			divmult_id_buf		[NUM_CHANNELS];
	int8_t 			phase_id_buf		[NUM_CHANNELS];
	float 			fine_phase_buf		[NUM_CHANNELS];
	int8_t 			shape_buf 			[NUM_CHANNELS];
	float 			gain_buf			[NUM_CHANNELS];
	enum lfoModes	mode_buf 			[NUM_CHANNELS];
	uint8_t 		to_vca_buf 			[NUM_CHANNELS];


	//Probably can be made into statics
	uint8_t 		trig_armed 			[NUM_CHANNELS];
	uint8_t  		trigout 			[NUM_CHANNELS];
	
	// LPG trigger delays for phase-spread timing
	uint16_t		lpg_trigger_delay	[NUM_CHANNELS];		// Countdown in timer ticks (decremented each envout_pwm update)

	/* Deferred LFO phase reset.  Set by main loop (chord retrigger) and
	 * audio ISR (new_key / jack-trig in LFO-VCA mode).  Consumed by
	 * update_oscillators() inside the per-channel trigger atomic block,
	 * so the LFO phase reset becomes synchronous with the Halo
	 * reseed/buffer flip.  Without this, PWM_OUTS_TIM (7.2 kHz) would
	 * recompute out_lpf from the new phase ~140 µs after the main-loop
	 * reset, while the pitch reseed waits for OSC_TIM (1.8 kHz, up to
	 * 555 µs) — the audio ISR perceives the envelope re-attacking
	 * before the pitch updates, producing an audible artifact at
	 * chord retriggers. */
	volatile uint8_t lfo_reset_pending	[NUM_CHANNELS];

} o_lfos;

extern o_lfos lfos;


void update_lfos(float multiplier);
void init_lfos(void);
void use_internal_lfo_base(void);
void clear_lfo_locks(void);
void init_lfo_object(o_lfos *t_lfo);
void init_lfos_shape(void);
void init_lfo_speed(void);
void update_lfo_sample(void);
void update_lfo_params(void);
void apply_lfo_reset(void);
void read_LFO_speed_gain(void);
void update_lfo_gain(int16_t turn);
void read_lfo_speed(int16_t turn);
void read_LFO_phase(void);
void read_LFO_shape(void);
void wrap_lfo_fine_phase(uint8_t chan, float fine_inc);
float calc_lfo_phase(float phase_id);

/* Recompute every unlocked LFO-mode channel's phase_id / phase from the
 * current lfos.phase_spread and that channel's divmult.  LPG-mode strums
 * read lfos.phase_spread directly, so this helper only touches LFO-mode
 * channels. */
void apply_phase_spread(void);

/* Per-channel phase_id (LFO_PHASE_TABLELEN units, wrapped to [0, 24))
 * derived from the unified phase_spread for `chan`.  Matches the bulk
 * apply_phase_spread() math but exposed for the audio-ISR trigger reset
 * path which needs a single channel's value without iterating. */
float phase_id_from_spread(uint8_t chan);
void read_lfo_cv(void);
void init_lfo_to_vc_mode(void);
void cache_uncache_all_lfo_to_vca(enum CacheUncache cache_uncache);
void set_all_lfo_to_vca(uint8_t newstate);
void cache_uncache_lfomode(uint8_t chan, enum CacheUncache cache_uncache);
void cache_uncache_all_lfomodes(enum CacheUncache cache_uncache);
void sync_LFO_phase(void);
void set_all_lfo_mode(enum lfoModes mode);
