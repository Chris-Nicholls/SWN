/*
 * oscillator.c
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


#pragma once
#include <stm32f7xx.h>
#include <math.h>

#include "sphere.h"
#include "globals.h"

// Phase modulation LFO shapes
#define PHASE_MOD_LFO_NUM_SHAPES 2
enum PhaseModLfoShapes {
	PHASE_MOD_LFO_SINE = 0,
	PHASE_MOD_LFO_TRIANGLE = 1
};

// Resonator mode constants
#define RESONATOR_ATTACK_FREQ	500.0f								// Fast attack ~10ms
#define RESONATOR_DECAY_FREQ	8.0f								// Slow decay ~500ms
#define RESONATOR_ATTACK_ALPHA	(RESONATOR_ATTACK_FREQ / F_SAMPLERATE)	// ~0.00227
#define RESONATOR_DECAY_ALPHA	(RESONATOR_DECAY_FREQ / F_SAMPLERATE)	// ~0.0000453
#define RESONATOR_GAIN			32.0f								// Gain applied to coherence VCA
#define RESONATOR_LED_SCALE		32.0f								// Scale coherence to 0-1 for LED display

// Compute phase modulation LFO value from position (0-1) and shape
// Returns value in range -1 to +1
static inline float compute_phase_mod_lfo(float pos, uint8_t shape) {
	shape = shape % PHASE_MOD_LFO_NUM_SHAPES;  // Wrap to valid shapes
	switch (shape) {
		case PHASE_MOD_LFO_TRIANGLE:
			// Triangle: rises 0->1 in first half, falls 1->0 in second half, then shift to -1 to +1
			return (pos < 0.5f) ? (4.0f * pos - 1.0f) : (3.0f - 4.0f * pos);
		case PHASE_MOD_LFO_SINE:
		default:
			return sinf(pos * 2.0f * 3.14159265f);
	}
}


enum WtInterpRequests {
	WT_INTERP_REQ_NONE,
	WT_INTERP_REQ_REFRESH,
	WT_INTERP_REQ_FORCE
};


typedef struct o_wt_osc{

	// Current wavetable for each channel (interpolated from within the sphere)
	// Two buffers are kept, so we can crossfade when switching wavetables/spheres
	//
	float 						mc 						[2][NUM_CHANNELS][WT_TABLELEN];
	uint8_t						buffer_sel				[NUM_CHANNELS]		;

	// Status of interpolation and crossfade
	enum WtInterpRequests		wt_interp_request		[NUM_CHANNELS]		;
	float 						wt_xfade				[NUM_CHANNELS]		;

	// Position within sphere, calculated directly from calc_params.wt_pos[DIM][chan]
	//
	uint8_t 					m0						[3][NUM_CHANNELS]	;
	uint8_t 					m1						[3][NUM_CHANNELS]	;
	float 						m_frac					[3][NUM_CHANNELS]	;
	float 						m_frac_inv				[3][NUM_CHANNELS]	;

	// WT READING HEAD
	float 						wt_head_pos 			[NUM_CHANNELS]		;
	float						wt_head_pos_inc			[NUM_CHANNELS]		;
	uint16_t 					rh0						[NUM_CHANNELS]		;
	uint16_t 					rh1						[NUM_CHANNELS]		;
	float 						rhd						[NUM_CHANNELS]		; 
	float 						rhd_inv					[NUM_CHANNELS]		;

	// Phase modulation LFO runtime state (not saved in presets) - per channel
	float						phase_mod_lfo_pos	[NUM_CHANNELS];			// Current LFO position (0-1) per channel
	float						phase_mod_lfo_inc	[NUM_CHANNELS];			// LFO increment per sample per channel

	// Resonator mode: quadrature coherence detection (pseudo-Hilbert)
	float						coherence_dc_I		[NUM_CHANNELS];			// In-phase DC component (slow LPF)
	float						coherence_dc_Q		[NUM_CHANNELS];			// Quadrature DC component (90° shifted)
	float						coherence_env		[NUM_CHANNELS];			// Envelope of sqrt(I²+Q²)
	
} o_wt_osc;


// Per-channel LFO speed multipliers for organic drift
extern const float phase_spread_speed_mult[NUM_CHANNELS];

void	init_wt_osc(void);
void 	process_audio_block_codec(int32_t *src, int32_t *dst);
