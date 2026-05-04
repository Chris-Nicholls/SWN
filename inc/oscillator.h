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
#include "halo.h"

#define RESONATOR_GAIN			32.0f								// Gain applied to coherence VCA
#define RESONATOR_LED_SCALE		32.0f								// Scale coherence to 0-1 for LED display
#define MAX_UNISON_VOICES		6									// Maximum number of unison voices per channel

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
	float 						wt_head_pos 			[NUM_CHANNELS][MAX_UNISON_VOICES];
	float						wt_head_pos_inc			[NUM_CHANNELS][MAX_UNISON_VOICES];

	// For standard playback (non-interpolated reading)
	uint16_t 					rh0						[NUM_CHANNELS][MAX_UNISON_VOICES];
	uint16_t 					rh1						[NUM_CHANNELS][MAX_UNISON_VOICES];
	float 						rhd						[NUM_CHANNELS][MAX_UNISON_VOICES]; 
	float 						rhd_inv					[NUM_CHANNELS][MAX_UNISON_VOICES];


	// Resonator mode: quadrature coherence detection (pseudo-Hilbert)
	float						coherence_dc_I		[NUM_CHANNELS];			// In-phase DC component (slow LPF)
	float						coherence_dc_Q		[NUM_CHANNELS];			// Quadrature DC component (90° shifted)
	float						coherence_env		[NUM_CHANNELS];			// Envelope of sqrt(I²+Q²)

	// Unison params (cached here for audio thread performance)
	float						unison_spread_amt	[NUM_CHANNELS];
	uint8_t						unison_voice_count	[NUM_CHANNELS];

	// Plaits state
	float						plaits_last_cv_input[NUM_CHANNELS];
	uint32_t					plaits_refractory_timer[NUM_CHANNELS];

	// Halo physical model state (per channel)
	o_halo				halo_state[NUM_CHANNELS];

	// Seed waveform cache — per channel we keep TWO adjacent waveforms
	// from the current bank (floor(pos) and ceil(pos)) so that on note
	// trigger we can linearly interpolate between them by the fractional
	// part of pending_seed_pos. This preserves smooth WT morphing when
	// the user holds the browse encoder between two integer positions.
	int16_t						seed_cache       [NUM_CHANNELS][2][WT_TABLELEN];
	// Float browse position (0..NUM_WAVEFORMS_IN_SPHERE), written by
	// update_wt()'s case 15. Moving this does NOT immediately change
	// what's playing — the currently-running buffer keeps running until
	// the next note trigger, at which point q_back is seeded with
	// lerp(seed_cache[0], seed_cache[1], frac).
	float						pending_seed_pos [NUM_CHANNELS];
	// Which integer index / bank is currently loaded in each cache slot.
	// Set to 0xFFFF during an in-flight flash read so neither
	// update_oscillators nor the audio ISR will use a half-written seed.
	uint16_t					active_seed_idx  [NUM_CHANNELS][2];
	uint16_t					active_seed_bank [NUM_CHANNELS][2];

	// ─── Buffer-flip crossfade ────────────────────────────────────────
	// After every buffer flip (cycle-advance OR trigger) we keep a small
	// "shadow" reader following the previous buffer's trajectory and
	// blend its output against the new buffer's output for xfade_remaining
	// audio samples.  This masks the ~Δ-sample phase rotation that the
	// cascaded circular LPF imparts every cycle (visible as a tiny step
	// at the transition from q_front[M-1] → q_back[0]); without it the
	// step lands once per audible cycle and smears across harmonics as
	// audible aliasing, especially with high damping/cutoff.
	//
	// Done in the OUTPUT (audio ISR) rather than buffer content because
	// any in-place modification to q_back's first/last samples would
	// break the LPF's own circular closure and just relocate the step
	// inside q_back (q_back[M-1] → q_back[0] hard step).  Both buffers
	// stay valid until the next OSC_TIM round-robin pass for this
	// channel (~3 ms grace at 6-channel load) — comfortably more than
	// the ~32-sample xfade duration.
	//
	// Latched atomically with the flip itself: cycle-advance flips
	// happen in the audio ISR (so the state is captured inline);
	// trigger flips happen in OSC_TIM (so the trigger fast path
	// captures state under __disable_irq() before flipping buffer_sel).
	int32_t						xfade_remaining   [NUM_CHANNELS];
	uint8_t						xfade_prev_buffer [NUM_CHANNELS]; // mc[] index of the dying buffer
	float						xfade_prev_head   [NUM_CHANNELS]; // continuing read pos in dying buffer
	int32_t						xfade_prev_M      [NUM_CHANNELS]; // dying buffer's phys_N (might differ on trigger)
	float						xfade_prev_inc    [NUM_CHANNELS]; // dying buffer's phase increment

} o_wt_osc;

// ─── Crossfade duration ───
// 32 samples ≈ 0.67 ms at 48 kHz.  Long enough to mask the LPF phase
// step at any reasonable damping (verified: |Δ| stays under ~6 samples
// across the lpfCutoff × damping range), short enough that the new
// cycle's timbre is fully heard well before the next physics step
// arrives at high pitch.  Adjust here if needed; both event types
// share this length so tuning is one-knob.
#define WT_XFADE_LEN 32


void	init_wt_osc(void);
void 	process_audio_block_codec(int32_t *src, int32_t *dst);
