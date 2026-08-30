/*
 * drum_ui.h - control surface + pattern/voice glue for the drum station
 *
 * Owns the per-channel drum state (voice algorithm, euclidean pattern,
 * level, pitch, filter/decay/other) and the mapping from the SWN's
 * physical controls onto it:
 *   - channel slider      -> that channel's own active-step count (k),
 *                            i.e. pattern density, no selection needed.
 *   - channel button press -> edit-focus selection (persists until
 *                            another channel button is pressed).
 *   - channel button HELD + browse-encoder turn -> that channel's level.
 *   - browse-encoder turn (no channel held) -> rotate the selected
 *                            channel's pattern.
 *   - browse-encoder push+turn -> selected channel's total step count (n).
 *   - Depth/Latitude/Longitude -> filter/decay/other, Transpose -> pitch,
 *     LFO speed -> clock divide/multiply, LFO shape -> cycle the voice
 *     algorithm (flat across every registered family, see
 *     kDrumVoiceRegistry in drum_voice.h), all always acting on the
 *     selected channel (never global).
 * All six channels' patterns share one fixed bar length (DRUM_BAR_TICKS
 * master-clock ticks); a channel's `n` subdivides that same span into n
 * steps rather than changing how long the loop takes, so e.g. 4 steps
 * and 8 steps both still loop once per bar -- the 8-step channel's
 * individual steps are just half as long, so (all active) it plays
 * twice as many notes per bar as the 4-step one. LFO speed then scales
 * that per-channel, on top of the shared bar.
 * The ENV OUT jacks double as gate outputs: driven high for
 * DRUM_GATE_TICKS PWM ticks whenever that channel fires (see
 * update_envout_pwm() in envout_pwm.c), in lockstep with the existing
 * channel-button trigger-flash LED.
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

#include <stdint.h>

#include "globals.h"
#include "drum_voice.h"
#include "euclid_pattern.h"

/* Largest per-voice state across all ported algorithms.  Sized with
 * headroom so adding a family doesn't force a re-layout of o_drum_chan;
 * init_drum_ui() asserts at runtime by refusing to bind any voice whose
 * state_size exceeds this. */
#define DRUM_VOICE_STATE_BYTES	512

/* Master-clock ticks per bar, shared by every channel regardless of its
 * own step count -- see the file header comment. 16 matches the
 * default pattern length (n=16) so a fresh/default kit's step rate
 * equals the incoming clock 1:1, same as before this existed. */
#define DRUM_BAR_TICKS	16

typedef struct o_drum_chan {
	const DrumVoiceOps *ops;			// NULL => channel renders silence
	EuclidChannelState	euclid;

	float				level;			// 0..1, from this channel's slider
	float				pitch;			// semitones, from rotm_TRANSPOSE
	float				filter;			// 0..1, rotm_DEPTH
	float				decay;			// 0..1, rotm_LATITUDE
	float				other;			// 0..1, rotm_LONGITUDE

	/* Per-channel clock divide/multiply, from rotm_LFOSPEED acting on
	 * the selected channel only (not global). clock_divmult_id indexes
	 * the same LFO_DIVMULTS[] ratio table as the rest of the module
	 * (see calc_divmult_amount()); clock_rate is that ratio and
	 * step_phase is this channel's own fractional step accumulator,
	 * advanced by clock_rate once per master-clock tick in
	 * update_drum_triggers(). */
	float				clock_divmult_id;
	float				clock_rate;
	float				step_phase;

	/* Set by the pattern/CV logic in OSC_TIM, consumed (and cleared) by
	 * the audio ISR just before rendering, so a hit always lands on a
	 * block boundary rather than mid-render. */
	volatile uint8_t	trigger_pending;

	uint8_t				state[DRUM_VOICE_STATE_BYTES] __attribute__((aligned(8)));
} o_drum_chan;

extern o_drum_chan	drum_chan[NUM_CHANNELS];
extern uint8_t		drum_selected_chan;

/* Non-zero for a short while after channel `c` fires; decremented by the
 * LED update so the channel button/inring LED flashes on each hit. */
extern volatile uint8_t drum_trig_flash[NUM_CHANNELS];

/* Non-zero for a short while after channel `c` fires; decremented by
 * update_envout_pwm() (PWM_OUTS_TIM, 7.2 kHz) to drive that channel's
 * ENV OUT jack high as a gate pulse. Same trigger event as
 * drum_trig_flash, different clock domain/consumer. */
#define DRUM_GATE_TICKS	20		/* ~2.8 ms at 7.2 kHz */
extern volatile uint8_t drum_gate_ticks[NUM_CHANNELS];

/* How long the outer-ring param bar (Depth/Latitude/Longitude) stays up
 * after the last encoder tick, in ms -- see start_ongoing_display_drum_param()
 * in led_cont.c. */
#define DRUM_PARAM_DISPLAY_TIMER_LIMIT	700

void init_drum_ui(void);

/* Main-loop poll: sliders, channel buttons, parameter/pattern encoders. */
void read_drum_ui(void);

/* OSC_TIM: advances every channel's pattern on each master-clock step and
 * arms triggers (or takes them from a plugged CV jack instead). */
void update_drum_triggers(void);

/* Audio ISR: renders `n` samples of `chan` into `out` (already filtered
 * and enveloped, unscaled by level). */
void drum_render_channel(uint8_t chan, float *out, int n);
