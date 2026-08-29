/*
 * drum_ui.h - control surface + pattern/voice glue for the drum station
 *
 * Owns the per-channel drum state (voice algorithm, euclidean pattern,
 * level, pitch, filter/decay/other) and the mapping from the SWN's
 * physical controls onto it.  See doc: sliders = per-channel level,
 * channel buttons = edit-focus selection, Depth/Latitude/Longitude =
 * filter/decay/other, browse encoder = k / n / rotation, Transpose =
 * pitch.
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

typedef struct o_drum_chan {
	const DrumVoiceOps *ops;			// NULL => channel renders silence
	EuclidChannelState	euclid;

	float				level;			// 0..1, from this channel's slider
	float				pitch;			// semitones, from rotm_TRANSPOSE
	float				filter;			// 0..1, rotm_DEPTH
	float				decay;			// 0..1, rotm_LATITUDE
	float				other;			// 0..1, rotm_LONGITUDE

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

void init_drum_ui(void);

/* Main-loop poll: sliders, channel buttons, parameter/pattern encoders. */
void read_drum_ui(void);

/* OSC_TIM: advances every channel's pattern on each master-clock step and
 * arms triggers (or takes them from a plugged CV jack instead). */
void update_drum_triggers(void);

/* Audio ISR: renders `n` samples of `chan` into `out` (already filtered
 * and enveloped, unscaled by level). */
void drum_render_channel(uint8_t chan, float *out, int n);
