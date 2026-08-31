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
 *     LFO speed -> clock divide/multiply, LFO shape -> cycle the
 *     channel's voice within its own fixed category (see
 *     kDrumVoiceRegistry in drum_voice.h), all always acting on the
 *     selected channel (never global).
 * Channel roles are fixed by category, one per channel (A=Kick,
 * B=Snare, C=Closed HH, D=Open HH, E=Crash, F=Other). The closed-hat
 * channel firing always chokes (instantly silences) the open-hat
 * channel, same as a real hi-hat's two sounds sharing one cymbal.
 * All six channels' patterns share one fixed bar length (DRUM_BAR_TICKS
 * master-clock ticks); a channel's `n` subdivides that same span into n
 * steps rather than changing how long the loop takes, so e.g. 4 steps
 * and 8 steps both still loop once per bar at clock_rate 1 -- the
 * 8-step channel's individual steps are just half as long, so (all
 * active) it plays twice as many notes per bar as the 4-step one. LFO
 * speed then scales that per-channel: at clock_rate 1 a channel
 * resyncs to step 0 every bar (so same-n channels never drift apart);
 * slower than 1, its own loop spans more than one bar, so it only
 * resyncs every round(1/clock_rate) bars instead of being snapped back
 * before its pattern has played out (see bars_until_resync).
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
#include "pattern_grids.h"

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

	/* Grids-mode pattern density for this channel, 0..255 (Grids'
	 * own scale, compared against the interpolated 0..255 level --
	 * kept as a byte rather than a 0..1 float like the voice params
	 * below so it feeds grids_step_active() untouched). Only read
	 * while drum_pattern_engine == PATTERN_ENGINE_GRIDS and this
	 * channel is Grids-eligible; the euclidean k is left alone. */
	uint8_t				density;

	/* Output gain multiplier applied on top of `level` for whichever
	 * hit is currently sounding (see oscillator.c's mixing stage).
	 * Only Grids-driven hits ever set this away from 1.0 -- a Grids
	 * step whose post-perturbation level exceeds GRIDS_ACCENT_LEVEL is
	 * an accent, so it's mixed at full gain while everything else is
	 * pulled back, giving audible loud/soft variation without needing
	 * per-voice velocity support. Euclidean hits (and CV-triggered
	 * hits on a Grids-eligible channel, which bypass Grids entirely)
	 * always leave it at 1.0. Set at trigger time in OSC_TIM, held
	 * until the next hit overwrites it -- by then the previous hit has
	 * long since decayed to silence, so there's no audible seam. */
	float				accent_gain;

	/* 0..1, from rotm_TRANSPOSE push+turn (sec_OSC_SPREAD). Applied to
	 * every pattern-triggered hit (both engines; not CV-triggered
	 * hits, which already have real-world timing): a bit of random
	 * gain jitter always, plus a small random *delay* before the hit
	 * actually fires (never early -- there's no way to trigger before
	 * "on the grid"). See schedule_pattern_hit() in drum_ui.c. */
	float				humanize;

	/* Counts down in OSC_TIM ticks to a humanize-delayed fire(); 0 =
	 * nothing pending. Only meaningful while humanize > 0. */
	uint8_t				fire_delay;

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

	/* Counts down the number of shared bar-boundaries left before this
	 * channel's next hard resync (see update_drum_triggers()). A
	 * slowed-down channel (clock_rate<1) takes more than one bar to
	 * complete its own n-step loop, so it must not be snapped back to
	 * step 0 at every single bar boundary -- only at the boundary
	 * where its own loop actually completes. */
	uint8_t				bars_until_resync;

	/* Set by the pattern/CV logic in OSC_TIM, consumed (and cleared) by
	 * the audio ISR just before rendering, so a hit always lands on a
	 * block boundary rather than mid-render. */
	volatile uint8_t	trigger_pending;

	/* Same producer/consumer split as trigger_pending: set from
	 * OSC_TIM (fire()) when the closed-hat channel fires, consumed by
	 * the audio ISR to silence this (the open-hat) channel instantly --
	 * a classic hi-hat choke group. Never touch `state` directly from
	 * OSC_TIM: only the audio ISR reads/writes a voice's state buffer,
	 * so silencing it has to go through this same flag handoff. */
	volatile uint8_t	choke_pending;

	uint8_t				state[DRUM_VOICE_STATE_BYTES] __attribute__((aligned(8)));
} o_drum_chan;

extern o_drum_chan	drum_chan[NUM_CHANNELS];
extern uint8_t		drum_selected_chan;

/* Pressing the button of the already-selected channel (rather than
 * switching to a different one) toggles this instead of changing
 * selection. While set, the knob params in read_voice_encoders()
 * (filter/decay/other/pitch/clock-rate/humanize) apply their delta to
 * every channel at once instead of just drum_selected_chan -- pattern
 * shape (rotation/n/k), voice selection, and CV/preset controls stay
 * per-channel regardless. Selecting a different channel always clears
 * it. */
extern uint8_t		drum_global_edit_mode;

/* Which pattern algorithm drives the kit, toggled kit-wide by
 * butm_LFOVCA_BUTTON. Grids only has kick/snare/hihat data, so it can
 * only drive channels 0..3 (see drum_chan_grids_part()); Crash and
 * Other stay euclidean in either mode. */
enum DrumPatternEngine {
	PATTERN_ENGINE_EUCLID,
	PATTERN_ENGINE_GRIDS,
};

extern enum DrumPatternEngine	drum_pattern_engine;

/* One shared 32-step position for the whole kit (Grids has no
 * per-channel step count), plus its shared X/Y map position and chaos
 * amount, all 0..255. */
extern GridsState	grids_state;
extern uint8_t		grids_x;
extern uint8_t		grids_y;
extern uint8_t		grids_chaos;

/* Grids part (0=kick, 1=snare, 2=hihat) driving channel `c`, or -1 if
 * that channel has no Grids data and always stays euclidean. Both hat
 * channels share the one hihat part; their densities stay independent. */
int8_t	drum_chan_grids_part(uint8_t c);

/* Non-zero for a short while after channel `c` fires; decremented by the
 * LED update so the channel button/inring LED flashes on each hit. */
extern volatile uint8_t drum_trig_flash[NUM_CHANNELS];

/* Non-zero for a short while after channel `c` fires; decremented by
 * update_envout_pwm() (PWM_OUTS_TIM, 7.2 kHz) to drive that channel's
 * ENV OUT jack high as a gate pulse. Same trigger event as
 * drum_trig_flash, different clock domain/consumer.
 *
 * lfos.out_lpf[] (which calculate_lfo_leds() reads for the panel's
 * "LFO out" LEDs) mirrors this same gate, but that LED redraw only
 * runs at 60 Hz (16.7 ms) -- an asynchronous ~2.8 ms pulse against a
 * 16.7 ms sampler is a coin flip to ever get seen at all, which is why
 * those LEDs were barely lighting up. 240 ticks is ~2x one LED period,
 * long enough that the sampler can't miss it regardless of phase,
 * while still reading as a short gate at the physical ENV OUT jack. */
#define DRUM_GATE_TICKS	240		/* ~33.3 ms at 7.2 kHz */
extern volatile uint8_t drum_gate_ticks[NUM_CHANNELS];

/* How long the outer-ring param bar (Depth/Latitude/Longitude) stays up
 * after the last encoder tick, in ms -- see start_ongoing_display_drum_param()
 * in led_cont.c. */
#define DRUM_PARAM_DISPLAY_TIMER_LIMIT	700

void init_drum_ui(void);

/* Main-loop poll: sliders, channel buttons, parameter/pattern encoders. */
void read_drum_ui(void);

/* Call after loading a preset: suspends every channel's slider (k or
 * Grids density) until the user physically moves it back to
 * (approximately) match the just-loaded value, so it can't instantly
 * overwrite what was just loaded. See read_channel_sliders() in
 * drum_ui.c for why this is needed at all. */
void drum_ui_request_slider_pickup(void);

/* True while channel c's slider LED should read as "not showing the
 * real value" (see drum_ui_request_slider_pickup()) -- led_cont.c
 * pulses it instead of the usual behaviour while this holds. */
uint8_t drum_ui_slider_pickup_pending(uint8_t chan);

/* OSC_TIM: advances every channel's pattern on each master-clock step and
 * arms triggers (or takes them from a plugged CV jack instead). */
void update_drum_triggers(void);

/* Audio ISR: renders `n` samples of `chan` into `out` (already filtered
 * and enveloped, unscaled by level). */
void drum_render_channel(uint8_t chan, float *out, int n);
