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
 *   - Depth/Latitude/Longitude push+turn -> that same param's own random
 *     offset amount, rerolled bipolar on every hit; OCT push+turn ->
 *     ghost-note amount; OCT turn -> chaos (Grids: shared; Euclid:
 *     per-channel); LFOMODE press -> cycle the selected channel's
 *     CV-jack mode (trigger/density/filter); FINE held -> record that
 *     channel's filter/decay/other performance, released -> loop it
 *     (see read_automation() in drum_ui.c).
 * Channel roles are fixed by category, one per channel (A=Kick,
 * B=Snare, C=Closed HH, D=Open HH, E=Other, F=Other). The closed-hat
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

	/* 0..1, from rotm_LATITUDE push+turn (sec_DISPPATT). Per-step
	 * probability that a step gets included in ghost_pattern below --
	 * a classic ghost note, layered on top of the real pattern rather
	 * than softening it. See DRUM_GHOST_GAIN for the gain it fires at. */
	float				ghost_amount;

	/* Which steps are ghost hits this bar/lap -- bit i set means step i
	 * (Euclid: 0..n-1; Grids: 0..31, this channel's own copy even
	 * though the step position itself is shared) fires a ghost hit if
	 * it wasn't already a real one. Rerolled once per bar (Euclid) or
	 * lap (Grids) from ghost_amount rather than fresh every single
	 * step, so the ghost layer reads as a stable pattern of its own
	 * instead of a different random sprinkle every time round. See
	 * update_drum_triggers(). */
	uint32_t			ghost_pattern;

	/* 0..255, from OCT turn while in Euclid mode (same control, scale,
	 * and step size as the shared pattern_chaos it stands in for --
	 * see read_voice_encoders()). Grids-driven channels keep reading
	 * the one shared pattern_chaos instead; per-channel chaos is only
	 * meaningful for Euclid, where each channel already has its own
	 * independent pattern. */
	uint8_t				chaos_amount;

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

	/* 0..1, from each of DEPTH/LATITUDE/LONGITUDE's own push+turn
	 * (sec_DISPERSION/sec_DISPPATT/sec_WTSEL_SPREAD respectively, all
	 * dead in the old wavetable UI). How much random bipolar offset
	 * (-amount..+amount) gets added to that param's own base value
	 * every time this channel fires -- a fresh reroll per hit, not
	 * held between hits. See apply_random_offsets() in drum_ui.c. */
	float				filter_random;
	float				decay_random;
	float				other_random;

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

	/* Which of trigger/density-mod/filter-mod this channel's own CV
	 * jack (A_VOCT+c) is currently doing -- cycled by butm_LFOMODE_BUTTON.
	 * See update_drum_triggers() (trigger), read_channel_sliders()
	 * (density), and read_cv_filter_mod() (filter) in drum_ui.c. */
	uint8_t				cv_mode;

	/* Automation transport for this channel's filter/decay/other, driven
	 * by holding FINE (see read_automation() in drum_ui.c): OFF while
	 * under manual knob control, RECORD while FINE is held (sampling the
	 * live knob values into the lanes below once per bar_tick), PLAY
	 * once FINE is released (looping the last recording, linearly
	 * interpolated between its DRUM_BAR_TICKS points). Turning
	 * Depth/Latitude/Longitude manually while PLAY-ing cancels back to
	 * OFF -- see apply_filter_delta() etc. Not saved with presets/
	 * autosave in v1; lost on power-cycle, same as any other live
	 * performance loop. */
	uint8_t				automation_state;
	float				automation_filter[DRUM_BAR_TICKS];
	float				automation_decay[DRUM_BAR_TICKS];
	float				automation_other[DRUM_BAR_TICKS];

	/* Performance mode only (VOCTSW -- see drum_ui_performance_mode()):
	 * this channel's mute. A plain channel-button press toggles it
	 * immediately; FINE+press instead queues the toggle for the next
	 * bar boundary, applied in update_drum_triggers(). Not saved with
	 * presets/autosave -- always starts unmuted on boot, same as
	 * automation above. */
	uint8_t				muted;

	uint8_t				state[DRUM_VOICE_STATE_BYTES] __attribute__((aligned(8)));
} o_drum_chan;

enum ChannelCvMode {
	CV_MODE_TRIGGER,	/* today's only behavior: rising edge fires a hit, bypassing the pattern */
	CV_MODE_DENSITY,	/* feeds read_channel_sliders()'s density/k input instead of the physical slider */
	CV_MODE_FILTER,		/* modulates filter cutoff on top of the manual knob position */
	NUM_CV_MODES,
};

enum DrumAutomationState {
	AUTOMATION_OFF,
	AUTOMATION_RECORD,
	AUTOMATION_PLAY,
};

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
 * only drive channels 0..3 (see drum_chan_grids_part()); the two Other
 * Other stay euclidean in either mode. */
enum DrumPatternEngine {
	PATTERN_ENGINE_EUCLID,
	PATTERN_ENGINE_GRIDS,
};

extern enum DrumPatternEngine	drum_pattern_engine;

/* One shared 32-step position for the whole kit (Grids has no
 * per-channel step count), plus its shared X/Y map position, all
 * 0..255. */
extern GridsState	grids_state;
extern uint8_t		grids_x;
extern uint8_t		grids_y;

/* Shared perturbation amount, 0..255, for Grids only -- edited via a
 * plain turn of the OCT encoder (rotm_OCT/pec_OCT -- otherwise fully
 * dead) while in Grids mode, threaded straight into
 * grids_step_active()'s own chaos parameter. The same OCT turn edits
 * each channel's own o_drum_chan.chaos_amount instead while in Euclid
 * mode, since each Euclid channel already has its own independent
 * pattern -- see update_drum_triggers() in drum_ui.c. */
extern uint8_t		pattern_chaos;

/* Shared clock divide/multiply for Grids' one stepper -- see the
 * comment on its definition in drum_ui.c. Same LFO_DIVMULTS[] scale as
 * the per-channel euclidean clock_divmult_id/clock_rate. */
extern float		grids_clock_divmult_id;
extern float		grids_clock_rate;

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

/* True while the panel's VCA/1V-oct switch (VOCTSW) is in its
 * performance-mode position: sliders become channel volume and buttons
 * become mutes (see read_performance_controls() in drum_ui.c) instead
 * of density/select, and everything else -- knob edits, voice
 * browsing, the pattern-engine toggle, CV mode, automation -- is
 * unreachable until the switch flips back. Whichever pattern was
 * already programmed in edit mode keeps playing underneath the whole
 * time; this is a live mixing overlay, not a pause. Read live off the
 * switch, not stored state, same as the FINE_BUTTON checks elsewhere. */
uint8_t drum_ui_performance_mode(void);

/* True while channel c has a FINE+press mute toggle queued for the
 * next bar boundary (performance mode only) -- led_cont.c blinks the
 * channel button between its current and pending state while this
 * holds, so a cued change reads differently from one that already
 * landed. */
uint8_t drum_ui_mute_pending(uint8_t chan);

/* OSC_TIM: advances every channel's pattern on each master-clock step and
 * arms triggers (or takes them from a plugged CV jack instead). */
void update_drum_triggers(void);

/* Audio ISR: renders `n` samples of `chan` into `out` (already filtered
 * and enveloped, unscaled by level). */
void drum_render_channel(uint8_t chan, float *out, int n);
