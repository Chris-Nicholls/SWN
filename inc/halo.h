/*
 * halo.h - Halo circular-buffer physical string model
 *
 * A 512-sample circular buffer represents one period of a waveform.
 * Two arrays — position q[] (the audible output, stored in wt_osc.mc[])
 * and velocity v[] — evolve via Lagrangian physics each waveform period.
 *
 * Algorithm per advanceCycle():
 *   1. ADSR-enveloped colored noise injection into v[]
 *   2. Antipodal coupling: force between q[n] and q[n+N/2]
 *   3. Position update: q[n] += dt * v[n]
 *   4. Circular cascaded 2-pole LPF on both v[] and q[]
 *
 * Port of the LagrangianAcoustics class from app/js/synths.js.
 */

#pragma once

#include <stdint.h>
#include "globals.h"
#include "sphere.h"

#define RS_N            WT_TABLELEN   /* 512 samples per period (max) */
#define RS_DT           0.02f         /* integration timestep (fixed) */
#define RS_NONLINEARITY 0.01f         /* antipodal coupling strength (fixed) */

/* ── Pitch-adapted physics buffer ─────────────────────────────────────────
 * The wavetable storage is RS_N=512 samples, but the physics only needs to
 * cover one period of the audible note.  At low pitches one period spans
 * many audio samples (e.g. 480 @ 100 Hz / 48 kHz), so we use the full 512.
 * At higher pitches one period is much shorter — anything finer than that
 * many samples is aliased away by the audio ISR's wavetable read anyway,
 * so we run the physics at a smaller M ≤ RS_N.
 *
 * Per-channel M (rs->phys_N) is chosen at note-trigger time from the
 * current pitch and frozen for the duration of the note.  The audio ISR
 * plays back the first M samples of the wavetable with a phase increment
 * scaled to match (so audible frequency is unchanged).
 *
 * Multiple-of-16 simplifies vectorisation alignment and gives ample
 * granularity for chord-mode pitches.  Minimum 96 keeps the cascaded
 * 2-pole circular LPF stable across the full user-exposed cutoff range
 * (lpfCutoff ∈ [1, 42]; filter requires cutoff < N/2). */
#define RS_M_MIN        96
#define RS_M_STEP       16

/* Compute the physics buffer size that best matches a given audio
 * frequency (Hz).  Chooses M ≈ Fs/f, rounded to a multiple of RS_M_STEP
 * and clamped to [RS_M_MIN, RS_N].  Inline so it's cheap to call from the
 * OSC_TIM ISR. */
static inline int32_t halo_phys_n_for_pitch(float pitch_hz, float fs_hz)
{
    if (!(pitch_hz > 0.0f)) return RS_N; /* defensive: silent / invalid */
    int32_t m = (int32_t)((fs_hz / pitch_hz) + 0.5f);
    m = (m / RS_M_STEP) * RS_M_STEP;
    if (m < RS_M_MIN) m = RS_M_MIN;
    if (m > RS_N) m = RS_N;
    return m;
}

/* ---------- ADSR envelope phases ---------- */
enum RsEnvPhase {
    RS_ENV_ATTACK,
    RS_ENV_DECAY,
    RS_ENV_SUSTAIN,
    RS_ENV_RELEASE,
    RS_ENV_OFF
};

/* ---------- Per-channel state ---------- */
typedef struct o_halo {

    /* Velocity array — placed in SRAM1 */
    float       v[RS_N];

    /* Raw (pre-gain, pre-noise-mix) wavetable cycle captured at trigger
     * time, already resampled to phys_N samples and normalised to
     * [-1, 1].  Used by halo_advance_cycle for per-cycle
     * wavetable injection when wtAttack > 0 (JS reference:
     * `this._wtOriginal`).  Written by halo_seed_lerp alongside
     * the initial q[] seed. */
    float       _wtOriginal[RS_N];

    /* ADSR envelope state */
    enum RsEnvPhase envPhase;
    float       envLevel;           /* current envelope amplitude [0,1] */
    uint32_t    envCycle;           /* cycles elapsed in current phase */
    float       envReleaseStart;    /* level captured at note-off */

    /* Xorshift32 RNG (deterministic, no heap) */
    uint32_t    rngState;

    /* One-pole LP state for colored noise */
    float       noiseFilterState;

    /* Cached streaming-LPF coefficients (legacy: still consumed by
     * the now-unused halo_advance_cycle and the host harness's
     * Engine A).  The active firmware path lives in
     * halo_voice.cpp. */
    float       lpfCachedCutoffQ;
    float       lpfAlphaQ;
    float       lpfStateQ1;
    float       lpfStateQ2;

    float       lpfCachedCutoffV;
    float       lpfAlphaV;
    float       lpfStateV1;
    float       lpfStateV2;

    /* --- Runtime parameters (set from encoders/CVs) --- */
    int32_t     lpfCutoff;          /* harmonic number 1–64, default 24 */
    float       damping;            /* merged damping 0–1,   default 0.2 */
    float       noiseLevel;         /* 0–1,                  default 0.1 */
    float       noiseColor;         /* 0–1,                  default 0.4 */
    /* wtAttack: 0..1 — at 0, the entire wavetable is seeded into q[]
     * at note trigger (instant, "plucked" attack).  At 1, q[] starts
     * at zero and the waveform is built up via per-cycle injection
     * into v[] over many cycles (slow "bowed" attack).  Matches the
     * JS `wtAttack` parameter added to the LagrangianAcoustics class.
     * Default 0 reproduces the previous behaviour. */
    float       wtAttack;           /* 0–1,                  default 0   */

    /* Noise ADSR times (in cycles) — driven by LPG decay or fixed */
    uint32_t    noiseAttack;        /* default 5 */
    uint32_t    noiseDecay;         /* default 50 */
    float       noiseSustain;       /* level 0–1, default 0.3 */
    uint32_t    noiseRelease;       /* default 100 */

    /* Pitch-adapted physics buffer length.  Set at note trigger from
     * the current pitch (see halo_phys_n_for_pitch).  Stays
     * constant for the duration of the note so q[] / v[] never need
     * resampling mid-flight.  At init: RS_N (full 512). */
    int32_t     phys_N;

    /* Pending trigger flag (consumed by advance_cycle) */
    uint8_t     triggered;

    /* External envelope level (0..1) driven by the caller each OSC_TIM
     * tick — sourced from lfos.out_lpf[chan] so the noise amplitude
     * tracks whichever envelope is active on this channel (LPG vactrol
     * when mode==lfot_LPG, LFO shape when mode==lfot_LFO). This replaces
     * the internal ADSR, which was independent of the LPG/VCA and caused
     * note-start pops from its abrupt attack. */
    float       externalEnvLevel;

    /* ───────── Cross-context physics scheduling ─────────
     * advance_cycle() runs in OSC_TIM IRQ (priority 1,1), not in the
     * SAI audio ISR (priority 0,0). The audio ISR reads from the front
     * buffer (mc[buffer_sel[chan]]) only; OSC_TIM writes into the back
     * buffer (mc[!buffer_sel]) and flips buffer_sel atomically once the
     * new cycle is ready.
     *
     *   triggerPending : set by audio ISR when ring_trigger fires;
     *                    consumed by OSC_TIM which re-seeds q_back from
     *                    the current (possibly interpolated) seed cache
     *                    and re-attacks the internal envelope.
     *   cycleRequest   : set by audio ISR on phase wrap; consumed by
     *                    OSC_TIM which produces a fresh back buffer.
     *   backReady      : set by OSC_TIM when the back buffer holds a
     *                    complete, post-advance_cycle state; consumed
     *                    by audio ISR on the next phase wrap (flip).
     *
     * All three are single-byte flags — reads/writes are atomic on ARM. */
    volatile uint8_t triggerPending;
    volatile uint8_t cycleRequest;
    volatile uint8_t backReady;

} o_halo;

/* ---------- API ---------- */

/* Initialise all state to defaults, zero v[], set q[] to sine.
 * q points to wt_osc.mc[sel][chan][0]. */
void halo_init(o_halo *rs, float *q);

/* Run one cycle of the physics simulation on q[RS_N].
 * Legacy batch path — superseded by the streaming HaloVoice
 * (see inc/halo_voice.hpp).  Kept for the host harness's
 * Engine A comparison build only. */
void halo_advance_cycle(o_halo *rs, float *q);

/* Drive one iteration of the cross-context state machine.
 * Called from update_oscillators() (OSC_TIM IRQ). Consumes triggerPending
 * and cycleRequest; produces backReady when a new cycle has been rendered.
 *   q_front : mc[buffer_sel[chan]][chan]   (the buffer the audio ISR reads)
 *   q_back  : mc[!buffer_sel[chan]][chan]  (the buffer we write into)
 *   seed_a  : waveform at floor(browse_pos) from the current bank
 *   seed_b  : waveform at ceil(browse_pos)  from the current bank
 *   frac    : 0..1, fractional part of browse_pos (lerp weight).
 * When triggerPending is set, q_back is seeded with lerp(seed_a, seed_b, frac)
 * and backReady is flagged so the audio ISR crosses over on the next phase
 * wrap. If either seed pointer is NULL (in-flight flash read) the seed is
 * skipped and the running buffer continues unchanged. */
void halo_tick(o_halo *rs, float *q_front, float *q_back,
                      const int16_t *seed_a, const int16_t *seed_b,
                      float frac);

/* Copy a seed waveform (int16_t[512] from flash) into q[],
 * normalising to ±1.0, and zero v[].  M is the active physics buffer
 * length (samples), passed explicitly so that the caller can seed
 * q[0..M-1] *before* committing the new M to rs->phys_N — eliminating
 * a race where the audio ISR could observe new phys_N with stale
 * buffer content. */
void halo_seed(o_halo *rs, float *q,
                      const int16_t *waveform, int M);

/* Refresh the cached per-cycle injection source (_wtOriginal) from
 * the current browse position without touching q[], v[], or envelope
 * state.  Call this from OSC_TIM whenever pending_seed_pos may have
 * drifted (typically once per tick per channel round-robin) so that
 * wtAttack > 0 picks up wavetable scrubs immediately rather than
 * having to wait for the next note trigger.  Cheap: a single pass of
 * lerp into the phys_N-length buffer.  M follows the same semantics
 * as halo_seed / halo_seed_lerp. */
void halo_refresh_wt_original(o_halo *rs,
                                     const int16_t *wave_a,
                                     const int16_t *wave_b,
                                     float frac, int M);

/* Seed q[] as linear interpolation between two waveforms by `frac`∈[0,1].
 * Either pointer may be NULL (treated as all-zero); if both are NULL q[]
 * is left unchanged. Velocity buffer is zeroed regardless.  M is the
 * active physics buffer length (samples) — see halo_seed for the
 * motivation. */
void halo_seed_lerp(o_halo *rs, float *q,
                           const int16_t *wave_a, const int16_t *wave_b,
                           float frac, int M);

/* Trigger: start noise ADSR attack phase. */
/* Legacy helper: arms the envelope state on the o_halo struct. The
 * primary public trigger entrypoint is halo_trigger(uint8_t chan, ...)
 * declared in halo_voice.hpp; that's the one external code should call.
 * Renamed from halo_trigger() to avoid an overloaded-name collision in
 * C translation units that include both headers. */
void halo_arm_envelope(o_halo *rs);

/* Release: transition noise ADSR to release phase. */
void halo_release(o_halo *rs);

/* Asynchronous trigger request: set the triggerPending flag for a given
 * channel so halo_tick() (in OSC_TIM IRQ) will re-seed and re-attack
 * on its next invocation. Safe to call from any context — a single byte
 * write to a volatile field. The concrete o_halo instance is
 * resolved via wt_osc.halo_state[chan]. */
void halo_request_trigger(uint8_t chan);
