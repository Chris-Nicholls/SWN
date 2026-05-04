/*
 * halo_voice.hpp — Halo voice with streaming/amortised
 * physics.  This is the active firmware engine; the legacy batch
 * advance_cycle path in inc/halo.h has been retired (it lives
 * on only as Engine A in the host A/B harness under app/cpp_test/).
 *
 * Algorithm
 * ─────────
 *  - Single position buffer q[M] per voice (M = pitch-adapted, picked
 *    at trigger time).  No double-buffer flip in the steady-state
 *    audio path: a "physics writer head" chases the audio "read head"
 *    through q[] and updates ONE index per physics step.  Physics
 *    rate is matched to audio rate (~one step per audio sample,
 *    fractional carry handles non-integer pitches), so CPU cost is
 *    constant — no M-sample wrap-time spike.
 *  - Velocity buffer v[M] persists across cycles.  Coloured-noise
 *    injection state restarts each phys cycle (matches the JS
 *    reference and the legacy batch path).
 *  - Cascaded 2-pole LPF (q at full cutoff, v at half) runs in
 *    streaming form: the pole state is carried sample-by-sample
 *    inside step_one, naturally continuous across all index
 *    boundaries.  No within-buffer wrap discontinuity to mask.
 *  - DC removal on q[] uses a one-cycle delayed mean (the previous
 *    cycle's running sum becomes the current cycle's correction).
 *    Inaudible.
 *  - Trigger is the only place the buffer changes abruptly; we
 *    snapshot the dying q[] into a 32-sample shadow and read-side
 *    crossfade for kXfadeLen samples to mask the seed step.
 *
 * Why
 * ───
 *  - Host harness (app/cpp_test/) confirmed sonic parity with the
 *    legacy double-buffer engine across 17 stress scenarios, with
 *    BETTER behaviour on hard-saw / inject cases (max sample step
 *    0.553 vs 0.787).
 *  - The legacy path put a synchronous advance_cycle inside the
 *    audio ISR at every wrap; with 6 voices this could blow the
 *    1.33 ms audio-block budget (yellow led[2] in CPU-test mode →
 *    DAC underrun click).  Streaming amortises that cost evenly,
 *    so the worst case equals the average.
 */

#pragma once

#include <stdint.h>
#include "globals.h"
#include "sphere.h"

#ifdef __cplusplus

class HaloVoice {
public:
    /* Compile-time constants — kept in sync with firmware globals. */
    static constexpr int   kBufLen      = (int)WT_TABLELEN;        /* 512 */
    static constexpr float kSampleRate  = (float)F_SAMPLERATE;     /* 48000 */
    static constexpr float kDt          = 0.02f;
    static constexpr float kNonlin      = 0.01f;
    static constexpr int   kMmin        = 96;
    static constexpr int   kMstep       = 16;

    /* Trigger crossfade length.  At 48 kHz, 512 samples ≈ 10.7 ms.
     *
     * Old/new buffers are mixed using equal-power weights
     * (cos/sin of t·π/2) rather than linear or smoothstep.  An
     * equal-power crossfade keeps w_old² + w_new² ≡ 1, which means
     * the summed RMS amplitude stays roughly constant across the
     * transition — critical when six voices retrigger together,
     * because a linear crossfade dips the summed level mid-fade
     * and then snaps it back up, audible as a click against the
     * downstream soft compressor.  Equal-power keeps the
     * pre-compressor amplitude flat so the limiter doesn't have
     * to do any abrupt gain reduction. */
    static constexpr int   kXfadeLen    = 512;

    /* No constructors / destructors run on STM32 BSS instances at
     * boot — call init() once from init_wt_osc(). */
    void init();

    /* Trigger / re-trigger.  Picks a new pitch-adapted M, resamples
     * the 512-sample seed waveform into wt_orig_[0..M-1], seeds q[]
     * scaled by (1 - wtAttack), zeros v[], resets streaming state,
     * and arms a kXfadeLen-sample read-side crossfade against the
     * previous q[].
     *
     * trigger() is implemented as prepare_trigger() followed by
     * commit_trigger() and is the single-call API for tests and
     * any caller that doesn't need to bracket the commit with
     * a __disable_irq() window.  Firmware uses the split form
     * directly so the heavy seed/resample/pre-smooth work runs
     * with audio interrupts enabled (the audio ISR happily plays
     * the still-live old buffer the whole time), and only the
     * fast atomic swap runs inside __disable_irq(). */
    void trigger(const float* wave_512, float pitch_hz);

    /* Heavy half of the trigger.  Builds the new pitch-adapted M,
     * resamples wave_512 into wt_orig_staging_, seeds q_staging_,
     * DC-removes it, and pre-smooths it.  Touches only the per-voice
     * staging buffers — q_/v_/wt_orig_ remain coherent for the
     * still-running OLD voice, so it is safe to call this with
     * audio interrupts ENABLED.  Sets pending_commit_ = true; the
     * caller is then expected to call commit_trigger() before any
     * further parameter changes that affect M_-derived state. */
    void prepare_trigger(const float* wave_512, float pitch_hz);

    /* Light half of the trigger.  Snapshots the dying q[] into the
     * crossfade shadow, copies q_staging_/wt_orig_staging_ into
     * q_/wt_orig_, resets read/phys state, zeros v[], rebuilds LPF
     * coefficients, and arms the read-side crossfade.  Designed to
     * run inside __disable_irq() so audio sees a coherent transition
     * (every voice's swap lands on the same audio sample).
     * No-op if pending_commit_ is false. */
    void commit_trigger();

    /* Update the per-cycle injection source (live wavetable scrub)
     * without touching q[], v[], envelope, or LPF state.  Called
     * from OSC_TIM round-robin to track browse-encoder drift. */
    void load_wavetable(const float* wave_512);

    void set_pitch_hz(float hz);
    void set_damping(float v)         { damping_     = v; }
    void set_lpf_cutoff(int v)        { lpf_cutoff_  = v; lpf_dirty_ = true; }
    void set_noise_level(float v)     { noise_level_ = v; }
    void set_noise_color(float v)     { noise_color_ = v; }
    void set_wt_attack(float v);                /* see .cpp — splits 0..1 into
                                                 * blend (0..0.5) + attack
                                                 * envelope (0.5..1) */
    void set_external_env(float v)    { external_env_ = v; }

    /* Resonator mode: external audio adds to v[idx] before physics.
     * Safe to call from the audio ISR; the cpp class' fillBlock will
     * pick up the deposit on its next physics step at that index. */
    void inject_velocity(int idx, float amount);

    /* Render n_samples of audio into out[].  If audio_in is non-null,
     * each output sample i additionally injects audio_in[i] * 0.5
     * into v[] at the audio read position (resonator mode). */
    void fillBlock(float* out, int n_samples,
                   const float* audio_in = nullptr);

    int   phys_n()    const { return M_; }
    float pitch_hz()  const { return pitch_hz_; }

private:
    /* Position / velocity / per-cycle injection source. */
    float q_[kBufLen];
    float v_[kBufLen];
    float wt_orig_[kBufLen];

    int   M_         = kBufLen;

    /* Audio read head. */
    float read_head_ = 0.0f;
    float read_inc_  = 1.0f;
    float pitch_hz_  = 110.0f;

    /* Physics writer. */
    int   phys_head_  = 0;
    float phys_accum_ = 0.0f;

    /* Per-cycle latched coefficients (refreshed at phys_head wrap). */
    float dtt_         = kDt;
    float inj_amt_     = 0.0f;
    float wt_amp_      = 0.0f;
    float noise_amp_   = 0.0f;
    float alpha_       = 0.0f;
    float color_gain_  = 1.0f;
    float damp_scale_  = 0.0f;
    float noise_state_ = 0.0f;

    /* DC tracking (one-cycle latency). */
    float q_dc_full_      = 0.0f;
    float q_dc_partial_   = 0.0f;
    float inj_dc_full_    = 0.0f;
    float inj_dc_partial_ = 0.0f;

    /* Streaming cascaded 2-pole LPF state. */
    float lpf_q1_ = 0.0f, lpf_q2_ = 0.0f;
    float lpf_v1_ = 0.0f, lpf_v2_ = 0.0f;
    float lpf_alpha_q_ = 0.0f, lpf_alpha_v_ = 0.0f;
    bool  lpf_dirty_ = true;

    /* User parameters. */
    int   lpf_cutoff_  = 24;
    float damping_     = 0.2f;
    float noise_level_ = 0.1f;
    float noise_color_ = 0.4f;
    float external_env_ = 1.0f;

    /* wt_attack split into two derived controls; both updated by
     * set_wt_attack().
     *   inj_blend_       (0..1)  - injection mix used today
     *                              (param 0..0.5 maps to 0..1; >0.5 stays 1)
     *   attack_env_inc_  per-sample step that drives attack_env_ from 0→1.
     *                              Param  0..0.5 → instant (inc = 1);
     *                              Param 0.5..1  → 0..2s ramp (inc shrinks).
     */
    float wt_attack_      = 0.0f;
    float inj_blend_      = 0.0f;
    float attack_env_     = 1.0f;     /* 0 at trigger, ramps to 1 */
    float attack_env_inc_ = 1.0f;     /* per audio sample */

    /* RNG (xorshift32). */
    uint32_t rng_state_ = 0x12345678u;

    /* Trigger crossfade shadow.  The old q[] keeps "playing" via
     * xfade_old_head_ for xfade_total_ samples after a trigger and
     * is mixed linearly with the freshly-seeded q[].  Default total
     * is kXfadeLen but the slow-attack path (wt_attack > 0.5)
     * stretches it to match the attack-envelope duration so the old
     * note doesn't get cut off in 10 ms while the new note takes
     * up to 2 s to reach full amplitude. */
    float xfade_old_q_[kBufLen];
    int   xfade_remaining_ = 0;
    int   xfade_total_     = kXfadeLen;
    int   xfade_old_M_     = 0;
    float xfade_old_head_  = 0.0f;
    float xfade_old_inc_   = 0.0f;

    /* Trigger staging.  prepare_trigger() writes the new seed buffer
     * and resampled wavetable here while audio is still reading the
     * live q_/wt_orig_; commit_trigger() then swaps them into place
     * inside __disable_irq().  Keeping these buffers per-voice (vs.
     * a shared scratch) means all six voices in a chord can be
     * prepared in any order and committed atomically as a batch. */
    float q_staging_[kBufLen];
    float wt_orig_staging_[kBufLen];
    int   pending_M_           = 0;
    float pending_pitch_hz_    = 0.0f;
    float pending_lpf_alpha_q_ = 0.0f;
    float pending_lpf_alpha_v_ = 0.0f;
    float pending_lpf_q1_      = 0.0f;
    float pending_lpf_q2_      = 0.0f;
    bool  pending_commit_      = false;

    /* Internal helpers. */
    void  recompute_read_inc();
    void  recompute_lpf_coefs();
    void  start_new_phys_cycle();
    void  step_one_physics_index();
    int   phys_n_for_pitch(float hz) const;
};

extern "C" {
#endif /* __cplusplus */

/* ── Firmware-facing C API ─────────────────────────────────────────
 * The audio ISR / OSC_TIM / param-update code is C; it talks to the
 * voice array through these dispatchers.  Channel index is bounds-
 * checked; out-of-range calls are no-ops. */

void    halo_init_all(void);

void    halo_trigger(uint8_t chan,
                           const float* wave_512,
                           float pitch_hz);
/* Split-form trigger; see prepare_trigger() / commit_trigger() in
 * the C++ class for semantics.  Firmware code paths that need to
 * minimise the __disable_irq() window during chord retriggers should
 * call halo_prepare_trigger() for every voice first (heavy
 * work, IRQs on) and then halo_commit_trigger() for every
 * voice inside a single __disable_irq()/__enable_irq() bracket. */
void    halo_prepare_trigger(uint8_t chan,
                                   const float* wave_512,
                                   float pitch_hz);
void    halo_commit_trigger(uint8_t chan);
void    halo_load_wavetable(uint8_t chan, const float* wave_512);

void    halo_set_pitch_hz(uint8_t chan, float hz);
void    halo_set_damping(uint8_t chan, float v);
void    halo_set_lpf_cutoff(uint8_t chan, int32_t v);
void    halo_set_noise_level(uint8_t chan, float v);
void    halo_set_noise_color(uint8_t chan, float v);
void    halo_set_wt_attack(uint8_t chan, float v);
void    halo_set_external_env(uint8_t chan, float v);

void    halo_inject_velocity(uint8_t chan, int32_t idx, float amount);
void    halo_fill_block(uint8_t chan,
                              float* out, int32_t n_samples,
                              const float* audio_in);

int32_t halo_phys_n(uint8_t chan);

/* Convenience: build a 512-sample [-1,1] float seed by linearly
 * interpolating two int16 waveforms (NULL endpoints treated as
 * silence).  Drop-in helper for callers that have seed_a / seed_b /
 * frac and need to feed halo_trigger(). */
void    halo_build_seed_wave(const int16_t* wave_a,
                                   const int16_t* wave_b,
                                   float frac,
                                   float* out_512);

#ifdef __cplusplus
}
#endif
