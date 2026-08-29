/*
 * halo_voice_amortized.hpp — streaming-physics Halo.
 *
 * Same physical model as the firmware (LagrangianAcoustics: antipodal
 * coupling + DC removal + cascaded 2-pole LPF), but instead of advancing
 * the entire M-sample buffer at one wrap moment, the physics is
 * AMORTIZED across audio samples.  One physics "step" updates ONE
 * buffer index; the audio loop runs ~one step per output sample.
 *
 * Why:
 *   - The firmware path running advance_cycle synchronously inside the
 *     audio ISR creates a CPU spike at every buffer wrap — 6 voices
 *     wrapping in one block can blow the audio-ISR budget (~1.33 ms),
 *     causing an underrun click.  Amortization gives constant load
 *     instead of bursts.
 *   - Streaming physics keeps the LPF state continuous by construction
 *     (no buffer flip / wrap discontinuity to mask).
 *
 * Trade-offs:
 *   - DC removal can no longer be a batch pass over the whole buffer;
 *     we use the PREVIOUS cycle's mean to subtract from the CURRENT
 *     cycle's samples (one-cycle DC tracking lag — inaudible).
 *   - Antipodal coupling reads q[n + M/2] which may be from the
 *     previous-cycle or current-cycle pass depending on where phys_head
 *     is relative to that index.  The user explicitly said this is
 *     fine ("can refer to stale state").
 *   - Per-cycle injection's noise state is reset at each phys-cycle
 *     boundary, same as the firmware.
 *
 * Public API matches HaloVoice so test_main can A/B the two.
 */
#pragma once

#include <cstdint>
#include <vector>

class HaloVoiceAmortized {
public:
    static constexpr int   kMaxBufLen   = 512;
    static constexpr float kSampleRate  = 48000.0f;
    static constexpr float kDt          = 0.02f;     /* matches RS_DT */
    static constexpr float kNonlin      = 0.01f;     /* matches RS_NONLINEARITY */
    static constexpr int   kMmin        = 96;
    static constexpr int   kMstep       = 16;

    HaloVoiceAmortized();

    /* ── Note triggers ─────────────────────────────────────────────── */
    void  trigger(const float* waveform_512, float pitch_hz);
    void  retrigger(float pitch_hz);
    void  load_wavetable(const float* waveform_512);

    /* ── Parameter setters ─────────────────────────────────────────── */
    void  set_pitch_hz(float hz);
    void  set_damping(float v)         { damping_     = v; }
    void  set_lpf_cutoff(int v)        { lpf_cutoff_  = v; lpf_dirty_ = true; }
    void  set_noise_level(float v)     { noise_level_ = v; }
    void  set_noise_color(float v)     { noise_color_ = v; }
    void  set_wt_attack(float v)       { wt_attack_   = v; }
    void  set_external_env(float v)    { external_env_ = v; }
    void  set_envelope_ramp(float attack_ms, float sustain_level = 1.0f);

    /* ── Audio rendering ──────────────────────────────────────────── */
    void  fillBlock(float* out, int n_samples);

    /* ── Introspection ────────────────────────────────────────────── */
    int       phys_n()        const { return M_; }
    float     pitch_hz()      const { return pitch_hz_; }
    float     read_head()     const { return read_head_; }
    int       phys_head()     const { return phys_head_; }
    uint64_t  wrap_count()    const { return n_wraps_; }
    uint64_t  phys_count()    const { return n_phys_steps_; }

private:
    /* ── Physics buffers ──────────────────────────────────────────── */
    float    q_[kMaxBufLen];   /* position (also the audio output)     */
    float    v_[kMaxBufLen];   /* velocity                              */
    float    wt_orig_[kMaxBufLen]; /* per-cycle injection source        */

    int      M_         = kMaxBufLen;     /* active buffer length      */

    /* ── Audio playback ───────────────────────────────────────────── */
    float    read_head_ = 0.0f;
    float    read_inc_  = 1.0f;
    float    pitch_hz_  = 110.0f;

    /* ── Physics writer ───────────────────────────────────────────── */
    int      phys_head_ = 0;
    float    phys_accum_ = 0.0f;        /* fractional carry for sub-rate */

    /* Per-cycle physics state (refreshed when phys_head wraps to 0). */
    float    dtt_         = kDt;
    float    inj_amt_     = 0.0f;
    float    wt_amp_      = 0.0f;
    float    noise_amp_   = 0.0f;
    float    alpha_       = 0.0f;
    float    color_gain_  = 1.0f;
    float    damp_scale_  = 0.0f;
    float    noise_state_ = 0.0f;

    /* DC tracking (one-cycle latency).  *_partial accumulates over the
     * current cycle as phys_head sweeps; *_full is the mean we subtract
     * from THIS cycle's samples (carried over from last cycle's
     * partial). */
    float    q_dc_full_     = 0.0f;
    float    q_dc_partial_  = 0.0f;
    float    inj_dc_full_   = 0.0f;
    float    inj_dc_partial_ = 0.0f;

    /* Streaming cascaded 2-pole LPF state.  Q at full cutoff, V at
     * half cutoff (matches the firmware). */
    float    lpf_q1_ = 0.0f, lpf_q2_ = 0.0f;
    float    lpf_v1_ = 0.0f, lpf_v2_ = 0.0f;
    float    lpf_alpha_q_ = 0.0f, lpf_alpha_v_ = 0.0f;
    bool     lpf_dirty_   = true;

    /* ── Runtime parameters ───────────────────────────────────────── */
    int      lpf_cutoff_  = 24;
    float    damping_     = 0.2f;
    float    noise_level_ = 0.1f;
    float    noise_color_ = 0.4f;
    float    wt_attack_   = 0.0f;
    float    external_env_ = 1.0f;

    /* RNG (xorshift32, matches firmware). */
    uint32_t rng_state_ = 0x12345678u;

    /* Trigger crossfade: at every trigger we snapshot the OLD buffer
     * and the OLD audio read state, then blend the dying voice into
     * the new one over kXfadeLen samples.  Same idea as the original
     * engine — just adapted for our single-buffer design. */
    static constexpr int kXfadeLen = 32;
    float    xfade_old_q_[kMaxBufLen];
    int      xfade_remaining_  = 0;
    int      xfade_old_M_      = 0;
    float    xfade_old_head_   = 0.0f;
    float    xfade_old_inc_    = 0.0f;

    /* Optional envelope ramp on external_env_. */
    int      env_attack_samples_ = 0;
    int      env_phase_          = 0;
    float    env_sustain_        = 1.0f;

    /* Stats. */
    uint64_t n_wraps_      = 0;
    uint64_t n_phys_steps_ = 0;

    /* ── Helpers ──────────────────────────────────────────────────── */
    void  recompute_read_inc();
    void  recompute_lpf_coefs();
    void  start_new_phys_cycle();
    void  step_one_physics_index();
    int   phys_n_for_pitch(float hz) const;
};
