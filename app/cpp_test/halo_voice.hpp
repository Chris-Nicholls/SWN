/*
 * halo_voice.hpp — host-side C++ harness around the firmware's
 * Halo physics + audio playback loop.  Lets us exercise
 * src/halo.c in isolation, render audio to a buffer/WAV file,
 * and probe state at sample resolution — without the firmware-only
 * scheduling, ISR priorities, FSK diagnostics, etc. that complicate
 * the on-device debug loop.
 *
 * Design contract:
 *   - One instance == one channel, owning a private o_halo and
 *     the per-channel slice of o_wt_osc that the audio ISR touches
 *     (mc[2][N], buffer_sel, wt_head_pos, wt_head_pos_inc, xfade_*).
 *   - fillBlock() drives both the audio playback loop and the physics
 *     advance — exactly mirroring src/oscillator.c's
 *     process_audio_block_codec wrap handler — so any sonic artefact
 *     visible on hardware should reproduce here at the same sample
 *     index.
 *   - Triggers, parameter changes, and wavetable swaps are all
 *     synchronous (no double-buffered ISR scheduling); call them
 *     between fillBlock() invocations to stage them at a known sample
 *     boundary, or interleave them within fillBlock by passing
 *     callbacks (see test_main).
 */
#pragma once

extern "C" {
#include "halo.h"
}

#include <cstdint>
#include <vector>

/* Per-wrap diagnostic record.  Captures the audio sample either side
 * of a buffer wrap so the test driver can quantify the cycle-boundary
 * discontinuity in isolation (separate from any built-in seed steps,
 * which are constant across cycles). */
struct WrapEvent {
    int      sample_index;     /* index in fillBlock's out[]; this sample is the first of the new cycle */
    float    audio_pre;        /* out[sample_index - 1]            */
    float    audio_post;       /* out[sample_index]                */
    float    head_post_wrap;   /* head_pos after the wrap subtract */
    int      buffer_sel_after; /* which mc[] is now front           */
    bool     ran_advance;      /* true if advance_cycle ran on this wrap */
    /* LPF state right after advance ran (post-cycle).             */
    float    lpf_state_q1;
    float    lpf_state_q2;
    /* Boundary samples on each side, BEFORE linear-interp / xfade. */
    float    q_old_last;       /* mc[old_buffer_sel][M-1]          */
    float    q_new_first;      /* mc[new_buffer_sel][0]            */
    float    q_new_second;     /* mc[new_buffer_sel][1]            */
    /* Audio-side read state at the post-wrap sample. */
    float    rhd_post_wrap;    /* fractional part of head_pos       */
    int      rh0_post_wrap;    /* integer part                      */
};

class HaloVoice {
public:
    static constexpr int   kMaxBufLen  = RS_N;        /* 512 */
    static constexpr int   kXfadeLen   = 32;          /* matches WT_XFADE_LEN */
    static constexpr float kSampleRate = 48000.0f;

    HaloVoice();

    /* ── Note triggers ─────────────────────────────────────────────── */

    /* Load a 512-sample waveform (float, ~[-1, 1]) as the seed and
     * trigger a note at the given pitch.  Behaves like the firmware
     * trigger fast-path: picks a pitch-adapted phys_N, seeds the back
     * buffer, then flips buffer_sel atomically while latching a
     * crossfade against the dying front buffer for kXfadeLen samples. */
    void trigger(const float* waveform_512, float pitch_hz);

    /* Re-trigger using the previously-loaded waveform at a (possibly
     * new) pitch.  No-op if no waveform has been loaded yet. */
    void retrigger(float pitch_hz);

    /* Hot-update the per-cycle injection source (_wtOriginal) without
     * retriggering — mirrors a wavetable encoder scrub on hardware.
     * Re-uses the current phys_N so the existing note continues
     * playing with new injection content. */
    void load_wavetable(const float* waveform_512);

    /* ── Parameter setters ─────────────────────────────────────────── */

    void  set_pitch_hz(float hz);          /* recomputes head-pos increment */
    void  set_damping(float v);            /* 0..1 */
    void  set_lpf_cutoff(int v);           /* harmonic number, ~1..42 */
    void  set_noise_level(float v);        /* 0..1 */
    void  set_noise_color(float v);        /* 0..1 */
    void  set_wt_attack(float v);          /* 0..1 */
    void  set_external_env(float v);       /* 0..1 — gates per-cycle injection */

    /* Configure an internal envelope generator that re-ramps
     * externalEnvLevel from 0 → 1 on every trigger over `attack_ms`,
     * then holds at `sustain_level` until the next trigger.  Mimics
     * the firmware's LPG-vactrol / LFO-VCA-envelope behaviour, where
     * externalEnvLevel is NOT static at note start.  Set attack_ms = 0
     * to disable (default). */
    void  set_envelope_ramp(float attack_ms, float sustain_level = 1.0f);

    /* ── Audio rendering ───────────────────────────────────────────── */

    /* Render `n_samples` of audio at kSampleRate into `out`.  Output
     * is the post-mixer signal in the firmware's convention: physics
     * domain ±1.0 (the firmware scales by 32768 right after this loop
     * for downstream int16 math; we leave that to the caller). */
    void  fillBlock(float* out, int n_samples);

    /* ── Introspection (read-only) ────────────────────────────────── */

    const o_halo& state()       const { return rs_; }
    const float*         current_q()   const { return mc_[buffer_sel_]; }
    int                  phys_n()      const { return rs_.phys_N; }
    bool                 back_ready()  const { return rs_.backReady != 0; }
    float                head_pos()    const { return head_pos_; }
    float                head_inc()    const { return head_inc_; }
    int                  buffer_sel()  const { return buffer_sel_; }
    float                pitch_hz()    const { return pitch_hz_; }

    /* Counters useful for tests — incremented inside fillBlock. */
    uint64_t advance_count()           const { return n_advances_; }
    uint64_t wrap_count()              const { return n_wraps_; }

    /* Per-wrap diagnostics.  When recording is enabled, fillBlock pushes
     * a WrapEvent into the vector for every buffer wrap.  Useful for
     * tracking down cycle-boundary discontinuities (the artefacts the
     * user has been hearing on hardware). */
    void  enable_wrap_recording(bool on) { record_wraps_ = on; }
    void  clear_wrap_events()             { wrap_events_.clear(); }
    const std::vector<WrapEvent>& wrap_events() const { return wrap_events_; }

private:
    o_halo rs_;
    float         mc_[2][kMaxBufLen];

    int           buffer_sel_;
    float         head_pos_;
    float         head_inc_;
    float         pitch_hz_;

    /* Trigger-flip crossfade state (mirror of o_wt_osc.xfade_*[chan]). */
    int           xfade_remaining_;
    int           xfade_prev_buffer_;
    float         xfade_prev_head_;
    int           xfade_prev_M_;
    float         xfade_prev_inc_;

    /* Cached float seed so retrigger() / load_wavetable() can refer
     * back to "the most recently loaded waveform". */
    float         seed_wave_[kMaxBufLen];
    bool          has_seed_;

    /* Stats. */
    uint64_t      n_wraps_;
    uint64_t      n_advances_;

    /* Diagnostic recording. */
    bool                    record_wraps_ = false;
    std::vector<WrapEvent>  wrap_events_;
    float                   last_out_sample_ = 0.0f;  /* carry across blocks */

    /* Optional envelope ramp on externalEnvLevel.  When attack_samples_
     * > 0, every trigger resets env_phase_ to 0 and the per-sample
     * driver linearly ramps from 0 to env_sustain_ over
     * attack_samples_ samples, then holds. */
    int                     env_attack_samples_ = 0;
    int                     env_phase_          = 0;
    float                   env_sustain_        = 1.0f;

    /* Helpers. */
    void          recompute_head_inc();
    void          run_advance_in_audio();
    static void   float_to_i16(const float* in, int16_t* out, int n);
};
