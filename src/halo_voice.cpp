/*
 * halo_voice.cpp — see inc/halo_voice.hpp for design.
 *
 * Ported from app/cpp_test/halo_voice_amortized.cpp (the
 * Engine B that the host A/B harness validated against the legacy
 * batch path).  The only adaptations for firmware are:
 *   - Buffer length / sample rate come from globals.h / sphere.h
 *     (numerically identical to the host harness's compile-time
 *     constants, but defined here as one source of truth).
 *   - Seed waveforms arrive from update_oscillators as int16
 *     pairs + frac, so we expose halo_build_seed_wave() to
 *     do the lerp/scale once at the call site.
 *   - Static voice array + extern "C" facade so the C audio ISR
 *     can drive the voices without converting itself to C++.
 */

#include "halo_voice.hpp"

#include <cmath>
#include <cstring>

namespace {

inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi ? hi : x);
}

/* xorshift32 → uniform float in [-1, 1].  Identical to the legacy
 * implementation in src/halo.c. */
inline float xor_rand(uint32_t* s) {
    uint32_t x = *s;
    if (x == 0) x = 0xDEADBEEFu;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return static_cast<float>(static_cast<int32_t>(x)) / 2147483648.0f;
}

} /* namespace */

/* ── Lifecycle ──────────────────────────────────────────────────── */

void HaloVoice::init() {
    std::memset(q_, 0, sizeof(q_));
    std::memset(v_, 0, sizeof(v_));
    std::memset(wt_orig_, 0, sizeof(wt_orig_));
    std::memset(xfade_old_q_, 0, sizeof(xfade_old_q_));

    /* Boot silent.  The legacy halo_init seeded q[] with a
     * full-amplitude sine "so the voice makes a sound from cycle
     * zero before any trigger arrives", but that meant six voices
     * × full-amplitude sines all at the same phase showed up at
     * the codec on the very first audio block when the output mute
     * released — audible as a hard impulse on power-on.  Leaving
     * q[] at zero means each voice stays silent until its first
     * trigger (key press, chord change, jack gate, LFO cycle in
     * LFO-VCA mode).  In normal use the user always provokes a
     * trigger within a few milliseconds of touching the module. */

    M_                 = kBufLen;
    read_head_         = 0.0f;
    pitch_hz_          = 110.0f;
    phys_head_         = 0;
    phys_accum_        = 0.0f;
    dtt_               = kDt;
    inj_amt_           = 0.0f;
    wt_amp_ = noise_amp_ = damp_scale_ = 0.0f;
    alpha_             = 0.0f;
    color_gain_        = 1.0f;
    noise_state_       = 0.0f;
    q_dc_full_ = q_dc_partial_ = 0.0f;
    inj_dc_full_ = inj_dc_partial_ = 0.0f;
    lpf_q1_ = lpf_q2_ = lpf_v1_ = lpf_v2_ = 0.0f;
    lpf_alpha_q_ = lpf_alpha_v_ = 0.0f;
    lpf_dirty_         = true;
    lpf_cutoff_        = 24;
    damping_           = 0.2f;
    noise_level_       = 0.1f;
    noise_color_       = 0.4f;
    wt_attack_         = 0.0f;
    external_env_      = 1.0f;
    rng_state_         = 0x12345678u + (uint32_t)(uintptr_t)this;
    xfade_remaining_   = 0;
    xfade_total_       = kXfadeLen;
    xfade_old_M_       = 0;
    xfade_old_head_    = 0.0f;
    xfade_old_inc_     = 0.0f;
    inj_blend_         = 0.0f;
    attack_env_        = 1.0f;
    attack_env_inc_    = 1.0f;
    std::memset(q_staging_,       0, sizeof(q_staging_));
    std::memset(wt_orig_staging_, 0, sizeof(wt_orig_staging_));
    pending_M_            = 0;
    pending_pitch_hz_     = 0.0f;
    pending_lpf_alpha_q_  = 0.0f;
    pending_lpf_alpha_v_  = 0.0f;
    pending_lpf_q1_       = 0.0f;
    pending_lpf_q2_       = 0.0f;
    pending_commit_       = false;

    recompute_read_inc();
    recompute_lpf_coefs();
}

/* ── Pitch / increment ──────────────────────────────────────────── */

int HaloVoice::phys_n_for_pitch(float hz) const {
    if (!(hz > 0.0f)) return kBufLen;
    int m = (int)((kSampleRate / hz) + 0.5f);
    m = (m / kMstep) * kMstep;
    if (m < kMmin)   m = kMmin;
    if (m > kBufLen) m = kBufLen;
    return m;
}

void HaloVoice::set_pitch_hz(float hz) {
    /* Reject non-finite / pathological inputs. NaN or +Inf would
     * propagate into read_inc_ and into fillBlock's `phys_accum_ +=
     * inc` loop, which would then loop forever (a NaN compares
     * neither >=  nor < 1.0) and stall the audio ISR — exactly the
     * "very high pitches kill audio" failure mode. */
    if (!std::isfinite(hz) || hz <= 0.0f) hz = 0.0001f;
    /* Hard upper bound. The model is undefined above Nyquist anyway;
     * clamping prevents runaway read_inc values from bizarre CV. */
    const float kMaxPitchHz = kSampleRate * 0.5f;     /* 24 kHz */
    if (hz > kMaxPitchHz) hz = kMaxPitchHz;
    pitch_hz_ = hz;
    recompute_read_inc();
}

void HaloVoice::recompute_read_inc() {
    read_inc_ = pitch_hz_ * (float)M_ / kSampleRate;
}

/* ── wtAttack split parameter ───────────────────────────────────
 *
 * Param range is unchanged externally (0..1) but interpreted in
 * two halves:
 *
 *   0.0 .. 0.5  → inj_blend_ = 0..1, attack envelope is OFF
 *                  (instant onset, identical to the old wtAttack 0..1
 *                  blend between initial-seed and per-cycle injection).
 *   0.5 .. 1.0  → inj_blend_ = 1.0  (fully injection-driven), and the
 *                  attack envelope ramps from 0 → 1 over a window
 *                  scaled linearly from 0 s (at param=0.5) up to a
 *                  max of kMaxAttackSecs (at param=1.0).
 *
 * The attack envelope multiplies the per-cycle injection inside
 * step_one_physics_index, so the string excitation swells in
 * gradually instead of being immediately at full level.  attack_env_
 * itself is not reset by parameter changes mid-note; it is only
 * armed (set to 0) inside trigger().  Moving the knob upward during
 * sustain therefore takes effect on the *next* trigger — moving it
 * downward shortens the remaining ramp by raising attack_env_inc_.
 */
void HaloVoice::set_wt_attack(float v) {
    constexpr float kMaxAttackSecs = 2.0f;

    if (!std::isfinite(v)) v = 0.0f;
    v = clampf(v, 0.0f, 1.0f);
    wt_attack_ = v;

    if (v <= 0.5f) {
        /* Lower half: existing blend, no envelope. */
        inj_blend_      = clampf(v * 2.0f, 0.0f, 1.0f);
        attack_env_inc_ = 1.0f;          /* envelope completes in 1 sample */
    } else {
        /* Upper half: fully injection-driven, with attack envelope. */
        inj_blend_       = 1.0f;
        float secs       = (v - 0.5f) * 2.0f * kMaxAttackSecs;
        if (secs <= 1e-6f) {
            attack_env_inc_ = 1.0f;
        } else {
            attack_env_inc_ = 1.0f / (secs * kSampleRate);
        }
    }
}

void HaloVoice::recompute_lpf_coefs() {
    float max_cutoff = (float)(M_ / 2 - 1);
    float cutoff_q = clampf((float)lpf_cutoff_,        1.0f, max_cutoff);
    float cutoff_v = clampf((float)lpf_cutoff_ * 0.5f, 1.0f, max_cutoff);
    lpf_alpha_q_ = std::exp(-2.0f * (float)M_PI * cutoff_q / (float)M_);
    lpf_alpha_v_ = std::exp(-2.0f * (float)M_PI * cutoff_v / (float)M_);
    lpf_dirty_ = false;
}

/* ── Trigger (heavy half) ──────────────────────────────────────── */

void HaloVoice::prepare_trigger(const float* wave_512, float pitch_hz) {
    /* Pick the new pitch-adapted M up front so we know how much of
     * the staging buffers we'll touch. */
    int new_M = phys_n_for_pitch(pitch_hz);

    /* Resample wave_512 → wt_orig_staging_.  We DON'T touch
     * wt_orig_ yet — physics on the still-live OLD voice is
     * reading it in step_one_physics_index() for per-cycle
     * injection, and we don't want to fight that.  The commit
     * path below memcpys the staging buffer into wt_orig_ inside
     * __disable_irq(). */
    const float ratio = (float)kBufLen / (float)new_M;
    for (int n = 0; n < new_M; ++n) {
        float pos = (float)n * ratio;
        int   i0  = (int)pos;
        int   i1  = (i0 + 1) & (kBufLen - 1);
        float t   = pos - (float)i0;
        wt_orig_staging_[n] = wave_512[i0] * (1.0f - t) + wave_512[i1] * t;
    }

    /* Seed q_staging_ from wt_orig_staging_ scaled by (1 - inj_blend),
     * with optional coloured-noise mixed in.  DC removed.  Mirrors
     * the legacy halo_seed_lerp's "Step 2".  inj_blend_ is
     * the lower-half mapping of wt_attack (saturates at 1.0 once
     * param ≥ 0.5), so the upper-half attack-envelope range still
     * starts from a fully empty buffer. */
    float seed_amt = 1.0f - clampf(inj_blend_, 0.0f, 1.0f);
    if (seed_amt > 1e-6f) {
        float n_lvl = clampf(noise_level_, 0.0f, 1.0f);
        float wta   = seed_amt * (1.0f - n_lvl);
        float cp    = clampf(noise_color_, 0.0f, 1.0f);
        float a     = (cp < 0.01f) ? 0.0f
                                   : (1.0f - std::pow(10.0f, -4.0f * cp));
        float cg    = (a < 1e-6f) ? 1.0f
                                   : std::sqrt((1.0f + a) / (1.0f - a));
        float n_amp = seed_amt * n_lvl * cg * 0.5f;
        float state = 0.0f, sum = 0.0f;
        for (int n = 0; n < new_M; ++n) {
            float white = xor_rand(&rng_state_);
            state = a * state + (1.0f - a) * white;
            float inj = wta * wt_orig_staging_[n] + n_amp * state;
            q_staging_[n] = inj;
            sum          += inj;
        }
        if (std::fabs(sum) > 1e-9f) {
            float dc = sum / (float)new_M;
            for (int n = 0; n < new_M; ++n) q_staging_[n] -= dc;
        }
    } else {
        for (int n = 0; n < new_M; ++n) q_staging_[n] = 0.0f;
    }

    /* Compute the LPF coefficients for the new M up front.  We need
     * them both for pre-smoothing here AND so commit_trigger() can
     * latch them without recomputing inside __disable_irq().  These
     * are the same formulas as recompute_lpf_coefs(), specialised
     * for new_M instead of the live M_. */
    {
        float max_cutoff = (float)(new_M / 2 - 1);
        float cutoff_q   = clampf((float)lpf_cutoff_,        1.0f, max_cutoff);
        float cutoff_v   = clampf((float)lpf_cutoff_ * 0.5f, 1.0f, max_cutoff);
        pending_lpf_alpha_q_ = std::exp(-2.0f * (float)M_PI * cutoff_q / (float)new_M);
        pending_lpf_alpha_v_ = std::exp(-2.0f * (float)M_PI * cutoff_v / (float)new_M);
    }

    /* Pre-smooth q_staging_ to approximate one full cycle of physics
     * LPF.  Without this the audio ISR reads the raw, un-smoothed
     * seed for the first few cycles after a trigger, which is
     * dominated by the coloured-noise term — its amplitude can be
     * several × wt_orig because color_gain = √((1+a)/(1-a)) blows up
     * at higher noise_color values.  Two warm-up passes (IIR
     * convergence on a circular buffer) followed by one committing
     * pass mirrors the 2-pole streaming LPF inside
     * step_one_physics_index().
     *
     * COST: ~125 µs across 6 voices at M≈300.  This is the single
     * most expensive piece of the trigger path, which is why we run
     * it here in prepare_trigger() (audio interrupts ENABLED) instead
     * of inside the commit's __disable_irq() window — running it in
     * an IRQ-blocked window blew the 500 µs audio block budget on
     * simultaneous chord retriggers and produced a "burst of noise"
     * click from SAI DMA underrun (independent of any actual noise
     * level — it's a downstream symptom of the audio ISR being
     * delayed past its block deadline). */
    if (new_M > 0) {
        float a  = pending_lpf_alpha_q_;
        float b  = 1.0f - a;
        float s1 = q_staging_[new_M - 1];
        float s2 = s1;
        for (int pass = 0; pass < 2; ++pass) {
            for (int n = 0; n < new_M; ++n) {
                float x = q_staging_[n];
                s1 = a * s1 + b * x;
                s2 = a * s2 + b * s1;
            }
        }
        for (int n = 0; n < new_M; ++n) {
            float x = q_staging_[n];
            s1 = a * s1 + b * x;
            s2 = a * s2 + b * s1;
            q_staging_[n] = s2;
        }
        pending_lpf_q1_ = s1;
        pending_lpf_q2_ = s2;
    } else {
        pending_lpf_q1_ = pending_lpf_q2_ = 0.0f;
    }

    pending_M_         = new_M;
    pending_pitch_hz_  = pitch_hz;
    pending_commit_    = true;
}

/* ── Trigger (light half) ──────────────────────────────────────── */

void HaloVoice::commit_trigger() {
    if (!pending_commit_) return;

    /* Snapshot the dying buffer + read state for the trigger
     * crossfade BEFORE we overwrite q[].
     *
     * Crossfade duration: defaults to kXfadeLen (~10.7 ms at 48 kHz)
     * for instant-attack triggers.  When the slow-attack path is
     * engaged (wt_attack > 0.5, attack_env_inc_ < 1.0) the new voice
     * needs up to kMaxAttackSecs (2 s) to swell to full level, so a
     * 10 ms fade-out of the dying snapshot would leave a near-silent
     * gap that the listener perceives as "the previous note got cut
     * off".  Stretch the fade to match the attack window so the old
     * snapshot recedes linearly over the same duration the new voice
     * is building up.  The snapshot loops as a frozen waveform during
     * the long fade — that's not perfect (it doesn't continue
     * decaying) but the linear w_old → 0 ramp keeps it inaudible by
     * the time the new voice dominates. */
    int xf_len = kXfadeLen;
    if (attack_env_inc_ > 0.0f && attack_env_inc_ < 1.0f) {
        int attack_samples = (int)(1.0f / attack_env_inc_);
        if (attack_samples > xf_len) xf_len = attack_samples;
    }
    if (M_ > 0 && M_ <= kBufLen) {
        std::memcpy(xfade_old_q_, q_, (size_t)M_ * sizeof(float));
        xfade_old_M_     = M_;
        xfade_old_head_  = read_head_;
        xfade_old_inc_   = read_inc_;
        xfade_total_     = xf_len;
        xfade_remaining_ = xf_len;
    }

    /* NOTE: there is no separate amplitude ramp-in.  The earlier
     * `output × smoothstep(t)` ramp turned out to be the very click
     * we were trying to hide — multiplying the live output by zero
     * at sample 0 forces a step from the previous voice's level to
     * silence, which is exactly the "abrupt cut" that becomes
     * audible when six voices retrigger together.  The buffer
     * crossfade above (with equal-power weights) handles both the
     * fade-out of the dying note and the fade-in of the new seed
     * without any zero-multiply discontinuity. */

    /* Arm the (slow) wtAttack envelope.  When the upper half of the
     * wt_attack parameter is engaged (>0.5) attack_env_inc_ is small
     * enough that this ramp takes up to kMaxAttackSecs to reach 1.0,
     * producing a swelling/bowed onset.  Below 0.5 the increment is
     * 1.0 and the envelope hits full on the very first audio sample,
     * preserving today's instant-attack behaviour. */
    attack_env_ = 0.0f;

    /* Bulk-copy staging into live.  These memcpys are the bulk of
     * the disable-IRQ cost (~5 µs/voice with cache, 30 µs total for
     * 6 voices).  Cheap compared to the 280 µs the old single-call
     * trigger() spent inside __disable_irq(). */
    int new_M = pending_M_;
    if (new_M > 0 && new_M <= kBufLen) {
        std::memcpy(q_,        q_staging_,        (size_t)new_M * sizeof(float));
        std::memcpy(wt_orig_,  wt_orig_staging_,  (size_t)new_M * sizeof(float));
    }

    /* Reset velocity, streaming state, LPF state, head. */
    std::memset(v_, 0, sizeof(v_));
    M_              = new_M;
    pitch_hz_       = pending_pitch_hz_;
    read_head_      = 0.0f;
    phys_head_      = 0;
    phys_accum_     = 0.0f;
    q_dc_full_      = 0.0f;
    q_dc_partial_   = 0.0f;
    inj_dc_full_    = 0.0f;
    inj_dc_partial_ = 0.0f;

    /* Latch the LPF coefficients we computed in prepare_trigger().
     * Avoids recomputing exp() inside the disable-IRQ window. */
    lpf_alpha_q_ = pending_lpf_alpha_q_;
    lpf_alpha_v_ = pending_lpf_alpha_v_;
    lpf_q1_      = pending_lpf_q1_;
    lpf_q2_      = pending_lpf_q2_;
    lpf_v1_ = lpf_v2_ = 0.0f;
    lpf_dirty_   = false;

    recompute_read_inc();
    start_new_phys_cycle();

    pending_commit_ = false;
}

/* ── Combined single-call trigger (test/legacy path) ───────────── */

void HaloVoice::trigger(const float* wave_512, float pitch_hz) {
    prepare_trigger(wave_512, pitch_hz);
    commit_trigger();
}

void HaloVoice::load_wavetable(const float* wave_512) {
    /* Live refresh of the per-cycle injection source — does NOT
     * touch q[], v[], or filter state.  Matches the legacy
     * halo_refresh_wt_original semantics. */
    const float ratio = (float)kBufLen / (float)M_;
    for (int n = 0; n < M_; ++n) {
        float pos = (float)n * ratio;
        int   i0  = (int)pos;
        int   i1  = (i0 + 1) & (kBufLen - 1);
        float t   = pos - (float)i0;
        wt_orig_[n] = wave_512[i0] * (1.0f - t) + wave_512[i1] * t;
    }
}

/* ── Per-cycle physics setup ───────────────────────────────────── */

void HaloVoice::start_new_phys_cycle() {
    dtt_ = clampf(kDt + q_[0] * kDt * 0.5f, 0.005f, 0.05f);

    float lvl = clampf(external_env_, 0.0f, 1.0f);
    inj_amt_  = clampf(inj_blend_, 0.0f, 1.0f) * lvl * kDt;
    if (inj_amt_ > 1e-9f) {
        float n_lvl = clampf(noise_level_, 0.0f, 1.0f);
        wt_amp_     = inj_amt_ * (1.0f - n_lvl);
        float cp    = clampf(noise_color_, 0.0f, 1.0f);
        alpha_      = (cp < 0.01f) ? 0.0f
                                   : (1.0f - std::pow(10.0f, -4.0f * cp));
        color_gain_ = (alpha_ < 1e-6f) ? 1.0f
                                       : std::sqrt((1.0f + alpha_) /
                                                   (1.0f - alpha_));
        noise_amp_  = inj_amt_ * n_lvl * color_gain_ * 0.5f;
        /* damp_scale_ multiplies the per-cycle injection into v[].
         * The JS reference uses (damping × 10), but JS clamps damping
         * to a minimum of 0.01 — at damping = 0 the injection
         * collapses to zero and the string never gets re-excited, so
         * the audible energy decays away over a few seconds and the
         * voice goes silent.  Floor the scale at 1.0 here so the
         * "depth" knob at zero still produces sound; damping then
         * controls the LPF mix only (the dry/wet branch in
         * step_one_physics_index() still bypasses the filter when
         * damping == 0 so the user gets clean output). */
        damp_scale_ = damping_ * 10.0f;
        if (damp_scale_ < 1.0f) damp_scale_ = 1.0f;
    } else {
        wt_amp_ = noise_amp_ = damp_scale_ = 0.0f;
    }

    noise_state_ = 0.0f;

    float invM       = 1.0f / (float)M_;
    q_dc_full_       = q_dc_partial_   * invM;
    inj_dc_full_     = inj_dc_partial_ * invM;
    q_dc_partial_    = 0.0f;
    inj_dc_partial_  = 0.0f;

    if (lpf_dirty_) recompute_lpf_coefs();
}

/* ── Single physics index update ───────────────────────────────── */

void HaloVoice::step_one_physics_index() {
    int n  = phys_head_;
    int nR = n + (M_ >> 1);
    if (nR >= M_) nR -= M_;

    /* 1. Per-cycle injection into v[n] (one-cycle DC tracking).
     *    `attack_env_` (advanced per audio sample in fillBlock())
     *    multiplies the injection so that the upper half of the
     *    wtAttack parameter produces a slow swelling onset rather
     *    than an immediate full-strength excitation. */
    if (inj_amt_ > 1e-9f) {
        float white = xor_rand(&rng_state_);
        noise_state_ = alpha_ * noise_state_ + (1.0f - alpha_) * white;
        float inj = (wt_amp_ * wt_orig_[n] + noise_amp_ * noise_state_)
                  * attack_env_;
        v_[n] += inj * damp_scale_;
        inj_dc_partial_ += inj;
        v_[n] -= inj_dc_full_ * damp_scale_;
    }

    /* 2. Antipodal coupling. */
    float qn  = q_[n];
    float qnR = q_[nR];
    float d   = qnR - qn;
    float f   = d * kNonlin;
    v_[n] += f * dtt_ - qn * dtt_ * 0.1f;

    /* 3. Velocity soft-clip. */
    float vn = v_[n];
    if (vn >  1.0f) vn =  1.0f;
    if (vn < -1.0f) vn = -1.0f;
    if (std::fabs(vn) > 0.5f) vn *= 0.99f;
    v_[n] = vn;

    /* 4. Position update. */
    q_[n] = qn + dtt_ * vn;

    /* 5. DC removal on q (one-cycle latency). */
    q_[n] -= q_dc_full_;
    q_dc_partial_ += q_[n];

    /* 6. Safety clamp on q. */
    if (q_[n] >  2.0f) q_[n] =  2.0f;
    if (q_[n] < -2.0f) q_[n] = -2.0f;

    /* 7. Streaming LPF on q. */
    {
        float mix = damping_;
        if (mix > 0.0f) {
            float a   = lpf_alpha_q_;
            float b   = 1.0f - a;
            float dry = 1.0f - mix;
            float x   = q_[n];
            lpf_q1_   = a * lpf_q1_ + b * x;
            lpf_q2_   = a * lpf_q2_ + b * lpf_q1_;
            q_[n]     = dry * x + mix * lpf_q2_;
        } else {
            lpf_q1_ = q_[n];
            lpf_q2_ = q_[n];
        }
    }

    /* 8. Streaming LPF on v. */
    {
        float mix = damping_;
        if (mix > 0.0f) {
            float a   = lpf_alpha_v_;
            float b   = 1.0f - a;
            float dry = 1.0f - mix;
            float x   = v_[n];
            lpf_v1_   = a * lpf_v1_ + b * x;
            lpf_v2_   = a * lpf_v2_ + b * lpf_v1_;
            v_[n]     = dry * x + mix * lpf_v2_;
        } else {
            lpf_v1_ = v_[n];
            lpf_v2_ = v_[n];
        }
    }

    /* Advance head; new cycle setup on wrap. */
    phys_head_++;
    if (phys_head_ >= M_) {
        phys_head_ = 0;
        start_new_phys_cycle();
    }
}

/* ── Resonator-mode injection ──────────────────────────────────── */

void HaloVoice::inject_velocity(int idx, float amount) {
    if (idx < 0 || idx >= M_) return;
    v_[idx] += amount;
}

/* ── Audio block render ───────────────────────────────────────── */

void HaloVoice::fillBlock(float* out, int n_samples,
                                const float* audio_in) {
    const float fM = (float)M_;
    /* Cap physics rate at one step per audio sample.  At low pitches
     * (read_inc_ < 1) this is a no-op and physics still runs exactly
     * "one cycle per audio cycle" — matching the legacy behaviour.
     * At high pitches (read_inc_ > 1) we'd otherwise do read_inc_
     * physics steps per audio sample, which scales CPU linearly with
     * pitch and blows the audio block budget around 8 kHz with all
     * six voices.  The cap pins per-sample physics CPU to a constant
     * (≈ 6 × kSampleRate steps/s ≈ 1.4 % of CPU) at the cost of the
     * physics "evolving" at a fixed M/Fs Hz rate above the cap.
     * Audio still scans the buffer at the correct pitch — it just
     * reads a snapshot of more slowly-evolving physics. */
    float steps_per_sample = read_inc_;
    if (steps_per_sample > 1.0f) steps_per_sample = 1.0f;
    /* xfade_total_ is set per-trigger in commit_trigger (defaults to
     * kXfadeLen; longer when the slow-attack path is active). */
    const int   xf_total = (xfade_total_ > 0) ? xfade_total_ : kXfadeLen;
    const float inv_xf   = 1.0f / (float)xf_total;

    for (int i = 0; i < n_samples; ++i) {
        /* Advance the wtAttack envelope at audio rate so the slow-
         * onset window timing is independent of pitch / physics
         * step rate.  Saturates at 1.0 once the configured attack
         * time has elapsed. */
        if (attack_env_ < 1.0f) {
            attack_env_ += attack_env_inc_;
            if (attack_env_ > 1.0f) attack_env_ = 1.0f;
        }

        /* Audio: advance read head. */
        read_head_ += read_inc_;
        if (read_head_ >= fM) {
            while (read_head_ >= fM) read_head_ -= fM;
            if (!std::isfinite(read_head_)) read_head_ = 0.0f;
        }
        int   r0 = (int)read_head_;
        if (r0 < 0 || r0 >= M_) r0 = 0;
        int   r1 = r0 + 1;
        if (r1 >= M_) r1 = 0;
        float rd = read_head_ - (float)r0;

        /* Resonator mode: inject external audio into v[] at the
         * audio read position BEFORE physics runs at this index. */
        if (audio_in) {
            float amt = audio_in[i] * 0.5f;
            v_[r0] += amt * (1.0f - rd);
            v_[r1] += amt * rd;
        }

        float raw = q_[r0] * (1.0f - rd) + q_[r1] * rd;
        if (!std::isfinite(raw)) raw = 0.0f;

        /* Trigger crossfade against the dying buffer.  Linear blend:
         * w_new = t,  w_old = 1 - t,  with t = (kXfadeLen -
         * xfade_remaining + 1) · inv_xf in (0, 1].  A linear
         * crossfade dips by ~3 dB at the midpoint for uncorrelated
         * signals, but the previous equal-power form (cos/sin)
         * required two software trig calls per sample (~200 cycles)
         * which — across 6 voices × 24 samples × ~21 blocks of
         * crossfade — pushed the audio ISR past its 500 µs block
         * budget, audibly underrunning SAI DMA and stalling the
         * same-priority PWM_OUTS_TIM ISR enough to flicker the
         * envelope LEDs.  Linear is ~5 cycles per sample, so the
         * crossfade work is now negligible against the rest of the
         * audio path. */
        if (xfade_remaining_ > 0 && xfade_old_M_ > 0) {
            xfade_old_head_ += xfade_old_inc_;
            float old_fM = (float)xfade_old_M_;
            if (xfade_old_head_ >= old_fM) {
                while (xfade_old_head_ >= old_fM)
                    xfade_old_head_ -= old_fM;
                if (!std::isfinite(xfade_old_head_)) xfade_old_head_ = 0.0f;
            }
            int   o0 = (int)xfade_old_head_;
            if (o0 < 0 || o0 >= xfade_old_M_) o0 = 0;
            int   o1 = o0 + 1;
            if (o1 >= xfade_old_M_) o1 = 0;
            float od = xfade_old_head_ - (float)o0;
            float old_raw = xfade_old_q_[o0] * (1.0f - od) +
                            xfade_old_q_[o1] * od;

            float w_new = (float)(xf_total - xfade_remaining_ + 1) * inv_xf;
            if (w_new < 0.0f) w_new = 0.0f;
            if (w_new > 1.0f) w_new = 1.0f;
            float w_old = 1.0f - w_new;
            raw = old_raw * w_old + raw * w_new;
            xfade_remaining_--;
        }

        out[i] = raw;

        /* Physics: do `steps_per_sample` work units (fractional). */
        phys_accum_ += steps_per_sample;
        while (phys_accum_ >= 1.0f) {
            step_one_physics_index();
            phys_accum_ -= 1.0f;
        }
    }
}

/* ─────────────────────────────────────────────────────────────────
 * C facade — one static voice per channel, dispatched by index.
 * ─────────────────────────────────────────────────────────────── */

/* Voice array lives in SRAM1 — at 4 × 512 × 4 = 8 KB per voice and
 * NUM_CHANNELS = 6 that's 48 KB, way too big for DTCM (which is also
 * crowded with the codec / DMA / Plaits state).  SRAM1 is plenty
 * large and the latency penalty is negligible: each fillBlock makes
 * sequential strided accesses through q_/v_/wt_orig_, so the cache
 * + prefetcher hide the SRAM1 latency completely. */
namespace {

SRAM1DATA HaloVoice g_voices[NUM_CHANNELS];

inline HaloVoice* voice_at(uint8_t chan) {
    return (chan < NUM_CHANNELS) ? &g_voices[chan] : nullptr;
}

} /* namespace */

extern "C" {

void halo_init_all(void) {
    for (uint8_t c = 0; c < NUM_CHANNELS; ++c)
        g_voices[c].init();
}

void halo_trigger(uint8_t chan, const float* wave_512, float pitch_hz) {
    if (auto* v = voice_at(chan)) v->trigger(wave_512, pitch_hz);
}

void halo_prepare_trigger(uint8_t chan, const float* wave_512, float pitch_hz) {
    if (auto* v = voice_at(chan)) v->prepare_trigger(wave_512, pitch_hz);
}

void halo_commit_trigger(uint8_t chan) {
    if (auto* v = voice_at(chan)) v->commit_trigger();
}

void halo_load_wavetable(uint8_t chan, const float* wave_512) {
    if (auto* v = voice_at(chan)) v->load_wavetable(wave_512);
}

void halo_set_pitch_hz(uint8_t chan, float hz)        { if (auto* v = voice_at(chan)) v->set_pitch_hz(hz); }
void halo_set_damping(uint8_t chan, float val)        { if (auto* v = voice_at(chan)) v->set_damping(val); }
void halo_set_lpf_cutoff(uint8_t chan, int32_t val)   { if (auto* v = voice_at(chan)) v->set_lpf_cutoff((int)val); }
void halo_set_noise_level(uint8_t chan, float val)    { if (auto* v = voice_at(chan)) v->set_noise_level(val); }
void halo_set_noise_color(uint8_t chan, float val)    { if (auto* v = voice_at(chan)) v->set_noise_color(val); }
void halo_set_wt_attack(uint8_t chan, float val)      { if (auto* v = voice_at(chan)) v->set_wt_attack(val); }
void halo_set_external_env(uint8_t chan, float val)   { if (auto* v = voice_at(chan)) v->set_external_env(val); }

void halo_inject_velocity(uint8_t chan, int32_t idx, float amount) {
    if (auto* v = voice_at(chan)) v->inject_velocity((int)idx, amount);
}

void halo_fill_block(uint8_t chan, float* out, int32_t n_samples,
                           const float* audio_in) {
    if (auto* v = voice_at(chan)) v->fillBlock(out, (int)n_samples, audio_in);
    else if (out) std::memset(out, 0, (size_t)n_samples * sizeof(float));
}

int32_t halo_phys_n(uint8_t chan) {
    auto* v = voice_at(chan);
    return v ? (int32_t)v->phys_n() : (int32_t)WT_TABLELEN;
}

void halo_build_seed_wave(const int16_t* wave_a,
                                const int16_t* wave_b,
                                float frac,
                                float* out_512) {
    /* Linearly blend two int16 waveforms into a normalised float[]
     * suitable for halo_trigger().  NULL endpoints contribute
     * silence; both NULL → all zeros. */
    if (!out_512) return;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    const float inv_scale = 1.0f / 32768.0f;

    if (wave_a && wave_b) {
        for (int n = 0; n < (int)WT_TABLELEN; ++n) {
            float a = (float)wave_a[n];
            float b = (float)wave_b[n];
            out_512[n] = (a * (1.0f - frac) + b * frac) * inv_scale;
        }
    } else if (wave_a) {
        for (int n = 0; n < (int)WT_TABLELEN; ++n)
            out_512[n] = (float)wave_a[n] * (1.0f - frac) * inv_scale;
    } else if (wave_b) {
        for (int n = 0; n < (int)WT_TABLELEN; ++n)
            out_512[n] = (float)wave_b[n] * frac * inv_scale;
    } else {
        std::memset(out_512, 0, sizeof(float) * (size_t)WT_TABLELEN);
    }
}

} /* extern "C" */
