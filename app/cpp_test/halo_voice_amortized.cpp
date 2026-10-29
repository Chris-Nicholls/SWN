/*
 * halo_voice_amortized.cpp — see header for design.
 */
#include "halo_voice_amortized.hpp"

#include <cmath>
#include <cstring>
#include <algorithm>

namespace {

inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi ? hi : x);
}

/* xorshift32, matches firmware. */
inline float xor_rand(uint32_t* s) {
    uint32_t x = *s;
    if (x == 0) x = 0xDEADBEEFu;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return (float)(int32_t)x / 2147483648.0f;
}

} /* namespace */

/* ── Construction ─────────────────────────────────────────────────── */

HaloVoiceAmortized::HaloVoiceAmortized() {
    std::memset(q_, 0, sizeof(q_));
    std::memset(v_, 0, sizeof(v_));
    std::memset(wt_orig_, 0, sizeof(wt_orig_));
    /* Initialise q to a sine wave so the first cycle has something
     * audible before any trigger.  Mirrors halo_init. */
    for (int n = 0; n < kMaxBufLen; ++n) {
        q_[n] = std::sin(2.0f * (float)M_PI * (float)n / (float)kMaxBufLen);
    }
    rng_state_ = 0x12345678u + (uint32_t)(uintptr_t)this;
    recompute_read_inc();
    recompute_lpf_coefs();
}

/* ── Pitch-adapted M (matches firmware halo_phys_n_for_pitch) ─ */

int HaloVoiceAmortized::phys_n_for_pitch(float hz) const {
    if (!(hz > 0.0f)) return kMaxBufLen;
    int m = (int)((kSampleRate / hz) + 0.5f);
    m = (m / kMstep) * kMstep;
    if (m < kMmin)      m = kMmin;
    if (m > kMaxBufLen) m = kMaxBufLen;
    return m;
}

void HaloVoiceAmortized::set_pitch_hz(float hz) {
    if (hz <= 0.0f) hz = 0.0001f;
    pitch_hz_ = hz;
    recompute_read_inc();
}

void HaloVoiceAmortized::recompute_read_inc() {
    read_inc_ = pitch_hz_ * (float)M_ / kSampleRate;
}

void HaloVoiceAmortized::recompute_lpf_coefs() {
    float cutoff_q = clampf((float)lpf_cutoff_,         1.0f, (float)(M_/2 - 1));
    float cutoff_v = clampf((float)lpf_cutoff_ * 0.5f,  1.0f, (float)(M_/2 - 1));
    lpf_alpha_q_ = std::exp(-2.0f * (float)M_PI * cutoff_q / (float)M_);
    lpf_alpha_v_ = std::exp(-2.0f * (float)M_PI * cutoff_v / (float)M_);
    lpf_dirty_ = false;
}

void HaloVoiceAmortized::set_envelope_ramp(float attack_ms, float sustain_level) {
    env_attack_samples_ = (int)(attack_ms * 0.001f * kSampleRate);
    if (env_attack_samples_ < 0) env_attack_samples_ = 0;
    env_sustain_        = sustain_level;
    env_phase_          = env_attack_samples_;   /* held at sustain by default */
}

/* ── Trigger paths ────────────────────────────────────────────────── */

void HaloVoiceAmortized::trigger(const float* waveform_512, float pitch_hz) {
    /* Snapshot the dying buffer + read state for the trigger
     * crossfade BEFORE we overwrite q[]. */
    if (M_ > 0 && M_ <= kMaxBufLen) {
        std::memcpy(xfade_old_q_, q_, (size_t)M_ * sizeof(float));
        xfade_old_M_     = M_;
        xfade_old_head_  = read_head_;
        xfade_old_inc_   = read_inc_;
        xfade_remaining_ = kXfadeLen;
    }

    /* Pick new M for the new pitch. */
    int new_M = phys_n_for_pitch(pitch_hz);

    /* Resample 512 → new_M with linear interp. */
    const float ratio = (float)kMaxBufLen / (float)new_M;
    for (int n = 0; n < new_M; ++n) {
        float pos = (float)n * ratio;
        int   i0  = (int)pos;
        int   i1  = (i0 + 1) & (kMaxBufLen - 1);
        float t   = pos - (float)i0;
        wt_orig_[n] = waveform_512[i0] * (1.0f - t) + waveform_512[i1] * t;
    }

    /* Seed q from the (already-resampled) wavetable, scaled by
     * (1 - wtAttack) — same logic as halo_seed_lerp. */
    float seed_amt = 1.0f - clampf(wt_attack_, 0.0f, 1.0f);
    if (seed_amt > 1e-6f) {
        float n_lvl = clampf(noise_level_, 0.0f, 1.0f);
        float wta   = seed_amt * (1.0f - n_lvl);
        float cp    = clampf(noise_color_, 0.0f, 1.0f);
        float a     = (cp < 0.01f) ? 0.0f : (1.0f - std::pow(10.0f, -4.0f * cp));
        float cg    = (a < 1e-6f) ? 1.0f : std::sqrt((1.0f + a) / (1.0f - a));
        float n_amp = seed_amt * n_lvl * cg * 0.5f;
        float state = 0.0f, sum = 0.0f;
        for (int n = 0; n < new_M; ++n) {
            float white = xor_rand(&rng_state_);
            state = a * state + (1.0f - a) * white;
            float inj = wta * wt_orig_[n] + n_amp * state;
            q_[n]  = inj;
            sum   += inj;
        }
        if (std::fabs(sum) > 1e-9f) {
            float dc = sum / (float)new_M;
            for (int n = 0; n < new_M; ++n) q_[n] -= dc;
        }
    } else {
        for (int n = 0; n < new_M; ++n) q_[n] = 0.0f;
    }

    /* Reset velocity, LPF state, and physics-cycle state. */
    std::memset(v_, 0, sizeof(v_));
    M_              = new_M;
    pitch_hz_       = pitch_hz;
    read_head_      = 0.0f;
    phys_head_      = 0;
    phys_accum_     = 0.0f;
    q_dc_full_      = 0.0f;
    q_dc_partial_   = 0.0f;
    inj_dc_full_    = 0.0f;
    inj_dc_partial_ = 0.0f;
    /* Seed the LPF state with the buffer's last sample so the first
     * processed sample picks up smoothly. */
    lpf_q1_ = (M_ > 0) ? q_[M_ - 1] : 0.0f;
    lpf_q2_ = lpf_q1_;
    lpf_v1_ = lpf_v2_ = 0.0f;
    recompute_read_inc();
    recompute_lpf_coefs();
    start_new_phys_cycle();

    if (env_attack_samples_ > 0) {
        env_phase_     = 0;
        external_env_  = 0.0f;
    }
}

void HaloVoiceAmortized::retrigger(float pitch_hz) {
    /* Re-trigger using the cached wavetable in wt_orig_ (no need to
     * stash the pre-resample float[512] — wt_orig_ is the resampled
     * result we'd seed from anyway). */
    /* Synthesize a 512-pt waveform from wt_orig_ for re-trigger;
     * easiest path is to reverse-resample, but we can equivalently
     * just call trigger() with the current wt_orig_ stretched back to
     * 512. */
    float wave[kMaxBufLen];
    const float ratio = (float)M_ / (float)kMaxBufLen;
    for (int n = 0; n < kMaxBufLen; ++n) {
        float pos = (float)n * ratio;
        int   i0  = (int)pos;
        int   i1  = (i0 + 1) % M_;
        float t   = pos - (float)i0;
        wave[n] = wt_orig_[i0] * (1.0f - t) + wt_orig_[i1] * t;
    }
    trigger(wave, pitch_hz);
}

void HaloVoiceAmortized::load_wavetable(const float* waveform_512) {
    /* Resample into wt_orig_ at current M; do not touch q[], v[], or
     * envelope state (mirrors halo_refresh_wt_original). */
    const float ratio = (float)kMaxBufLen / (float)M_;
    for (int n = 0; n < M_; ++n) {
        float pos = (float)n * ratio;
        int   i0  = (int)pos;
        int   i1  = (i0 + 1) & (kMaxBufLen - 1);
        float t   = pos - (float)i0;
        wt_orig_[n] = waveform_512[i0] * (1.0f - t) + waveform_512[i1] * t;
    }
}

/* ── Per-cycle setup ──────────────────────────────────────────────── */

void HaloVoiceAmortized::start_new_phys_cycle() {
    /* dtt depends on q[0]; compute now (q[0] reflects the freshly-
     * processed value from the cycle just completed). */
    dtt_ = clampf(kDt + q_[0] * kDt * 0.5f, 0.005f, 0.05f);

    /* Injection coefficients. */
    float lvl = clampf(external_env_, 0.0f, 1.0f);
    inj_amt_  = clampf(wt_attack_, 0.0f, 1.0f) * lvl * kDt;
    if (inj_amt_ > 1e-9f) {
        float n_lvl = clampf(noise_level_, 0.0f, 1.0f);
        wt_amp_     = inj_amt_ * (1.0f - n_lvl);
        float cp    = clampf(noise_color_, 0.0f, 1.0f);
        alpha_      = (cp < 0.01f) ? 0.0f
                      : (1.0f - std::pow(10.0f, -4.0f * cp));
        color_gain_ = (alpha_ < 1e-6f) ? 1.0f
                      : std::sqrt((1.0f + alpha_) / (1.0f - alpha_));
        noise_amp_  = inj_amt_ * n_lvl * color_gain_ * 0.5f;
        damp_scale_ = damping_ * 10.0f;
    } else {
        wt_amp_ = noise_amp_ = damp_scale_ = 0.0f;
    }

    /* Coloured-noise filter restarts each cycle (matches firmware/JS). */
    noise_state_ = 0.0f;

    /* Promote last cycle's running sums to "the mean to subtract this
     * cycle".  Divide by M to get the per-sample correction. */
    float invM    = 1.0f / (float)M_;
    q_dc_full_    = q_dc_partial_   * invM;
    inj_dc_full_  = inj_dc_partial_ * invM;
    q_dc_partial_   = 0.0f;
    inj_dc_partial_ = 0.0f;

    if (lpf_dirty_) recompute_lpf_coefs();
}

/* ── Single physics step at phys_head_ ────────────────────────────── */

void HaloVoiceAmortized::step_one_physics_index() {
    int n  = phys_head_;
    int nR = n + (M_ >> 1);
    if (nR >= M_) nR -= M_;

    /* 1. Per-cycle injection into v[n].  Track running sum so the
     *    NEXT cycle can subtract this cycle's mean (one-cycle DC
     *    tracking lag). */
    float inj = 0.0f;
    if (inj_amt_ > 1e-9f) {
        float white = xor_rand(&rng_state_);
        noise_state_ = alpha_ * noise_state_ + (1.0f - alpha_) * white;
        inj = wt_amp_ * wt_orig_[n] + noise_amp_ * noise_state_;
        v_[n] += inj * damp_scale_;
        inj_dc_partial_ += inj;
        /* Subtract last cycle's injection mean (constant per cycle). */
        v_[n] -= inj_dc_full_ * damp_scale_;
    }

    /* 2. Antipodal coupling.  q[nR] may have been advanced by this
     *    cycle's pass already (if nR < n) or still hold last cycle's
     *    value (if nR > n).  Either is fine — small phase glide is
     *    inaudible and the user gave explicit go-ahead for stale
     *    coupling reads. */
    float qn   = q_[n];
    float qnR  = q_[nR];
    float d    = qnR - qn;
    float f    = d * kNonlin;
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

    /* 7. Streaming LPF on q (cascaded 2-pole, state preserved across
     *    samples — naturally continuous everywhere). */
    {
        float a = lpf_alpha_q_, b = 1.0f - a, mix = damping_;
        if (mix > 0.0f) {
            float dry = 1.0f - mix;
            float x   = q_[n];
            lpf_q1_   = a * lpf_q1_ + b * x;
            lpf_q2_   = a * lpf_q2_ + b * lpf_q1_;
            q_[n]     = dry * x + mix * lpf_q2_;
        } else {
            /* Drag state along even when bypassed so future damping
             * sweeps start cleanly. */
            lpf_q1_ = lpf_q2_ = q_[n];
        }
    }

    /* 8. Streaming LPF on v. */
    {
        float a = lpf_alpha_v_, b = 1.0f - a, mix = damping_;
        if (mix > 0.0f) {
            float dry = 1.0f - mix;
            float x   = v_[n];
            lpf_v1_   = a * lpf_v1_ + b * x;
            lpf_v2_   = a * lpf_v2_ + b * lpf_v1_;
            v_[n]     = dry * x + mix * lpf_v2_;
        } else {
            lpf_v1_ = lpf_v2_ = v_[n];
        }
    }

    /* Advance phys_head; on wrap, refresh per-cycle state. */
    phys_head_++;
    n_phys_steps_++;
    if (phys_head_ >= M_) {
        phys_head_ = 0;
        n_wraps_++;
        start_new_phys_cycle();
    }
}

/* ── Audio block render ──────────────────────────────────────────── */

void HaloVoiceAmortized::fillBlock(float* out, int n_samples) {
    const float fM = (float)M_;
    /* Physics steps per audio sample, on average.  One full physics
     * cycle spans M indices and one full audio cycle = M/inc samples,
     * so steps_per_audio_sample = inc.  Use a fractional accumulator
     * to handle non-integer rates without long-term drift. */
    const float steps_per_sample = read_inc_;

    const float inv_xf = 1.0f / (float)kXfadeLen;

    for (int i = 0; i < n_samples; ++i) {
        /* Optional envelope ramp on external_env_. */
        if (env_attack_samples_ > 0) {
            if (env_phase_ < env_attack_samples_) {
                external_env_ = env_sustain_ *
                                (float)env_phase_ /
                                (float)env_attack_samples_;
                env_phase_++;
            } else {
                external_env_ = env_sustain_;
            }
        }

        /* Audio: advance read head, interpolate q[]. */
        read_head_ += read_inc_;
        if (read_head_ >= fM) {
            while (read_head_ >= fM) read_head_ -= fM;
            if (!std::isfinite(read_head_)) read_head_ = 0.0f;
        }
        int   r0  = (int)read_head_;
        if (r0 < 0 || r0 >= M_) r0 = 0;
        int   r1  = r0 + 1;
        if (r1 >= M_) r1 = 0;
        float rd  = read_head_ - (float)r0;
        float raw = q_[r0] * (1.0f - rd) + q_[r1] * rd;
        if (!std::isfinite(raw)) raw = 0.0f;

        /* Trigger crossfade.  Read the OLD buffer at its own continuing
         * head and linearly blend toward the new voice. */
        if (xfade_remaining_ > 0 && xfade_old_M_ > 0) {
            xfade_old_head_ += xfade_old_inc_;
            float old_fM = (float)xfade_old_M_;
            if (xfade_old_head_ >= old_fM) {
                while (xfade_old_head_ >= old_fM) xfade_old_head_ -= old_fM;
                if (!std::isfinite(xfade_old_head_)) xfade_old_head_ = 0.0f;
            }
            int   o0 = (int)xfade_old_head_;
            if (o0 < 0 || o0 >= xfade_old_M_) o0 = 0;
            int   o1 = o0 + 1;
            if (o1 >= xfade_old_M_) o1 = 0;
            float od = xfade_old_head_ - (float)o0;
            float old_raw = xfade_old_q_[o0] * (1.0f - od) +
                            xfade_old_q_[o1] * od;
            float t = (float)(kXfadeLen - xfade_remaining_ + 1) * inv_xf;
            if (t > 1.0f) t = 1.0f;
            raw = old_raw * (1.0f - t) + raw * t;
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
