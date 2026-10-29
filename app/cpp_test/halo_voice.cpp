/*
 * halo_voice.cpp — host-side wrapper around the firmware
 * Halo physics + audio playback loop.  See header for design.
 */
#include "halo_voice.hpp"

#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdio>

extern "C" {
#include "stm32f7xx.h"   /* DWT stub */
#include "diag_log.h"    /* no-op stub */
#include "oscillator.h"  /* o_wt_osc fake (only halo_request_trigger uses it) */
}

/* ── Globals required by the firmware C objects ───────────────────── */
DWT_Type                  _host_dwt           = {0};
o_wt_osc                  wt_osc;             /* unused by the test path, but
                                                 halo_request_trigger
                                                 references it. */
volatile uint32_t         diag_advance_cycle_peak_cycles = 0;
volatile uint32_t         diag_seed_lerp_peak_cycles     = 0;
volatile uint32_t         diag_trigger_arm_cycle[NUM_CHANNELS]   = {0};
volatile uint32_t         diag_retrigger_count[NUM_CHANNELS]     = {0};

/* ── Helpers ──────────────────────────────────────────────────────── */

void HaloVoice::float_to_i16(const float* in, int16_t* out, int n) {
    for (int i = 0; i < n; ++i) {
        float v = in[i] * 32768.0f;
        if (v >  32767.0f) v =  32767.0f;
        if (v < -32768.0f) v = -32768.0f;
        out[i] = (int16_t)v;
    }
}

/* ── Construction / configuration ─────────────────────────────────── */

HaloVoice::HaloVoice()
    : buffer_sel_(0),
      head_pos_(0.0f),
      head_inc_(1.0f),
      pitch_hz_(110.0f),
      xfade_remaining_(0),
      xfade_prev_buffer_(0),
      xfade_prev_head_(0.0f),
      xfade_prev_M_(kMaxBufLen),
      xfade_prev_inc_(1.0f),
      has_seed_(false),
      n_wraps_(0),
      n_advances_(0)
{
    std::memset(mc_, 0, sizeof(mc_));
    std::memset(seed_wave_, 0, sizeof(seed_wave_));
    halo_init(&rs_, mc_[buffer_sel_]);
    rs_.externalEnvLevel = 1.0f;       /* full-on so per-cycle injection runs */
    recompute_head_inc();
}

void HaloVoice::set_pitch_hz(float hz) {
    if (hz <= 0.0f) hz = 0.0001f;
    pitch_hz_ = hz;
    recompute_head_inc();
}

void HaloVoice::set_damping(float v)        { rs_.damping     = v; }
void HaloVoice::set_lpf_cutoff(int v)       { rs_.lpfCutoff   = v;
                                                    rs_.lpfCachedCutoffQ = -1.0f;
                                                    rs_.lpfCachedCutoffV = -1.0f; }
void HaloVoice::set_noise_level(float v)    { rs_.noiseLevel  = v; }
void HaloVoice::set_noise_color(float v)    { rs_.noiseColor  = v; }
void HaloVoice::set_wt_attack(float v)      { rs_.wtAttack    = v; }
void HaloVoice::set_external_env(float v)   { rs_.externalEnvLevel = v; }

void HaloVoice::set_envelope_ramp(float attack_ms, float sustain_level) {
    env_attack_samples_ = (int)(attack_ms * 0.001f * kSampleRate);
    if (env_attack_samples_ < 0) env_attack_samples_ = 0;
    env_sustain_        = sustain_level;
    env_phase_          = env_attack_samples_;   /* start at sustain */
}

void HaloVoice::recompute_head_inc() {
    int N = rs_.phys_N;
    if (N < RS_M_MIN || N > RS_N) N = RS_N;
    head_inc_ = (pitch_hz_ * (float)N) / kSampleRate;
}

/* ── Trigger paths ────────────────────────────────────────────────── */

void HaloVoice::trigger(const float* waveform_512, float pitch_hz) {
    /* Cache the float seed so retrigger() / load_wavetable() have
     * something to refer back to. */
    std::memcpy(seed_wave_, waveform_512, sizeof(seed_wave_));
    has_seed_ = true;

    /* Convert to int16 to match the firmware seed_lerp signature
     * (firmware seeds come from flash as int16_t[512]). */
    int16_t wave_i16[kMaxBufLen];
    float_to_i16(waveform_512, wave_i16, kMaxBufLen);

    /* Pitch-adapt M for the new note. */
    int new_M = halo_phys_n_for_pitch(pitch_hz, kSampleRate);

    int front  = buffer_sel_;
    int back   = front ^ 1;
    float* qb  = mc_[back];

    /* Disqualify back buffer (no natural flip). */
    rs_.backReady = 0;

    /* Seed q_back at the new M.  halo_seed_lerp also writes
     * _wtOriginal (the per-cycle injection source). */
    halo_seed_lerp(&rs_, qb, wave_i16, wave_i16, 0.0f, new_M);

    /* Re-attack the envelope state. */
    halo_arm_envelope(&rs_);

    /* Atomic commit: latch xfade against dying buffer, swap buffer_sel,
     * reset head, update phys_N, recompute inc, invalidate LPF cache. */
    int old_M = rs_.phys_N;
    if (old_M < RS_M_MIN || old_M > RS_N) old_M = RS_N;
    float old_head = head_pos_;
    if (!std::isfinite(old_head) || old_head < 0.0f) old_head = 0.0f;
    while (old_head >= (float)old_M) old_head -= (float)old_M;

    xfade_prev_buffer_ = front;
    xfade_prev_head_   = old_head;
    xfade_prev_M_      = old_M;
    xfade_prev_inc_    = head_inc_;
    xfade_remaining_   = kXfadeLen;

    rs_.phys_N             = new_M;
    rs_.lpfCachedCutoffQ   = -1.0f;
    rs_.lpfCachedCutoffV   = -1.0f;
    pitch_hz_              = pitch_hz;
    recompute_head_inc();
    buffer_sel_            = back;
    head_pos_              = 0.0f;

    /* Clear pending flags so a stray request can't fire post-trigger. */
    rs_.triggerPending = 0;
    rs_.cycleRequest   = 0;
    /* backReady is left at 0 — first audio wrap will run advance and
     * produce the next q_back. */

    /* Restart the envelope ramp if one is configured. */
    if (env_attack_samples_ > 0) {
        env_phase_         = 0;
        rs_.externalEnvLevel = 0.0f;
    }

}

void HaloVoice::retrigger(float pitch_hz) {
    if (!has_seed_) return;
    trigger(seed_wave_, pitch_hz);
}

void HaloVoice::load_wavetable(const float* waveform_512) {
    std::memcpy(seed_wave_, waveform_512, sizeof(seed_wave_));
    has_seed_ = true;

    int16_t wave_i16[kMaxBufLen];
    float_to_i16(waveform_512, wave_i16, kMaxBufLen);

    /* Refresh _wtOriginal at the current phys_N — does not touch
     * q[], v[], envelope, or filter state.  Mirrors the firmware's
     * encoder-scrub path. */
    halo_refresh_wt_original(&rs_, wave_i16, wave_i16, 0.0f,
                                    rs_.phys_N);
}

/* ── Audio playback loop ─────────────────────────────────────────── */

void HaloVoice::run_advance_in_audio() {
    int front = buffer_sel_;
    int back  = front ^ 1;
    float* qf = mc_[front];
    float* qb = mc_[back];
    int M = rs_.phys_N;
    if (M < RS_M_MIN || M > RS_N) M = RS_N;

    std::memcpy(qb, qf, (size_t)M * sizeof(float));
    halo_advance_cycle(&rs_, qb);
    rs_.backReady = 1;
    n_advances_++;
}

void HaloVoice::fillBlock(float* out, int n_samples) {
    int   M     = rs_.phys_N;
    if (M < RS_M_MIN || M > RS_N) M = RS_N;
    const float fM     = (float)M;
    float*      q      = mc_[buffer_sel_];

    int   xfade_rem    = xfade_remaining_;
    float prev_head    = xfade_prev_head_;
    int   prev_M_l     = xfade_prev_M_;
    if (prev_M_l < RS_M_MIN || prev_M_l > RS_N) prev_M_l = RS_N;
    float prev_fM      = (float)prev_M_l;
    float prev_inc     = xfade_prev_inc_;
    float* prev_q      = mc_[xfade_prev_buffer_];
    const float inv_xf = 1.0f / (float)kXfadeLen;

    for (int i = 0; i < n_samples; ++i) {
        bool wrap_recorded_here = false;

        /* Drive the optional envelope ramp.  Updates externalEnvLevel
         * per-sample so per-cycle injection scales smoothly during
         * the attack phase. */
        if (env_attack_samples_ > 0) {
            if (env_phase_ < env_attack_samples_) {
                rs_.externalEnvLevel =
                    env_sustain_ * (float)env_phase_ /
                    (float)env_attack_samples_;
                env_phase_++;
            } else {
                rs_.externalEnvLevel = env_sustain_;
            }
        }

        head_pos_ += head_inc_;
        if (head_pos_ >= fM) {
            while (head_pos_ >= fM) head_pos_ -= fM;
            if (!std::isfinite(head_pos_)) head_pos_ = 0.0f;
            n_wraps_++;

            /* Snapshot for wrap diagnostics (capture BEFORE the flip
             * and advance so we can attribute the step to either). */
            WrapEvent ev{};
            if (record_wraps_) {
                ev.sample_index    = i;
                ev.audio_pre       = (i > 0) ? out[i-1] : last_out_sample_;
                ev.head_post_wrap  = head_pos_;
                int old_sel = buffer_sel_;
                ev.q_old_last      = mc_[old_sel][M - 1];
            }

            /* Buffer flip: swap to the back buffer if advance has
             * produced one ready (it has, after the first wrap post-
             * trigger).  Streaming-LPF state continuity ensures the
             * flip itself is step-free; no cycle-flip xfade armed
             * here. */
            if (rs_.backReady) {
                buffer_sel_  ^= 1;
                rs_.backReady = 0;
                q             = mc_[buffer_sel_];
            }

            /* Run the next physics step synchronously, but skip while
             * a trigger crossfade is active — the xfade reader is
             * still sampling from the old front buffer (now the
             * "back"), and advance would overwrite it. */
            bool ran_adv = false;
            if (xfade_rem == 0) {
                run_advance_in_audio();
                ran_adv = true;
            }

            if (record_wraps_) {
                ev.buffer_sel_after = buffer_sel_;
                ev.ran_advance      = ran_adv;
                ev.q_new_first      = mc_[buffer_sel_][0];
                ev.q_new_second     = mc_[buffer_sel_][1];
                ev.lpf_state_q1     = rs_.lpfStateQ1;
                ev.lpf_state_q2     = rs_.lpfStateQ2;
                ev.rh0_post_wrap    = (int)head_pos_;
                ev.rhd_post_wrap    = head_pos_ - (float)(int)head_pos_;
                /* audio_post is filled in at the bottom of the loop. */
                wrap_events_.push_back(ev);
                wrap_recorded_here = true;
            }
        }

        /* Linear interp read with streaming-LPF boundary handling. */
        uint16_t rh0 = (uint16_t)head_pos_;
        if (rh0 >= (uint16_t)M) rh0 = 0;
        float rhd = head_pos_ - (float)rh0;
        float s0  = q[rh0];
        float s1;
        if (rh0 == (uint16_t)(M - 1)) {
            if (rs_.backReady) {
                int back_idx = buffer_sel_ ^ 1;
                s1 = mc_[back_idx][0];
            } else {
                s1 = q[0];
            }
        } else {
            s1 = q[rh0 + 1];
        }
        float raw = s0 * (1.0f - rhd) + s1 * rhd;

        /* Trigger-crossfade against the dying buffer. */
        if (xfade_rem > 0) {
            prev_head += prev_inc;
            if (prev_head >= prev_fM) {
                while (prev_head >= prev_fM) prev_head -= prev_fM;
                if (!std::isfinite(prev_head)) prev_head = 0.0f;
            }
            uint16_t pr0 = (uint16_t)prev_head;
            if (pr0 >= (uint16_t)prev_M_l) pr0 = 0;
            uint16_t pr1 = pr0 + 1;
            if (pr1 >= (uint16_t)prev_M_l) pr1 = 0;
            float prd      = prev_head - (float)((uint16_t)prev_head);
            float prev_raw = prev_q[pr0] * (1.0f - prd) + prev_q[pr1] * prd;

            float t = (float)(kXfadeLen - xfade_rem + 1) * inv_xf;
            if (t > 1.0f) t = 1.0f;
            raw = prev_raw * (1.0f - t) + raw * t;
            xfade_rem--;
        }

        if (!std::isfinite(raw)) raw = 0.0f;
        out[i] = raw;

        if (wrap_recorded_here) {
            wrap_events_.back().audio_post = raw;
        }
    }

    /* Commit xfade locals back. */
    xfade_remaining_ = xfade_rem;
    xfade_prev_head_ = prev_head;
    if (n_samples > 0) last_out_sample_ = out[n_samples - 1];
}
