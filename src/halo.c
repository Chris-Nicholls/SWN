/*
 * halo.c - Halo circular-buffer physical string model
 *
 * Port of the LagrangianAcoustics class from app/js/synths.js.
 * See inc/halo.h for algorithm overview.
 */

#include "halo.h"
#include "oscillator.h"
#include "stm32f7xx.h"   /* DWT for physics-timing diagnostic */
#include "diag_log.h"    /* firehose event stream on right audio channel */
#include <math.h>
#include <string.h>

/* Memory barrier: full DMB on Cortex-M (firmware target), no-op on
 * non-ARM hosts so the file compiles into the C++ test harness in
 * app/cpp_test.  ARMv7-M defines __arm__; macOS arm64 hosts define
 * __aarch64__ but not __arm__, so this gate keeps the dmb out of host
 * builds where the assembler would reject the bare `dmb` mnemonic. */
#if defined(__arm__)
  #define RS_MEM_BARRIER() __asm volatile ("dmb" ::: "memory")
#else
  #define RS_MEM_BARRIER() ((void)0)
#endif

/* Diagnostic: split halo_tick() into its physics (advance_cycle)
 * and trigger (seed_lerp) costs.  Populated only when this file is
 * compiled with the instrumentation enabled; globals live in
 * timekeeper.c so display_cpu_usage() can pick them up.  Temporary —
 * remove once OSC_TIM overruns are resolved. */
extern volatile uint32_t diag_advance_cycle_peak_cycles;
extern volatile uint32_t diag_seed_lerp_peak_cycles;

/* ────────────────────────────────────────────────────────────────────────── */
/*  Helpers                                                                   */
/* ────────────────────────────────────────────────────────────────────────── */

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* Xorshift32 → uniform float in [-1, 1].
 * Seeds of 0 degenerate to all-zero output, so guard on entry. */
static inline float xor_rand(uint32_t *state) {
    uint32_t s = *state;
    if (s == 0) s = 0xDEADBEEFu;
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    *state = s;
    return ((float)(int32_t)s) / 2147483648.0f;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  ADSR Envelope (times in waveform cycles)                                  */
/* ────────────────────────────────────────────────────────────────────────── */

__attribute__((unused))
static float advance_envelope(o_halo *rs) {
    switch (rs->envPhase) {
        case RS_ENV_ATTACK: {
            uint32_t a = rs->noiseAttack;
            rs->envLevel = (a > 0) ? fminf(1.0f, (float)rs->envCycle / (float)a) : 1.0f;
            rs->envCycle++;
            if (rs->envLevel >= 1.0f) {
                rs->envPhase = RS_ENV_DECAY;
                rs->envCycle = 0;
            }
            break;
        }
        case RS_ENV_DECAY: {
            uint32_t d = rs->noiseDecay;
            float s = rs->noiseSustain;
            rs->envLevel = (d > 0) ? 1.0f - (1.0f - s) * fminf(1.0f, (float)rs->envCycle / (float)d) : s;
            rs->envCycle++;
            if (rs->envCycle >= d) {
                rs->envPhase = RS_ENV_SUSTAIN;
                rs->envCycle = 0;
            }
            break;
        }
        case RS_ENV_SUSTAIN:
            rs->envLevel = rs->noiseSustain;
            break;
        case RS_ENV_RELEASE: {
            uint32_t r = rs->noiseRelease;
            float start = rs->envReleaseStart;
            rs->envLevel = (r > 0) ? start * (1.0f - fminf(1.0f, (float)rs->envCycle / (float)r)) : 0.0f;
            rs->envCycle++;
            if (rs->envCycle >= r) {
                rs->envPhase = RS_ENV_OFF;
                rs->envLevel = 0.0f;
            }
            break;
        }
        case RS_ENV_OFF:
        default:
            rs->envLevel = 0.0f;
            break;
    }
    return rs->envLevel;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Streaming cascaded 2-pole LPF                                             */
/*                                                                            */
/*  Two cascaded one-pole IIRs.  The pole state (y1, y2) is preserved by      */
/*  the caller across cycles, so y[0] of cycle k+1 picks up smoothly from     */
/*  y[N-1] of cycle k:                                                        */
/*                                                                            */
/*      y1 = α·y1_prev + (1-α)·x[n]                                           */
/*      y2 = α·y2_prev + (1-α)·y1                                             */
/*      out[n] = (1-mix)·x[n] + mix·y2                                        */
/*                                                                            */
/*  Cost: 1 forward pass of N samples for the full 2-pole cascade.  The       */
/*  old circular variant required ~4 passes (forced response + filter pass    */
/*  per pole) so this is roughly 4× faster — fast enough that we can run      */
/*  advance_cycle every audible cycle and retire the round-robin schedule.    */
/*                                                                            */
/*  IMPORTANT: this filter does NOT enforce self-cyclic closure (y[N-1] is    */
/*  NOT pinned to y[0]).  The buffer is no longer safe to wrap within —       */
/*  audio MUST always wrap at a buffer-flip moment, which means physics MUST  */
/*  keep up with audio cycle-for-cycle.  In return, the flip is naturally    */
/*  step-free at any damping level.                                           */
/* ────────────────────────────────────────────────────────────────────────── */

static void streaming_lpf(float *cachedCutoff, float *alpha,
                          float *state1, float *state2,
                          float *arr, float cutoff, float mix, int N) {
    if (mix <= 0.0f) {
        /* Even when bypassed, drag the state along so a damping
         * sweep from 0 → non-zero starts cleanly from current signal. */
        if (N > 0) {
            *state1 = arr[N - 1];
            *state2 = arr[N - 1];
        }
        return;
    }

    /* Stability guard: cutoff < N/2 (Nyquist). */
    float max_cutoff = (float)(N / 2 - 1);
    if (cutoff > max_cutoff) cutoff = max_cutoff;
    if (cutoff < 1.0f)       cutoff = 1.0f;

    if (*cachedCutoff != cutoff) {
        float omega0 = 2.0f * (float)M_PI * cutoff / (float)N;
        *alpha = expf(-omega0);
        *cachedCutoff = cutoff;
    }

    float a = *alpha;
    float b = 1.0f - a;
    float dry = 1.0f - mix;
    float y1 = *state1;
    float y2 = *state2;

    for (int n = 0; n < N; n++) {
        float x = arr[n];
        y1 = a * y1 + b * x;
        y2 = a * y2 + b * y1;
        arr[n] = dry * x + mix * y2;
    }

    *state1 = y1;
    *state2 = y2;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Public API                                                                */
/* ────────────────────────────────────────────────────────────────────────── */

void halo_init(o_halo *rs, float *q) {
    /* Zero velocity */
    memset(rs->v, 0, sizeof(rs->v));

    /* Default to full-size buffer until a trigger picks a pitch-adapted M. */
    rs->phys_N = RS_N;

    /* Initialise q to a sine wave at ±1.0 scale
     * (The physics operates in normalised [-1, 1] range;
     *  oscillator.c scales up to DAC range on output.) */
    for (int n = 0; n < RS_N; n++) {
        q[n] = sinf(2.0f * (float)M_PI * (float)n / (float)RS_N);
    }

    /* ADSR off */
    rs->envPhase = RS_ENV_OFF;
    rs->envLevel = 0.0f;
    rs->envCycle = 0;
    rs->envReleaseStart = 0.0f;

    /* RNG init */
    rs->rngState = 0x12345678u + (uint32_t)(uintptr_t)rs; /* unique per channel */
    rs->noiseFilterState = 0.0f;

    /* LPF cache invalid */
    rs->lpfCachedCutoffQ = -1.0f;
    rs->lpfAlphaQ = 0.0f;
    rs->lpfStateQ1 = 0.0f;
    rs->lpfStateQ2 = 0.0f;
    rs->lpfCachedCutoffV = -1.0f;
    rs->lpfAlphaV = 0.0f;
    rs->lpfStateV1 = 0.0f;
    rs->lpfStateV2 = 0.0f;

    /* Default runtime params */
    rs->lpfCutoff   = 24;
    rs->damping     = 0.2f;
    rs->noiseLevel  = 0.1f;
    rs->noiseColor  = 0.4f;
    rs->wtAttack    = 0.0f;

    /* _wtOriginal starts zeroed; first trigger fills it from the seed. */
    memset(rs->_wtOriginal, 0, sizeof(rs->_wtOriginal));

    /* Default ADSR */
    rs->noiseAttack  = 5;
    rs->noiseDecay   = 50;
    rs->noiseSustain = 0.3f;
    rs->noiseRelease = 100;

    rs->triggered = 0;
    rs->triggerPending = 0;
    rs->cycleRequest = 0;
    rs->backReady = 0;
    rs->externalEnvLevel = 0.0f;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Scheduler tick — runs from OSC_TIM IRQ (priority 1,1).                    */
/*  Consumes triggerPending / cycleRequest set by the audio ISR and produces  */
/*  a fresh back buffer, signalling backReady when complete. The audio ISR    */
/*  is the only reader of q_front, and only flips buffer_sel once backReady.  */
/* ────────────────────────────────────────────────────────────────────────── */

void halo_tick(o_halo *rs, float *q_front, float *q_back,
                      const int16_t *seed_a, const int16_t *seed_b,
                      float frac)
{
    /* Trigger handling: a note trigger resets the circular buffer to the
     * currently-selected wavetable cycle (lerp between seed_a and seed_b).
     * The external envelope (LPG vactrol or LFO-VCA, see externalEnvLevel)
     * shapes the amplitude of the output, so any discontinuity at the
     * seed instant is masked by the envelope being near zero at attack.
     *
     * We do NOT gate this on !backReady.  The old design waited for the
     * audio ISR to consume any pending back buffer before writing the
     * trigger, but that wait is gated by the oscillator's phase wrap
     * (which only fires once per waveform period).  At low pitches that
     * introduces up to 2× the waveform period of latency — easily
     * 40-80 ms — causing notes to be missed or arrive late.  Since the
     * audio ISR only reads q_front and only flips buffer_sel on the
     * NEXT wrap, overwriting q_back here is safe: the ISR never touches
     * q_back between wraps.  The worst that happens is a previous
     * advance_cycle result is lost, which is inaudible because the
     * trigger supersedes it anyway. */
    if (rs->triggerPending) {
        uint32_t t0 = DWT->CYCCNT;
        halo_seed_lerp(rs, q_back, seed_a, seed_b, frac, rs->phys_N);
        uint32_t dt_lerp = DWT->CYCCNT - t0;
        if (dt_lerp > diag_seed_lerp_peak_cycles)
            diag_seed_lerp_peak_cycles = dt_lerp;
        diag_log(DIAG_EVT_SEEDLERP, dt_lerp);
        halo_arm_envelope(rs);
        rs->triggerPending = 0;
        /* Drop any in-flight cycle request — the seed supersedes it. */
        rs->cycleRequest = 0;
        RS_MEM_BARRIER();
        rs->backReady = 1;
        return;
    }

    /* Advance into the back buffer if the audio ISR asked for one and
     * we haven't already produced one waiting to be consumed.
     *
     * NB: we deliberately do NOT modify q_back here to mask the cross-
     * cycle phase-rotation step (the cascaded LPF imparts a small
     * non-uniform group delay every cycle, so q_back ≈ phase-rotated
     * q_front).  Doing so would force q_back[0] = q_front[0] but break
     * q_back's own internal cyclic closure — moving the audible step
     * from the buffer-flip moment to q_back's own wrap (q_back[M-1] →
     * q_back[0]), which is in fact a worse outcome because the audio
     * may cycle inside q_back several times before the next flip.
     *
     * The right place to mask the step is the audio domain: the audio
     * ISR keeps a small xfade state per channel, blending the old and
     * new buffers at the same read position for ~32 samples after a
     * flip event.  See `xfade_remaining` etc. on o_wt_osc. */
    if (rs->cycleRequest && !rs->backReady) {
        int M = rs->phys_N;
        if (M < RS_M_MIN || M > RS_N) M = RS_N;
        memcpy(q_back, q_front, (size_t)M * sizeof(float));
        uint32_t t0 = DWT->CYCCNT;
        halo_advance_cycle(rs, q_back);
        uint32_t dt_adv = DWT->CYCCNT - t0;
        if (dt_adv > diag_advance_cycle_peak_cycles)
            diag_advance_cycle_peak_cycles = dt_adv;
        diag_log(DIAG_EVT_ADVANCE, dt_adv);

        rs->cycleRequest = 0;
        /* Memory barrier: ensure the q_back writes are globally visible
         * before the audio ISR sees backReady=1 and flips buffer_sel. */
        RS_MEM_BARRIER();
        rs->backReady = 1;
    }
}

/* Shared helper: lerp-resample wave_a/wave_b into wt[0..M-1].
 * Returns 1 on success, 0 if both inputs were NULL (caller handles
 * that case — the buffer is left untouched). */
static int fill_wt_from_seeds(float *wt,
                              const int16_t *wave_a, const int16_t *wave_b,
                              float frac, int M) {
    const float inv_scale = 1.0f / 32768.0f;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    if (M < RS_M_MIN || M > RS_N) M = RS_N;
    const float ratio = (float)RS_N / (float)M;

    if (wave_a && wave_b) {
        for (int n = 0; n < M; n++) {
            float pos = (float)n * ratio;
            int i0 = (int)pos;
            int i1 = (i0 + 1) & (RS_N - 1);
            float t = pos - (float)i0;
            float a = (float)wave_a[i0] * (1.0f - t) + (float)wave_a[i1] * t;
            float b = (float)wave_b[i0] * (1.0f - t) + (float)wave_b[i1] * t;
            wt[n] = (a * (1.0f - frac) + b * frac) * inv_scale;
        }
        return 1;
    } else if (wave_a) {
        for (int n = 0; n < M; n++) {
            float pos = (float)n * ratio;
            int i0 = (int)pos;
            int i1 = (i0 + 1) & (RS_N - 1);
            float t = pos - (float)i0;
            float a = (float)wave_a[i0] * (1.0f - t) + (float)wave_a[i1] * t;
            wt[n] = a * (1.0f - frac) * inv_scale;
        }
        return 1;
    } else if (wave_b) {
        for (int n = 0; n < M; n++) {
            float pos = (float)n * ratio;
            int i0 = (int)pos;
            int i1 = (i0 + 1) & (RS_N - 1);
            float t = pos - (float)i0;
            float b = (float)wave_b[i0] * (1.0f - t) + (float)wave_b[i1] * t;
            wt[n] = b * frac * inv_scale;
        }
        return 1;
    }
    return 0;
}

void halo_refresh_wt_original(o_halo *rs,
                                     const int16_t *wave_a,
                                     const int16_t *wave_b,
                                     float frac, int M) {
    /* Live refresh of the per-cycle injection source buffer.  Does NOT
     * touch q[], v[], envelope, or filter state — those only change on
     * a trigger.  Runs in OSC_TIM context (safe w.r.t. the advance_cycle
     * reader since that also runs in OSC_TIM). */
    if (M < RS_M_MIN || M > RS_N) M = RS_N;
    (void)fill_wt_from_seeds(rs->_wtOriginal, wave_a, wave_b, frac, M);
}

void halo_seed_lerp(o_halo *rs, float *q,
                           const int16_t *wave_a, const int16_t *wave_b,
                           float frac, int M) {
    /* Pitch-adaptive seeding: the seed buffers are RS_N=512 samples but
     * the physics runs on the first M (= phys_N) samples.  We resample
     * the 512-sample seed down to M via linear interpolation so that
     * one period of the seed waveform fills one period of q[].
     *
     * Two-step process (matches the new JS LagrangianAcoustics
     * constructor):
     *
     *   1. Build `_wtOriginal[0..M-1]` as the raw [-1, 1] waveform
     *      (lerped between wave_a and wave_b by `frac`, resampled to
     *      M).  This buffer is kept around for the lifetime of the
     *      note and consumed by the per-cycle injection in
     *      halo_advance_cycle when wtAttack > 0.
     *
     *   2. Compute seedAmt = 1 - wtAttack and either:
     *        - seedAmt > 1e-6  → write q[n] = (1-noiseLevel)*wt[n]
     *          + noiseLevel*colorGain*noise[n]/2, all scaled by
     *          seedAmt, then subtract the DC of that sum.  This is
     *          the JS `_inject(seedAmt, set=true)` path.
     *        - seedAmt ≈ 0     → zero q[0..M-1].  With wtAttack=1 the
     *          note starts silent and builds up via per-cycle
     *          injection alone.
     *
     * NULL pointers indicate an in-flight flash DMA — treat as silence
     * on that endpoint.  If both are NULL there is no valid data; leave
     * _wtOriginal and q alone but still zero velocity/filter state so
     * the trigger is "felt" (envelope re-attacks).
     *
     * M is taken as a parameter (not read from rs->phys_N) so the
     * trigger fast path can seed q_back at the *new* M before
     * committing it to rs->phys_N — keeping the audio ISR's view of
     * (phys_N, buffer_sel) consistent at all times. */

    if (M < RS_M_MIN || M > RS_N) M = RS_N;

    /* Step 1 — fill rs->_wtOriginal with the raw, resampled waveform.
     *   (Both-NULL branch handled explicitly; bails out without
     *   touching _wtOriginal.) */
    float *wt = rs->_wtOriginal;
    if (!fill_wt_from_seeds(wt, wave_a, wave_b, frac, M)) {
        /* Both NULL: leave _wtOriginal and q unchanged; still zero
         * velocity/filter so the trigger re-attacks cleanly. */
        memset(rs->v, 0, sizeof(rs->v));
        rs->noiseFilterState = 0.0f;
        return;
    }

    /* Step 2 — initial seed into q[] scaled by (1 - wtAttack).
     * Matches JS `_inject(seedAmt, set=true)`: mix of (1-noiseLevel)*wt
     * and noiseLevel*colorGain*noise/2, with DC removed at the end. */
    float seedAmt = 1.0f - clampf(rs->wtAttack, 0.0f, 1.0f);
    if (seedAmt > 1e-6f) {
        float noiseLevel = clampf(rs->noiseLevel, 0.0f, 1.0f);
        float wtAmp      = seedAmt * (1.0f - noiseLevel);
        float colorParam = clampf(rs->noiseColor, 0.0f, 1.0f);
        float alpha      = (colorParam < 0.01f) ? 0.0f
                           : (1.0f - powf(10.0f, -4.0f * colorParam));
        /* One-pole LP with coeff α scales white-noise RMS by
         * √((1-α)/(1+α)); pre-multiply by √((1+α)/(1-α)) so coloured
         * noise has the same power as white.  JS reference uses the
         * same compensation. */
        float colorGain  = (alpha < 1e-6f) ? 1.0f
                           : sqrtf((1.0f + alpha) / (1.0f - alpha));
        float noiseAmp   = seedAmt * noiseLevel * colorGain * 0.5f;

        float state  = 0.0f;
        float sumInj = 0.0f;
        for (int n = 0; n < M; n++) {
            float white = xor_rand(&rs->rngState);
            state = alpha * state + (1.0f - alpha) * white;
            float inj = wtAmp * wt[n] + noiseAmp * state;
            q[n] = inj;
            sumInj += inj;
        }
        rs->noiseFilterState = state;

        /* DC removal on q[0..M-1] so the cycle is zero-mean. */
        if (fabsf(sumInj) > 1e-9f) {
            float dc = sumInj / (float)M;
            for (int n = 0; n < M; n++) q[n] -= dc;
        }
    } else {
        /* wtAttack == 1: start silent, per-cycle injection builds it up. */
        for (int n = 0; n < M; n++) q[n] = 0.0f;
        rs->noiseFilterState = 0.0f;
    }

    /* Zero the entire velocity buffer (not just the active M):
     * if a future trigger picks a larger M, the now-active region must
     * already be zero rather than holding stale residue from a previous
     * smaller-M note. */
    memset(rs->v, 0, sizeof(rs->v));

    /* LPF state reset (legacy advance_cycle path). */
    int Mclamped = M;
    if (Mclamped < RS_M_MIN || Mclamped > RS_N) Mclamped = RS_N;
    float seed_last = (Mclamped > 0) ? q[Mclamped - 1] : 0.0f;
    rs->lpfStateQ1 = seed_last;
    rs->lpfStateQ2 = seed_last;
    rs->lpfStateV1 = 0.0f;
    rs->lpfStateV2 = 0.0f;

    /* NOTE: LPF cache is NOT invalidated here.  Cache invalidation on
     * phys_N change is performed by the caller (oscillator.c) at the
     * moment phys_N is updated, before halo_tick is invoked. */
}

void halo_seed(o_halo *rs, float *q,
                      const int16_t *waveform, int M) {
    /* Single-waveform seed path (no cross-fade).  Delegates to the
     * lerp path with both endpoints pointing at `waveform` and frac=0
     * so the wtAttack / noise-injection logic is kept in one place. */
    halo_seed_lerp(rs, q, waveform, waveform, 0.0f, M);
}

void halo_arm_envelope(o_halo *rs) {
    rs->envPhase = RS_ENV_ATTACK;
    rs->envLevel = 0.0f;
    rs->envCycle = 0;
    rs->envReleaseStart = 0.0f;
    rs->noiseFilterState = 0.0f;
    rs->triggered = 1;
}

void halo_release(o_halo *rs) {
    if (rs->envPhase != RS_ENV_RELEASE && rs->envPhase != RS_ENV_OFF) {
        rs->envReleaseStart = rs->envLevel;
        rs->envPhase = RS_ENV_RELEASE;
        rs->envCycle = 0;
    }
}

/* Request a trigger from any context (audio ISR, timer IRQs, main loop).
 * The actual seed/trigger work happens in halo_tick() on OSC_TIM. */
extern o_wt_osc wt_osc;
void halo_request_trigger(uint8_t chan) {
    if (chan >= NUM_CHANNELS) return;
    /* Temporary: stamp the DWT cycle count when a trigger first enters
     * the pending state so update_oscillators() can measure the
     * arm→consume delay.  Main-loop chord retriggers and PWM_OUTS_TIM
     * (LPG strum delay) triggers both flow through here, so this stamp
     * captures both paths.  Stamps are only recorded on the rising
     * edge (no-op if the flag is already latched). */
    if (!wt_osc.halo_state[chan].triggerPending) {
        extern volatile uint32_t diag_trigger_arm_cycle[];
        extern volatile uint32_t diag_retrigger_count[];
        uint32_t now = DWT->CYCCNT;
        /* Force nonzero so the consumer treats it as valid; a true
         * CYCCNT of 0 only happens for ~5 ns right after boot. */
        diag_trigger_arm_cycle[chan] = now ? now : 1u;
        diag_retrigger_count[chan]++;
    }
    wt_osc.halo_state[chan].triggerPending = 1;
}

void halo_advance_cycle(o_halo *rs, float *q) {
    float *v = rs->v;
    /* Pitch-adapted physics: N is per-channel and frozen at trigger.
     * Falls back to RS_N if phys_N was never set (init or pre-trigger). */
    int N = rs->phys_N;
    if (N < RS_M_MIN || N > RS_N) N = RS_N;
    const float dt = RS_DT;
    const float nonlin = RS_NONLINEARITY;
    /* Clamp dtt to prevent positive feedback when q[0] drifts large:
     * dtt = dt + q[0]*dt/2  →  if q[0] grows, dtt grows, which makes
     * position updates bigger, which grows q[0] further → divergence.
     * Clamping to [0.005, 0.05] keeps the integrator stable. */
    const float dtt = clampf(dt + q[0] * dt / 2.0f, 0.005f, 0.05f);

    /* ── 1. Per-cycle wavetable + noise injection into velocity ──
     *
     * Matches the new JS `_inject(injAmt, set=false)` path:
     *   injAmt    = wtAttack * envLevel * dt
     *   v[n]     += (wtAmp * wt[n] + noiseAmp * state) * damping * 10
     *   (DC removed from v afterwards)
     *
     * This replaces the old noise-only injection that added directly to
     * both v[] and q[].  Writing the injection into q[] bypassed the
     * circular LPF in step 4, leaving unfiltered noise energy at the
     * cycle boundary; at high damping that showed up as a discontinuity
     * and audible aliasing.  Routing everything through v[] lets the
     * velocity LPF (half-cutoff) smooth it before the position update
     * folds it into q[], and the position LPF then catches anything
     * left.  The `damping * 10` scaling ensures the injection rate
     * tracks the damping slider — at low damping, injections barely
     * energise the string; at high damping, more input is required to
     * keep the same audible amplitude because the LPF removes more
     * energy per cycle.
     *
     * The external envelope (LPG vactrol in LPG mode, LFO shape in
     * LFO-VCA mode) shapes the injection amount; see externalEnvLevel.
     * rs->noiseFilterState is NOT carried across cycles — the JS reset
     * (state = 0 at each _inject entry) is matched here so coloured
     * noise starts fresh each cycle and doesn't accumulate DC from its
     * own filter lag. */
    float injAmt = clampf(rs->wtAttack, 0.0f, 1.0f)
                   * rs->externalEnvLevel * RS_DT;
    if (injAmt > 1e-9f) {
        float noiseLevel = clampf(rs->noiseLevel, 0.0f, 1.0f);
        float wtAmp      = injAmt * (1.0f - noiseLevel);
        float colorParam = clampf(rs->noiseColor, 0.0f, 1.0f);
        float alpha      = (colorParam < 0.01f) ? 0.0f
                           : (1.0f - powf(10.0f, -4.0f * colorParam));
        float colorGain  = (alpha < 1e-6f) ? 1.0f
                           : sqrtf((1.0f + alpha) / (1.0f - alpha));
        float noiseAmp   = injAmt * noiseLevel * colorGain * 0.5f;
        float dampScale  = rs->damping * 10.0f;

        const float *wt = rs->_wtOriginal;
        float state  = 0.0f;
        float sumInj = 0.0f;
        for (int n = 0; n < N; n++) {
            float white = xor_rand(&rs->rngState);
            state = alpha * state + (1.0f - alpha) * white;
            float inj = wtAmp * wt[n] + noiseAmp * state;
            v[n] += inj * dampScale;
            sumInj += inj;
        }
        rs->noiseFilterState = state;

        /* DC removal on v[0..N-1] to keep the velocity zero-mean. */
        if (fabsf(sumInj) > 1e-9f) {
            float dc = (sumInj / (float)N) * dampScale;
            for (int n = 0; n < N; n++) v[n] -= dc;
        }
    }

    /* ── 2. Antipodal coupling ── */
    int halfN = N / 2;
    for (int n = 0; n < halfN; n++) {
        int nR = n + halfN;
        float d = q[nR] - q[n];
        float f = d * nonlin;
        v[n]  += f * dtt - q[n]  * dtt * 0.1f;
        v[nR] -= f * dtt + q[nR] * dtt * 0.1f;
    }

    /* Velocity soft-clip */
    for (int n = 0; n < N; n++) {
        if (v[n] > 1.0f) v[n] = 1.0f;
        if (v[n] < -1.0f) v[n] = -1.0f;
        if (fabsf(v[n]) > 0.5f) {
            v[n] *= 0.99f;
        }
    }

    /* ── 3. Position update ── */
    for (int n = 0; n < N; n++) {
        q[n] += dtt * v[n];
    }

    /* ── 3b. DC removal on q[] ── */
    /* The LPF in step 4 attenuates high harmonics but NOT the DC/fundamental.
     * Without this, DC slowly accumulates from noise injection rounding and
     * asymmetric coupling forces, eventually growing q[] to Inf → crash.
     * This is a one-pole DC blocker: subtract the running mean each cycle. */
    {
        float dcQ = 0.0f;
        for (int n = 0; n < N; n++) dcQ += q[n];
        dcQ *= (1.0f / (float)N);
        for (int n = 0; n < N; n++) q[n] -= dcQ;
    }

    /* ── 3c. Safety clamp on q[] ── */
    /* Hard limit to ±2.0 as a last resort to prevent float overflow.
     * Normal operating range is well within ±1.0. If we hit this clamp
     * something is already wrong, but at least we won't crash. */
    for (int n = 0; n < N; n++) {
        if (q[n] > 2.0f) q[n] = 2.0f;
        else if (q[n] < -2.0f) q[n] = -2.0f;
    }

    /* ── 4. Streaming LPF on velocity and position ──
     *
     * Velocity cutoff is lpfCutoff/2 (a half-integer harmonic number for
     * odd lpfCutoff).  State is preserved across cycles in
     * rs->lpfState{V,Q}{1,2} so the filter output flows continuously
     * across the buffer-flip boundary (no within-cycle phase step).  This
     * is what lets us drop the read-side crossfade for cycle-advance
     * flips — the flip is step-free at any damping. */
    streaming_lpf(&rs->lpfCachedCutoffV, &rs->lpfAlphaV,
                  &rs->lpfStateV1, &rs->lpfStateV2,
                  v, (float)rs->lpfCutoff * 0.5f, rs->damping, N);
    streaming_lpf(&rs->lpfCachedCutoffQ, &rs->lpfAlphaQ,
                  &rs->lpfStateQ1, &rs->lpfStateQ2,
                  q, (float)rs->lpfCutoff, rs->damping, N);
}
#if 0
/* ────────────────────────────────────────────────────────────────────────── */
/*  Streaming physics — superseded by halo_voice.cpp                   */
/*                                                                            */
/*  halo_step_one() updates ONE buffer index (n) of q[] using the      */
/*  same physics as halo_advance_cycle() (injection, antipodal         */
/*  coupling, position update, DC removal, soft-clip, cascaded LPF) but       */
/*  driven sample-by-sample from the audio ISR.  All per-cycle state          */
/*  (dtt, injection coefficients, DC means) is latched once at phys_head=0    */
/*  by halo_start_phys_cycle().                                        */
/*                                                                            */
/*  Trade-offs vs. the batch advance_cycle:                                   */
/*    - Antipodal coupling reads q[nR] which may be either previous-cycle     */
/*      (nR > n, not yet visited) or current-cycle (nR < n, freshly           */
/*      written).  Inaudible — explicit user OK to use stale state.           */
/*    - DC removal lags one cycle: this cycle subtracts last cycle's mean.    */
/*      Inaudible — DC drift over one period is sub-millisecond.              */
/*    - LPF state evolves continuously across all index boundaries,           */
/*      including the cycle wrap, so the buffer is naturally step-free        */
/*      everywhere.  No double-buffer flip, no cycle-advance crossfade.       */
/* ────────────────────────────────────────────────────────────────────────── */

void halo_start_phys_cycle(o_halo *rs, const float *q) {
    int N = rs->phys_N;
    if (N < RS_M_MIN || N > RS_N) N = RS_N;

    /* dtt depends on q[0], evaluated at the moment we start a new
     * cycle.  Same clamp as the legacy advance_cycle. */
    rs->cycle_dtt = clampf(RS_DT + q[0] * RS_DT * 0.5f, 0.005f, 0.05f);

    /* Injection coefficients — match advance_cycle's per-cycle derivation
     * exactly (envLevel × wtAttack × dt for the overall amplitude, then
     * split between wavetable and coloured-noise contributions). */
    float lvl    = clampf(rs->externalEnvLevel, 0.0f, 1.0f);
    float injAmt = clampf(rs->wtAttack, 0.0f, 1.0f) * lvl * RS_DT;
    rs->cycle_inj_amt = injAmt;

    if (injAmt > 1e-9f) {
        float n_lvl = clampf(rs->noiseLevel, 0.0f, 1.0f);
        rs->cycle_wt_amp     = injAmt * (1.0f - n_lvl);

        float cp    = clampf(rs->noiseColor, 0.0f, 1.0f);
        float alpha = (cp < 0.01f) ? 0.0f
                                   : (1.0f - powf(10.0f, -4.0f * cp));
        float cg    = (alpha < 1e-6f) ? 1.0f
                                      : sqrtf((1.0f + alpha) / (1.0f - alpha));
        rs->cycle_noise_alpha = alpha;
        rs->cycle_noise_amp   = injAmt * n_lvl * cg * 0.5f;
        rs->cycle_damp_scale  = rs->damping * 10.0f;
    } else {
        rs->cycle_wt_amp     = 0.0f;
        rs->cycle_noise_amp  = 0.0f;
        rs->cycle_damp_scale = 0.0f;
    }

    /* Coloured-noise filter restarts each cycle, matching JS / legacy. */
    rs->cycle_noise_state = 0.0f;

    /* Promote last cycle's running sums to "the mean to subtract this
     * cycle".  Both q[] and the velocity-injection term are tracked
     * with one-cycle latency. */
    float invN = 1.0f / (float)N;
    rs->cycle_q_dc_full      = rs->cycle_q_dc_partial   * invN;
    rs->cycle_inj_dc_full    = rs->cycle_inj_dc_partial * invN;
    rs->cycle_q_dc_partial   = 0.0f;
    rs->cycle_inj_dc_partial = 0.0f;

    /* Refresh LPF coefficients if cutoff has changed.  Stability guard
     * matches streaming_lpf(): cutoff < N/2.  These coefficients are
     * read by step_one's per-sample LPF pass. */
    int max_cutoff = N / 2 - 1;
    float cQ = (float)rs->lpfCutoff;
    if (cQ > (float)max_cutoff) cQ = (float)max_cutoff;
    if (cQ < 1.0f) cQ = 1.0f;
    if (rs->lpfCachedCutoffQ != cQ) {
        rs->lpfAlphaQ        = expf(-2.0f * (float)M_PI * cQ / (float)N);
        rs->lpfCachedCutoffQ = cQ;
    }
    float cV = (float)rs->lpfCutoff * 0.5f;
    if (cV > (float)max_cutoff) cV = (float)max_cutoff;
    if (cV < 1.0f) cV = 1.0f;
    if (rs->lpfCachedCutoffV != cV) {
        rs->lpfAlphaV        = expf(-2.0f * (float)M_PI * cV / (float)N);
        rs->lpfCachedCutoffV = cV;
    }
}

void halo_step_one(o_halo *rs, float *q, int32_t n) {
    int N = rs->phys_N;
    if (N < RS_M_MIN || N > RS_N) N = RS_N;
    if (n < 0 || n >= N) return;          /* defensive */

    int32_t nR = n + (N >> 1);
    if (nR >= N) nR -= N;

    float *v = rs->v;
    const float dtt = rs->cycle_dtt;

    /* 1. Per-cycle injection into v[n].  Track running sum so the
     *    NEXT cycle can subtract this cycle's mean (matches the
     *    batch advance_cycle's DC removal on injected v[], delayed
     *    by one cycle — inaudible). */
    if (rs->cycle_inj_amt > 1e-9f) {
        float white = xor_rand(&rs->rngState);
        rs->cycle_noise_state =
            rs->cycle_noise_alpha * rs->cycle_noise_state
          + (1.0f - rs->cycle_noise_alpha) * white;
        float inj =
            rs->cycle_wt_amp    * rs->_wtOriginal[n]
          + rs->cycle_noise_amp * rs->cycle_noise_state;
        v[n] += inj * rs->cycle_damp_scale;
        rs->cycle_inj_dc_partial += inj;
        /* Subtract last cycle's injection mean (constant per cycle). */
        v[n] -= rs->cycle_inj_dc_full * rs->cycle_damp_scale;
    }

    /* 2. Antipodal coupling.  q[nR] is read with whatever value it
     *    currently holds — previous cycle if nR > n, current cycle
     *    if nR < n.  Same algorithm as advance_cycle, just sample-
     *    by-sample. */
    float qn  = q[n];
    float qnR = q[nR];
    float d   = qnR - qn;
    float f   = d * RS_NONLINEARITY;
    v[n] += f * dtt - qn * dtt * 0.1f;

    /* 3. Velocity soft-clip. */
    float vn = v[n];
    if (vn >  1.0f) vn =  1.0f;
    if (vn < -1.0f) vn = -1.0f;
    if (fabsf(vn) > 0.5f) vn *= 0.99f;
    v[n] = vn;

    /* 4. Position update. */
    q[n] = qn + dtt * vn;

    /* 5. DC removal on q[] (one-cycle latency). */
    q[n] -= rs->cycle_q_dc_full;
    rs->cycle_q_dc_partial += q[n];

    /* 6. Safety clamp on q[n]. */
    if (q[n] >  2.0f) q[n] =  2.0f;
    if (q[n] < -2.0f) q[n] = -2.0f;

    /* 7. Streaming LPF on q (cascaded 2-pole).  State carries across
     *    samples and across cycles by construction — naturally
     *    continuous everywhere. */
    {
        float mix = rs->damping;
        if (mix > 0.0f) {
            float a   = rs->lpfAlphaQ;
            float b   = 1.0f - a;
            float dry = 1.0f - mix;
            float x   = q[n];
            rs->lpfStateQ1 = a * rs->lpfStateQ1 + b * x;
            rs->lpfStateQ2 = a * rs->lpfStateQ2 + b * rs->lpfStateQ1;
            q[n]           = dry * x + mix * rs->lpfStateQ2;
        } else {
            /* Drag state along even when bypassed so a damping sweep
             * 0 → non-zero starts cleanly. */
            rs->lpfStateQ1 = q[n];
            rs->lpfStateQ2 = q[n];
        }
    }

    /* 8. Streaming LPF on v. */
    {
        float mix = rs->damping;
        if (mix > 0.0f) {
            float a   = rs->lpfAlphaV;
            float b   = 1.0f - a;
            float dry = 1.0f - mix;
            float x   = v[n];
            rs->lpfStateV1 = a * rs->lpfStateV1 + b * x;
            rs->lpfStateV2 = a * rs->lpfStateV2 + b * rs->lpfStateV1;
            v[n]           = dry * x + mix * rs->lpfStateV2;
        } else {
            rs->lpfStateV1 = v[n];
            rs->lpfStateV2 = v[n];
        }
    }
}
#endif
