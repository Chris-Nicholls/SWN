/*
 * test_main.cpp — drives HaloVoice through a few standard test
 * cases, writes WAV files, and prints summary discontinuity metrics.
 *
 * Each scenario triggers a note, renders ~1 s of audio, and then
 * scans the output for sample-to-sample steps that exceed a
 * pitch-relative threshold (cycle-frequency artefacts).  The point of
 * the harness is to A/B candidate fixes against the SWN port without
 * having to flash the hardware every iteration.
 *
 * Usage:
 *   ./ringtest                 # runs the default scenario list
 *   ./ringtest <out_dir>       # writes WAVs under <out_dir>/
 */
#include "halo_voice.hpp"
#include "halo_voice_amortized.hpp"
#include "wav_io.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr int   kBlockSize    = 64;
constexpr float kSampleRate   = HaloVoice::kSampleRate;

/* Build a 512-sample seed waveform.  Mode 0 = sine, 1 = saw, 2 = square,
 * 3 = sine + 5th. */
void build_seed(int mode, float* out) {
    const int N = 512;
    for (int n = 0; n < N; ++n) {
        float phi = 2.0f * (float)M_PI * (float)n / (float)N;
        switch (mode) {
            case 1:  out[n] = (float)n * (2.0f / (float)N) - 1.0f; break;
            case 2:  out[n] = (n < N/2) ? 1.0f : -1.0f;            break;
            case 3:  out[n] = 0.7f * std::sin(phi) +
                              0.3f * std::sin(5.0f * phi);          break;
            case 0:
            default: out[n] = std::sin(phi);                        break;
        }
    }
}

struct Scenario {
    const char* label;
    int   seed_mode;
    float pitch_hz;
    float damping;
    int   lpf_cutoff;
    float noise_level;
    float noise_color;
    float wt_attack;
    float duration_s;
    /* Optional dynamic behaviour: 0/0 = static envelope at 1.0
     * (default).  attack_ms > 0 simulates the firmware's LPG/LFO-VCA
     * envelope ramp on each trigger.  retrigger_period_s > 0 fires a
     * fresh trigger every period, exercising the trigger crossfade. */
    float env_attack_ms      = 0.0f;
    float retrigger_period_s = 0.0f;
};

/* Single-sample step magnitudes between consecutive output samples,
 * measured ONLY at indices where the playback head was about to wrap
 * (i.e. samples that crossed a buffer boundary in the previous step).
 * This is where streaming-LPF / advance-cycle / xfade bugs manifest.
 *
 * Returns max |Δ| over the wrap points and a count of "loud" jumps
 * (those exceeding `loud_threshold`). */
struct StepStats {
    float    max_step;
    int      loud_count;
    int      wrap_samples;
};

StepStats analyse(const std::vector<float>& y, float loud_threshold) {
    StepStats st{0.0f, 0, 0};
    /* Without the engine sharing wrap indices, approximate by looking
     * at every sample-to-sample diff; the loudest will be wrap-driven
     * for any non-pathological signal. */
    for (size_t i = 1; i < y.size(); ++i) {
        float d = std::fabs(y[i] - y[i-1]);
        if (d > st.max_step) st.max_step = d;
        if (d > loud_threshold) st.loud_count++;
    }
    st.wrap_samples = (int)y.size();
    return st;
}

/* Drive any voice type that exposes the same fillBlock / trigger /
 * setter API.  Templated so we don't duplicate the loop body. */
template <typename Voice>
void render_with(Voice& voice, const Scenario& sc, std::vector<float>& out) {
    voice.set_damping     (sc.damping);
    voice.set_lpf_cutoff  (sc.lpf_cutoff);
    voice.set_noise_level (sc.noise_level);
    voice.set_noise_color (sc.noise_color);
    voice.set_wt_attack   (sc.wt_attack);
    if (sc.env_attack_ms > 0.0f) {
        voice.set_envelope_ramp(sc.env_attack_ms);
    } else {
        voice.set_external_env(1.0f);
    }

    float seed[512];
    build_seed(sc.seed_mode, seed);
    voice.trigger(seed, sc.pitch_hz);

    const int total = (int)out.size();
    int retrigger_period_samples =
        (sc.retrigger_period_s > 0.0f)
            ? (int)(sc.retrigger_period_s * kSampleRate) : 0;
    int next_retrigger = retrigger_period_samples;

    int i = 0;
    while (i < total) {
        int n = std::min(kBlockSize, total - i);
        if (retrigger_period_samples > 0 && next_retrigger > i &&
            next_retrigger < i + n) {
            n = next_retrigger - i;
        }
        voice.fillBlock(&out[i], n);
        i += n;
        if (retrigger_period_samples > 0 && i >= next_retrigger) {
            voice.retrigger(sc.pitch_hz);
            next_retrigger += retrigger_period_samples;
        }
    }
}

void run_scenario(const Scenario& sc, const std::string& out_dir) {
    const int total = (int)(sc.duration_s * kSampleRate);

    /* ── Engine A: original (synchronous advance in audio loop) ── */
    std::vector<float> out_a(total, 0.0f);
    {
        HaloVoice voice;
        render_with(voice, sc, out_a);
    }
    /* ── Engine B: amortized streaming physics ── */
    std::vector<float> out_b(total, 0.0f);
    uint64_t phys_count = 0, wrap_count = 0;
    int      phys_n_b = 0;
    {
        HaloVoiceAmortized voice;
        render_with(voice, sc, out_b);
        phys_count = voice.phys_count();
        wrap_count = voice.wrap_count();
        phys_n_b   = voice.phys_n();
    }

    /* Score each engine independently with the same thresholds. */
    auto score = [](const std::vector<float>& y){
        float peak = 0.0f;
        for (float v : y) if (std::fabs(v) > peak) peak = std::fabs(v);
        float thr = 0.05f * (peak > 0.0f ? peak : 1.0f);
        StepStats st{0.0f, 0, (int)y.size()};
        for (size_t i = 1; i < y.size(); ++i) {
            float d = std::fabs(y[i] - y[i-1]);
            if (d > st.max_step) st.max_step = d;
            if (d > thr) st.loud_count++;
        }
        return std::tuple<float, StepStats>(peak, st);
    };
    auto [peak_a, st_a] = score(out_a);
    auto [peak_b, st_b] = score(out_b);

    std::printf("  %-32s  A: peak=%5.2f maxStep=%5.3f  "
                "B: peak=%5.2f maxStep=%5.3f  "
                "B/phys=%llu B/wraps=%llu M=%d\n",
                sc.label,
                peak_a, st_a.max_step,
                peak_b, st_b.max_step,
                (unsigned long long)phys_count,
                (unsigned long long)wrap_count,
                phys_n_b);

    std::string base = out_dir + "/" + sc.label;
    for (auto& c : base) if (c == ' ') c = '_';
    if (!wav::write_mono_f32(base + "_A.wav", out_a))
        std::fprintf(stderr, "    !! failed to write %s_A.wav\n", base.c_str());
    if (!wav::write_mono_f32(base + "_B.wav", out_b))
        std::fprintf(stderr, "    !! failed to write %s_B.wav\n", base.c_str());
}

} /* anonymous */

/* Diagnose ONE scenario in detail: enable wrap recording, render,
 * then dump a CSV of every wrap and print the worst boundary jumps.
 * Used to root-cause cycle-boundary discontinuities. */
void run_diagnose(const Scenario& sc, const std::string& out_dir) {
    HaloVoice voice;

    voice.set_damping     (sc.damping);
    voice.set_lpf_cutoff  (sc.lpf_cutoff);
    voice.set_noise_level (sc.noise_level);
    voice.set_noise_color (sc.noise_color);
    voice.set_wt_attack   (sc.wt_attack);
    if (sc.env_attack_ms > 0.0f) {
        voice.set_envelope_ramp(sc.env_attack_ms);
    } else {
        voice.set_external_env(1.0f);
    }

    float seed[512];
    build_seed(sc.seed_mode, seed);
    voice.trigger(seed, sc.pitch_hz);
    voice.enable_wrap_recording(true);

    const int total = (int)(sc.duration_s * kSampleRate);
    std::vector<float> out(total, 0.0f);

    int retrigger_period_samples =
        (sc.retrigger_period_s > 0.0f)
            ? (int)(sc.retrigger_period_s * kSampleRate) : 0;
    int next_retrigger = retrigger_period_samples;

    {
        int i = 0;
        while (i < total) {
            int n = std::min(kBlockSize, total - i);
            if (retrigger_period_samples > 0 && next_retrigger > i &&
                next_retrigger < i + n) {
                n = next_retrigger - i;
            }
            voice.fillBlock(&out[i], n);
            i += n;
            if (retrigger_period_samples > 0 && i >= next_retrigger) {
                voice.retrigger(sc.pitch_hz);
                next_retrigger += retrigger_period_samples;
            }
        }
    }

    std::string base = out_dir + "/" + sc.label;
    for (auto& c : base) if (c == ' ') c = '_';

    /* WAV. */
    wav::write_mono_f32(base + ".wav", out);

    /* CSV with one row per wrap: cycle index, sample index, audio
     * step, post-wrap audio, post-wrap raw boundary samples
     * (q_old_last vs q_new_first), LPF state. */
    std::string csv_path = base + "_wraps.csv";
    FILE* f = std::fopen(csv_path.c_str(), "w");
    if (f) {
        std::fprintf(f, "wrap,sample,audio_pre,audio_post,audio_step,"
                        "q_old_last,q_new_first,raw_step,"
                        "lpf_q1,lpf_q2,buf_sel,ran_advance\n");
        const auto& events = voice.wrap_events();
        for (size_t k = 0; k < events.size(); ++k) {
            const auto& e = events[k];
            float audio_step = e.audio_post - e.audio_pre;
            float raw_step   = e.q_new_first - e.q_old_last;
            std::fprintf(f, "%zu,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n",
                         k, e.sample_index, e.audio_pre, e.audio_post, audio_step,
                         e.q_old_last, e.q_new_first, raw_step,
                         e.lpf_state_q1, e.lpf_state_q2,
                         e.buffer_sel_after, e.ran_advance ? 1 : 0);
        }
        std::fclose(f);
    }

    /* Summary stats. */
    const auto& events = voice.wrap_events();
    if (events.empty()) {
        std::printf("[%s] no wraps recorded\n", sc.label);
        return;
    }
    float max_audio_step = 0.0f, max_raw_step = 0.0f;
    int   max_audio_idx = -1,   max_raw_idx  = -1;
    double sum_abs_audio = 0.0, sum_abs_raw = 0.0;
    for (size_t k = 0; k < events.size(); ++k) {
        float a = std::fabs(events[k].audio_post - events[k].audio_pre);
        float r = std::fabs(events[k].q_new_first - events[k].q_old_last);
        sum_abs_audio += a;
        sum_abs_raw   += r;
        if (a > max_audio_step) { max_audio_step = a; max_audio_idx = (int)k; }
        if (r > max_raw_step)   { max_raw_step   = r; max_raw_idx   = (int)k; }
    }
    std::printf("[%s] %zu wraps. M=%d, head_inc=%.4f\n",
                sc.label, events.size(), voice.phys_n(), voice.head_inc());
    std::printf("  audio step: max=%.4f mean|.|=%.4f (worst at wrap %d, sample %d)\n",
                max_audio_step, sum_abs_audio / events.size(),
                max_audio_idx, max_audio_idx >= 0 ? events[max_audio_idx].sample_index : -1);
    std::printf("  raw   step: max=%.4f mean|.|=%.4f (worst at wrap %d, sample %d)\n",
                max_raw_step, sum_abs_raw / events.size(),
                max_raw_idx, max_raw_idx >= 0 ? events[max_raw_idx].sample_index : -1);

    /* Find big sample-to-sample steps anywhere in the output. */
    {
        float thresh = 0.20f;
        int   shown  = 0;
        std::printf("  big steps (|Δ| > %.2f) anywhere in output:\n", thresh);
        for (int i = 1; i < (int)out.size() && shown < 15; ++i) {
            float d = out[i] - out[i-1];
            if (std::fabs(d) > thresh) {
                std::printf("    sample %6d: %+7.4f -> %+7.4f (Δ=%+7.4f)\n",
                            i, out[i-1], out[i], d);
                shown++;
            }
        }
    }

    /* Print first 5 wraps + worst 5 wraps so the eyeball test is
     * cheap. */
    auto print_wrap = [](size_t k, const WrapEvent& e){
        std::printf("    wrap=%4zu s=%4d  audio:%+7.4f->%+7.4f Δ=%+.4f | "
                    "q[%d]=%+7.4f q[%d]=%+7.4f rhd=%.3f | "
                    "old[M-1]=%+7.4f new[0]=%+7.4f Δraw=%+.4f | "
                    "buf=%d adv=%d\n",
                    k, e.sample_index, e.audio_pre, e.audio_post,
                    e.audio_post - e.audio_pre,
                    e.rh0_post_wrap,     e.q_new_first,
                    e.rh0_post_wrap + 1, e.q_new_second,
                    e.rhd_post_wrap,
                    e.q_old_last, e.q_new_first,
                    e.q_new_first - e.q_old_last,
                    e.buffer_sel_after, e.ran_advance);
    };
    std::printf("  first 5 wraps:\n");
    for (size_t k = 0; k < events.size() && k < 5; ++k) print_wrap(k, events[k]);
    std::printf("  steady-state (wraps 50..54):\n");
    for (size_t k = 50; k < events.size() && k < 55; ++k) print_wrap(k, events[k]);
}

int main(int argc, char** argv) {
    std::string out_dir = "out";
    bool        diagnose = false;
    std::string diag_label;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--diagnose" && i + 1 < argc) {
            diagnose   = true;
            diag_label = argv[++i];
        } else if (a == "--out" && i + 1 < argc) {
            out_dir = argv[++i];
        } else if (a[0] != '-') {
            out_dir = a;     /* positional fallback */
        }
    }

    /* mkdir -p (host-only convenience). */
    std::string mk = "mkdir -p " + out_dir;
    if (std::system(mk.c_str()) != 0) {
        std::fprintf(stderr, "could not create %s\n", out_dir.c_str());
        return 1;
    }

    /* Sweep across pitch / damping / wt_attack to surface the cases
     * the user has been hearing on hardware: high pitch + high
     * damping shows the cross-buffer step, mid pitch + wtAttack > 0
     * shows the per-cycle injection click. */
    const Scenario scenarios[] = {
      /*  label,                        seed, pitch,  damp,  cutoff, nL,    nC,   wtA,   dur,   atk_ms, retrig_s */
        {"sine_220Hz_low_damp",         0,    220.0f, 0.10f, 24,     0.10f, 0.40f, 0.0f, 1.0f},
        {"sine_220Hz_high_damp",        0,    220.0f, 0.80f, 24,     0.10f, 0.40f, 0.0f, 1.0f},
        {"sine_440Hz_high_damp",        0,    440.0f, 0.80f, 24,     0.10f, 0.40f, 0.0f, 1.0f},
        {"sine_880Hz_high_damp",        0,    880.0f, 0.80f, 24,     0.10f, 0.40f, 0.0f, 1.0f},
        {"sine_1500Hz_high_damp",       0,   1500.0f, 0.80f, 24,     0.10f, 0.40f, 0.0f, 1.0f},
        {"saw_440Hz_mid_damp",          1,    440.0f, 0.40f, 24,     0.10f, 0.40f, 0.0f, 1.0f},
        {"sine_440Hz_inject",           0,    440.0f, 0.40f, 24,     0.10f, 0.40f, 0.5f, 1.0f},
        {"sine_440Hz_inject_max",       0,    440.0f, 0.40f, 24,     0.10f, 0.40f, 1.0f, 1.0f},
        {"sine_220Hz_inject_max",       0,    220.0f, 0.40f, 24,     0.10f, 0.40f, 1.0f, 1.0f},
        {"sine_880Hz_inject_max",       0,    880.0f, 0.40f, 24,     0.10f, 0.40f, 1.0f, 1.0f},
        {"saw_440Hz_high_damp_inject",  1,    440.0f, 0.80f, 24,     0.10f, 0.40f, 0.5f, 1.0f},

        /* Dynamic-envelope tests: simulate the firmware's LPG / LFO-VCA
         * ramp on externalEnvLevel.  A 30 ms attack is typical of an
         * LPG vactrol pluck. */
        {"sine_440Hz_env30ms",          0,    440.0f, 0.40f, 24,     0.10f, 0.40f, 0.5f, 1.0f, 30.0f},
        {"sine_880Hz_env30ms",          0,    880.0f, 0.40f, 24,     0.10f, 0.40f, 0.5f, 1.0f, 30.0f},
        {"sine_440Hz_env100ms_inject",  0,    440.0f, 0.40f, 24,     0.10f, 0.40f, 1.0f, 1.0f, 100.0f},

        /* Retrigger-burst tests: stress the trigger crossfade by
         * firing a fresh trigger every N seconds.  Listen for a click
         * at every retrigger. */
        {"retrigger_4Hz_440_static",    0,    440.0f, 0.40f, 24,     0.10f, 0.40f, 0.0f, 2.0f, 0.0f,   0.25f},
        {"retrigger_4Hz_440_env",       0,    440.0f, 0.40f, 24,     0.10f, 0.40f, 0.0f, 2.0f, 30.0f,  0.25f},
        {"retrigger_8Hz_880_inject",    0,    880.0f, 0.40f, 24,     0.10f, 0.40f, 0.5f, 2.0f, 30.0f,  0.125f},
    };

    if (diagnose) {
        for (const auto& sc : scenarios) {
            if (diag_label == sc.label) {
                run_diagnose(sc, out_dir);
                return 0;
            }
        }
        std::fprintf(stderr, "unknown scenario %s\n", diag_label.c_str());
        std::fprintf(stderr, "available:\n");
        for (const auto& sc : scenarios) std::fprintf(stderr, "  %s\n", sc.label);
        return 1;
    }

    std::printf("Halo host harness — output dir: %s\n", out_dir.c_str());
    std::printf("  A = current engine (synchronous advance_cycle in audio loop)\n");
    std::printf("  B = amortized engine (one physics step per audio sample, streaming)\n");
    for (const auto& sc : scenarios) {
        run_scenario(sc, out_dir);
    }
    return 0;
}
