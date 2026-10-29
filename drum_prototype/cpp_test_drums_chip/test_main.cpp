/*
 * test_main.cpp — drives the chiptune (NES 2A03 APU) voices (kick, snare,
 * closed/open hat, perc, cowbell) through default, long-decay, low-filter
 * and extreme-"other" settings, writes WAVs into out/, and prints sanity
 * checks (NaN/Inf scan, peak, RMS envelope, ZCR, FFT peak/centroid).
 *
 * Family-specific extra: `steps` counts the discrete downward jumps in the
 * amplitude envelope. This family's envelope is a genuine 16-step linear
 * staircase, so a blocky RMS trace with several distinct steps is the
 * CORRECT result here, not a defect -- a smooth exponential decay would be
 * the bug. `shape` reports whether the envelope fits a straight line better
 * than an exponential, which is what a linear staircase should do.
 *
 * Usage:
 *   ./drumtest_chip [out_dir]
 */
extern "C" {
#include "drum_voice.h"
}

#include "wav_io.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

constexpr int kBlockSize = 64;
constexpr int kFftSize   = 8192;

struct Scenario {
	const char *label;
	float pitch;
	float filter01;
	float decay01;
	float other01;
	float duration_s;
};

void render_voice(const DrumVoiceOps &ops, const Scenario &sc, std::vector<float> &out)
{
	std::vector<unsigned char> state(ops.state_size);
	void *st = state.data();

	ops.init(st);
	ops.set_filter(st, sc.filter01);
	ops.set_decay(st, sc.decay01);
	ops.set_other(st, sc.other01);
	ops.trigger(st, sc.pitch);

	int n_total = (int)(sc.duration_s * DRUM_VOICE_SAMPLE_RATE);
	out.assign(n_total, 0.0f);

	for (int i = 0; i < n_total; i += kBlockSize) {
		int n = std::min(kBlockSize, n_total - i);
		ops.render(st, out.data() + i, n);
	}
}

float zcr_estimate_hz(const std::vector<float> &y, int start, int n, float sr)
{
	int crossings = 0;
	for (int i = start + 1; i < start + n && i < (int)y.size(); i++) {
		if ((y[i - 1] < 0.0f) != (y[i] < 0.0f)) crossings++;
	}
	return (crossings / 2.0f) / ((float)n / sr);
}

float rms(const std::vector<float> &y, int start, int n)
{
	double acc = 0.0;
	int count = 0;
	for (int i = start; i < start + n && i < (int)y.size(); i++) {
		acc += (double)y[i] * y[i];
		count++;
	}
	return count ? (float)std::sqrt(acc / count) : 0.0f;
}

bool has_nan_or_inf(const std::vector<float> &y)
{
	for (float v : y) {
		if (std::isnan(v) || std::isinf(v)) return true;
	}
	return false;
}

float peak_abs(const std::vector<float> &y)
{
	float m = 0.0f;
	for (float v : y) m = std::max(m, std::fabs(v));
	return m;
}

float audible_duration_s(const std::vector<float> &y, float sr)
{
	float thresh = 0.01f * peak_abs(y);
	int first = -1, last = -1;
	for (int i = 0; i < (int)y.size(); i++) {
		if (std::fabs(y[i]) > thresh) {
			if (first < 0) first = i;
			last = i;
		}
	}
	return (first < 0) ? 0.0f : (float)(last - first) / sr;
}

/* Coarse amplitude envelope, one RMS value per 1ms hop, truncated at the
 * point the hit has died away. */
std::vector<float> envelope(const std::vector<float> &y, float sr)
{
	int hop = (int)(0.001f * sr);
	std::vector<float> raw;
	for (int i = 0; i + hop <= (int)y.size(); i += hop)
		raw.push_back(rms(y, i, hop));

	/* A 3-point smooth: without it the squares' own beating and the noise
	 * channel's variance swamp the staircase edges we are trying to count. */
	std::vector<float> env(raw.size());
	for (size_t i = 0; i < raw.size(); i++) {
		size_t lo = (i > 0) ? i - 1 : 0;
		size_t hi = std::min(raw.size() - 1, i + 1);
		env[i] = (raw[lo] + raw[i] + raw[hi]) / 3.0f;
	}

	float pk = env.empty() ? 0.0f : *std::max_element(env.begin(), env.end());
	int last = -1;
	for (int i = 0; i < (int)env.size(); i++)
		if (env[i] > 0.02f * pk) last = i;
	if (last >= 0) env.resize(last + 1);
	return env;
}

/* Number of distinct downward jumps in the envelope: a 16-step staircase
 * shows up as several of these, a smooth exponential as ~none. */
int staircase_steps(const std::vector<float> &env)
{
	int steps = 0;
	bool falling = false;
	for (size_t i = 1; i < env.size(); i++) {
		float prev = std::max(env[i - 1], 1e-9f);
		if (env[i] < prev * 0.90f) {
			if (!falling) steps++;
			falling = true;
		} else if (env[i] > prev * 0.98f) {
			falling = false;
		}
	}
	return steps;
}

/* R^2 of a straight-line fit through the samples in v. */
double linear_r2(const std::vector<double> &v)
{
	size_t n = v.size();
	if (n < 4) return 0.0;
	double sx = 0, sy = 0, sxx = 0, sxy = 0;
	for (size_t i = 0; i < n; i++) {
		sx += (double)i; sy += v[i];
		sxx += (double)i * i; sxy += (double)i * v[i];
	}
	double den = n * sxx - sx * sx;
	if (std::fabs(den) < 1e-12) return 0.0;
	double a = (n * sxy - sx * sy) / den;
	double b = (sy - a * sx) / n;
	double mean = sy / n, ss_res = 0, ss_tot = 0;
	for (size_t i = 0; i < n; i++) {
		double r = v[i] - (a * i + b);
		ss_res += r * r;
		ss_tot += (v[i] - mean) * (v[i] - mean);
	}
	return (ss_tot > 0) ? 1.0 - ss_res / ss_tot : 0.0;
}

/* "lin" if the decay is closer to a straight line than to an exponential. */
const char *envelope_shape(const std::vector<float> &env)
{
	size_t peak_i = 0;
	for (size_t i = 0; i < env.size(); i++)
		if (env[i] > env[peak_i]) peak_i = i;
	if (env.size() - peak_i < 6) return "n/a";

	std::vector<double> lin, log_;
	for (size_t i = peak_i; i < env.size(); i++) {
		lin.push_back(env[i]);
		log_.push_back(std::log(std::max((double)env[i], 1e-9)));
	}
	return linear_r2(lin) >= linear_r2(log_) ? "lin" : "exp";
}

void fft_in_place(std::vector<std::complex<double>> &a)
{
	const size_t n = a.size();
	for (size_t i = 1, j = 0; i < n; i++) {
		size_t bit = n >> 1;
		for (; j & bit; bit >>= 1) j ^= bit;
		j ^= bit;
		if (i < j) std::swap(a[i], a[j]);
	}
	for (size_t len = 2; len <= n; len <<= 1) {
		double ang = -2.0 * M_PI / (double)len;
		std::complex<double> wl(std::cos(ang), std::sin(ang));
		for (size_t i = 0; i < n; i += len) {
			std::complex<double> w(1.0, 0.0);
			for (size_t k = 0; k < len / 2; k++) {
				std::complex<double> u = a[i + k];
				std::complex<double> v = a[i + k + len / 2] * w;
				a[i + k] = u + v;
				a[i + k + len / 2] = u - v;
				w *= wl;
			}
		}
	}
}

void spectrum(const std::vector<float> &y, float sr, float &peak_hz, float &centroid_hz)
{
	std::vector<std::complex<double>> buf(kFftSize, {0.0, 0.0});
	int n = std::min((int)y.size(), kFftSize);
	for (int i = 0; i < n; i++) {
		double w = 0.5 - 0.5 * std::cos(2.0 * M_PI * i / (double)(n - 1));
		buf[i] = {y[i] * w, 0.0};
	}
	fft_in_place(buf);

	double best = -1.0, num = 0.0, den = 0.0;
	int best_k = 0;
	for (int k = 1; k < kFftSize / 2; k++) {
		double mag = std::abs(buf[k]);
		double hz = (double)k * sr / kFftSize;
		if (mag > best) { best = mag; best_k = k; }
		num += mag * hz;
		den += mag;
	}
	peak_hz = (float)((double)best_k * sr / kFftSize);
	centroid_hz = (float)(den > 0.0 ? num / den : 0.0);
}

bool analyse(const char *label, const std::vector<float> &y)
{
	const float sr = DRUM_VOICE_SAMPLE_RATE;
	int win = (int)(0.02f * sr);

	bool bad = has_nan_or_inf(y);
	float pk = peak_abs(y);
	float r0 = rms(y, 0, win);
	float r50 = rms(y, (int)(0.05f * sr), win);
	float r150 = rms(y, (int)(0.15f * sr), win);
	float f_peak = 0.0f, f_cen = 0.0f;
	spectrum(y, sr, f_peak, f_cen);

	std::vector<float> env = envelope(y, sr);

	/* The hit must have fully died by the end of the buffer and must not
	 * clip. Absolute level is not a pass criterion: a low Depth setting
	 * legitimately reduces a hat to near-silence. */
	int tail = (int)y.size() - win;
	bool decays  = tail > 0 && rms(y, tail, win) < 0.05f * std::max(r0, 1e-9f);
	bool alive   = (pk > 1e-5f);
	bool no_clip = (pk <= 1.0f);

	printf("  %-24s nan=%-3s peak=%.3f dur=%.3fs  rms 0/50/150ms=%.4f/%.4f/%.4f  "
	       "zcr=%.0fHz  fft peak=%.0fHz cen=%.0fHz  steps=%-2d shape=%-3s %s%s%s\n",
	       label, bad ? "YES" : "no", pk, audible_duration_s(y, sr),
	       r0, r50, r150, zcr_estimate_hz(y, 0, win, sr), f_peak, f_cen,
	       staircase_steps(env), envelope_shape(env),
	       (!bad && alive && decays && no_clip) ? "OK" : "CHECK!",
	       no_clip ? "" : " clip", decays ? "" : " tail");

	return !bad && alive && decays && no_clip;
}

struct VoiceCase {
	const char *name;
	const DrumVoiceOps *ops;
	std::vector<Scenario> scenarios;
};

} /* namespace */

int main(int argc, char **argv)
{
	std::string out_dir = (argc > 1) ? argv[1] : "out";

	/* The "*_default" scenario of each voice is tuned to reproduce the
	 * python reference's own settings (decay scale 1.0, reference noise
	 * period / duty / spacing) so compare_to_python.py has a like-for-like
	 * render to measure against. */
	std::vector<VoiceCase> cases = {
		{"chip_kick", &drum_voice_chip_kick, {
			{"chip_kick_default",     0.0f, 1.0f,  0.231f, 0.5f,  0.5f},
			{"chip_kick_longdecay",   0.0f, 1.0f,  1.0f,   0.5f,  1.2f},
			{"chip_kick_lowfilter",   0.0f, 0.15f, 0.231f, 0.5f,  0.5f},
			{"chip_kick_duty12",      0.0f, 1.0f,  0.231f, 0.0f,  0.5f},
			{"chip_kick_duty75",      0.0f, 1.0f,  0.231f, 1.0f,  0.5f},
		}},
		{"chip_snare", &drum_voice_chip_snare, {
			{"chip_snare_default",    0.0f, 1.0f,  0.231f, 0.28f, 0.4f},
			{"chip_snare_longdecay",  0.0f, 1.0f,  1.0f,   0.28f, 0.8f},
			{"chip_snare_lowfilter",  0.0f, 0.15f, 0.231f, 0.28f, 0.4f},
			{"chip_snare_fastnoise",  0.0f, 1.0f,  0.231f, 0.0f,  0.4f},
			{"chip_snare_slownoise",  0.0f, 1.0f,  0.231f, 1.0f,  0.4f},
		}},
		{"chip_chat", &drum_voice_chip_closed_hat, {
			{"chip_chat_default",     0.0f, 1.0f,  0.167f, 0.15f, 0.3f},
			{"chip_chat_longdecay",   0.0f, 1.0f,  1.0f,   0.15f, 0.6f},
			{"chip_chat_lowfilter",   0.0f, 0.15f, 0.167f, 0.15f, 0.3f},
			{"chip_chat_fastnoise",   0.0f, 1.0f,  0.167f, 0.0f,  0.3f},
			{"chip_chat_slownoise",   0.0f, 1.0f,  0.167f, 1.0f,  0.3f},
		}},
		{"chip_ohat", &drum_voice_chip_open_hat, {
			{"chip_ohat_default",     0.0f, 1.0f,  0.245f, 0.57f, 0.8f},
			{"chip_ohat_longdecay",   0.0f, 1.0f,  1.0f,   0.57f, 1.6f},
			{"chip_ohat_lowfilter",   0.0f, 0.15f, 0.245f, 0.57f, 0.8f},
			{"chip_ohat_longtap",     0.0f, 1.0f,  0.245f, 0.0f,  0.8f},
			{"chip_ohat_metaltap",    0.0f, 1.0f,  0.245f, 1.0f,  0.8f},
		}},
		{"chip_perc", &drum_voice_chip_perc, {
			{"chip_perc_default",     0.0f, 1.0f,  0.231f, 0.4f,  0.3f},
			{"chip_perc_longdecay",   0.0f, 1.0f,  1.0f,   0.4f,  0.8f},
			{"chip_perc_lowfilter",   0.0f, 0.15f, 0.231f, 0.4f,  0.3f},
			{"chip_perc_stacked",     0.0f, 1.0f,  0.231f, 0.0f,  0.3f},
			{"chip_perc_spread",      0.0f, 1.0f,  0.231f, 1.0f,  0.5f},
		}},
		{"chip_cowbell", &drum_voice_chip_cowbell, {
			{"chip_cowbell_default",   0.0f, 1.0f,  0.231f, 0.5f, 0.4f},
			{"chip_cowbell_longdecay", 0.0f, 1.0f,  1.0f,   0.5f, 0.9f},
			{"chip_cowbell_lowfilter", 0.0f, 0.15f, 0.231f, 0.5f, 0.4f},
			{"chip_cowbell_unison",    0.0f, 1.0f,  0.231f, 0.0f, 0.4f},
			{"chip_cowbell_octave",    0.0f, 1.0f,  0.231f, 1.0f, 0.4f},
		}},
	};

	int failures = 0, total = 0;
	for (auto &vc : cases) {
		printf("%s voice:\n", vc.name);
		for (auto &sc : vc.scenarios) {
			std::vector<float> y;
			render_voice(*vc.ops, sc, y);
			total++;
			if (!analyse(sc.label, y)) failures++;
			wav::write_mono_f32(out_dir + "/fw_" + sc.label + ".wav", y,
			                    (uint32_t)DRUM_VOICE_SAMPLE_RATE);
		}
	}

	printf("\n%d/%d scenarios passed. WAVs written under %s/\n", total - failures, total,
	       out_dir.c_str());
	return failures ? 1 : 0;
}
