/*
 * test_main.cpp — drives the Deluge-style kick/snare/closed-hat/
 * open-hat/cowbell DrumVoiceOps through default, long-decay,
 * low-filter and extreme-"other" settings, writes WAVs into out/, and
 * prints sanity checks (NaN/Inf scan, peak, RMS envelope, ZCR estimate,
 * plus an FFT peak/centroid so the result can be compared against the
 * python reference renders in drum_prototype/).
 *
 * Usage:
 *   ./drumtest_deluge [out_dir]
 */
extern "C" {
#include "drum_voice.h"

/* Not declared in drum_voice.h yet -- these voices are deliberately not
 * wired into the firmware's voice table until they have been verified. */
extern const DrumVoiceOps drum_voice_deluge_kick;
extern const DrumVoiceOps drum_voice_deluge_snare;
extern const DrumVoiceOps drum_voice_deluge_closed_hat;
extern const DrumVoiceOps drum_voice_deluge_open_hat;
extern const DrumVoiceOps drum_voice_deluge_cowbell;
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

/* Length of the audible portion: first to last sample above 1% of peak. */
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

/* Peak-magnitude bin and spectral centroid over the hit's onset --
 * enough to compare against the python reference renders. */
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

	/* The hit must have fully died by the end of the buffer, and must not
	 * clip. Level itself is not a pass criterion: a low Depth setting
	 * legitimately reduces a hat to near-silence. */
	int tail = (int)y.size() - win;
	bool decays  = tail > 0 && rms(y, tail, win) < 0.05f * std::max(r0, 1e-9f);
	bool alive   = (pk > 1e-5f);
	bool no_clip = (pk <= 1.0f);

	printf("  %-24s nan=%-3s peak=%.3f dur=%.3fs  rms 0/50/150ms=%.4f/%.4f/%.4f  "
	       "zcr=%.0fHz  fft peak=%.0fHz cen=%.0fHz  %s%s%s\n",
	       label, bad ? "YES" : "no", pk, audible_duration_s(y, sr),
	       r0, r50, r150, zcr_estimate_hz(y, 0, win, sr), f_peak, f_cen,
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

	std::vector<VoiceCase> cases = {
		{"kick", &drum_voice_deluge_kick, {
			{"kick_default",    0.0f, 1.0f,  0.2f, 0.27f, 1.0f},
			{"kick_longdecay",  0.0f, 1.0f,  1.0f, 0.27f, 3.0f},
			{"kick_lowfilter",  0.0f, 0.15f, 0.2f, 0.27f, 1.0f},
			{"kick_maxdrop",   -2.0f, 1.0f,  0.2f, 1.0f,  1.0f},
			{"kick_nodrop",     0.0f, 1.0f,  0.2f, 0.0f,  1.0f},
		}},
		{"snare", &drum_voice_deluge_snare, {
			{"snare_default",   0.0f, 1.0f,  0.12f, 0.5f, 0.8f},
			{"snare_longdecay", 0.0f, 1.0f,  1.0f,  0.5f, 3.0f},
			{"snare_lowfilter", 0.0f, 0.15f, 0.12f, 0.5f, 0.8f},
			{"snare_allnoise",  0.0f, 1.0f,  0.12f, 1.0f, 0.8f},
			{"snare_alltone",   0.0f, 1.0f,  0.12f, 0.0f, 0.8f},
		}},
		{"closed_hat", &drum_voice_deluge_closed_hat, {
			{"chat_default",    0.0f, 1.0f,  0.1f, 0.62f, 0.4f},
			{"chat_longdecay",  0.0f, 1.0f,  1.0f, 0.62f, 1.0f},
			{"chat_lowfilter",  0.0f, 0.15f, 0.1f, 0.62f, 0.4f},
			{"chat_dark",       0.0f, 1.0f,  0.1f, 0.0f,  0.4f},
			{"chat_bright",     0.0f, 1.0f,  0.1f, 1.0f,  0.4f},
		}},
		{"open_hat", &drum_voice_deluge_open_hat, {
			{"ohat_default",    0.0f, 1.0f,  0.18f, 0.5f, 1.0f},
			{"ohat_longdecay",  0.0f, 1.0f,  1.0f,  0.5f, 3.0f},
			{"ohat_lowfilter",  0.0f, 0.15f, 0.18f, 0.5f, 1.0f},
			{"ohat_dark",       0.0f, 1.0f,  0.18f, 0.0f, 1.0f},
			{"ohat_bright",     0.0f, 1.0f,  0.18f, 1.0f, 1.0f},
		}},
		{"cowbell", &drum_voice_deluge_cowbell, {
			{"cowbell_default",   0.0f, 1.0f,  0.19f, 0.48f, 0.6f},
			{"cowbell_longdecay", 0.0f, 1.0f,  1.0f,  0.48f, 2.0f},
			{"cowbell_lowfilter", 0.0f, 0.15f, 0.19f, 0.48f, 0.6f},
			{"cowbell_lowindex",  0.0f, 1.0f,  0.19f, 0.0f,  0.6f},
			{"cowbell_maxindex", 12.0f, 1.0f,  0.19f, 1.0f,  0.6f},
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
