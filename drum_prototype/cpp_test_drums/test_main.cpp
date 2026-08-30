/*
 * test_main.cpp — drives the mpump-style kick/snare DrumVoiceOps
 * through a few filter/decay/other settings, writes WAVs into
 * out/, and prints a rough sanity check (peak sample-block RMS,
 * envelope decay duration, zero-crossing-rate frequency estimate)
 * so a firmware port bug (silence, NaN, runaway filter) shows up
 * without needing to eyeball a spectrogram.
 *
 * Usage:
 *   ./drumtest [out_dir]
 */
extern "C" {
#include "drum_voice.h"
}

#include "wav_io.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

constexpr int kBlockSize = 64;

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

/* Zero-crossing-rate frequency estimate over a short window --
 * crude, but enough to sanity-check "is the kick body in the tens
 * of Hz and the snare noise broadband" without a full FFT. */
float zcr_estimate_hz(const std::vector<float> &y, int start, int n, float sr)
{
	int crossings = 0;
	for (int i = start + 1; i < start + n && i < (int)y.size(); i++) {
		if ((y[i - 1] < 0.0f) != (y[i] < 0.0f)) crossings++;
	}
	float seconds = (float)n / sr;
	return (crossings / 2.0f) / seconds;
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

void analyse(const char *label, const std::vector<float> &y)
{
	const float sr = DRUM_VOICE_SAMPLE_RATE;
	int win = (int)(0.02f * sr);   /* 20ms analysis windows */

	printf("  %-28s nan/inf=%s  ", label, has_nan_or_inf(y) ? "YES(!)" : "no");
	printf("rms@0ms=%.4f rms@50ms=%.4f rms@150ms=%.4f  zcr@0ms=%.0fHz zcr@50ms=%.0fHz\n",
	       rms(y, 0, win),
	       rms(y, (int)(0.05f * sr), win),
	       rms(y, (int)(0.15f * sr), win),
	       zcr_estimate_hz(y, 0, win, sr),
	       zcr_estimate_hz(y, (int)(0.05f * sr), win, sr));
}

} /* namespace */

int main(int argc, char **argv)
{
	std::string out_dir = (argc > 1) ? argv[1] : "out";

	std::vector<Scenario> kick_scenarios = {
		{"kick_default",    0.0f, 1.0f, 0.3f, 0.15f, 0.6f},
		{"kick_longdecay",  0.0f, 1.0f, 1.0f, 0.15f, 1.2f},
		{"kick_lowfilter",  0.0f, 0.15f, 0.3f, 0.15f, 0.6f},
		{"kick_hiclick",   -2.0f, 1.0f, 0.3f, 1.0f,  0.6f},
	};

	std::vector<Scenario> snare_scenarios = {
		{"snare_default",   0.0f, 1.0f, 0.3f, 0.55f, 0.5f},
		{"snare_longdecay", 0.0f, 1.0f, 1.0f, 0.55f, 1.0f},
		{"snare_lowfilter", 0.0f, 0.15f, 0.3f, 0.55f, 0.5f},
		{"snare_allnoise",  0.0f, 1.0f, 0.3f, 1.0f,  0.5f},
		{"snare_alltone",   0.0f, 1.0f, 0.3f, 0.0f,  0.5f},
	};

	printf("kick voice:\n");
	for (auto &sc : kick_scenarios) {
		std::vector<float> y;
		render_voice(drum_voice_mpump_kick, sc, y);
		analyse(sc.label, y);
		wav::write_mono_f32(out_dir + "/fw_" + sc.label + ".wav", y, (uint32_t)DRUM_VOICE_SAMPLE_RATE);
	}

	printf("snare voice:\n");
	for (auto &sc : snare_scenarios) {
		std::vector<float> y;
		render_voice(drum_voice_mpump_snare, sc, y);
		analyse(sc.label, y);
		wav::write_mono_f32(out_dir + "/fw_" + sc.label + ".wav", y, (uint32_t)DRUM_VOICE_SAMPLE_RATE);
	}

	printf("done. WAVs written under %s/\n", out_dir.c_str());
	return 0;
}
