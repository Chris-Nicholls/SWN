/*
 * test_main.cpp — sanity harness for the Plaits drum-voice wrappers
 * (plaits kick, snare, hi-hat). Renders each voice at its default settings
 * plus decay and "other" extremes and a low-filter setting, writes WAVs into
 * out/, and checks each result for NaN/Inf, non-silence, no clipping and a
 * decaying tail.
 *
 * Unlike the other families there is no python reference to compare against:
 * these are the real Mutable Instruments engines rather than a
 * reimplementation, so a spectral comparison would only be comparing them
 * with themselves.
 *
 * The render block size is deliberately 64 (> plaits::kMaxBlockSize == 48)
 * so the wrappers' chunking loop is exercised on every call.
 *
 * Usage:
 *   ./drumtest_plaits [out_dir]
 */
extern "C" {
#include "drum_voice.h"
}

#include "wav_io.hpp"

#include <algorithm>
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

bool analyse(const char *label, const std::vector<float> &y)
{
	const float sr = DRUM_VOICE_SAMPLE_RATE;
	int win = (int)(0.02f * sr);

	bool bad = has_nan_or_inf(y);
	float pk = peak_abs(y);
	float r0 = rms(y, 0, win);
	float r50 = rms(y, (int)(0.05f * sr), win);
	float r150 = rms(y, (int)(0.15f * sr), win);

	/* The hit must have essentially died by the end of the buffer and must
	 * not clip. Absolute level is not a pass criterion: a low Depth
	 * setting legitimately reduces a hat to near-silence. */
	int tail = (int)y.size() - win;
	bool decays  = tail > 0 && rms(y, tail, win) < 0.05f * std::max(r0, 1e-9f);
	bool alive   = (pk > 1e-4f);
	bool no_clip = (pk <= 1.0f);

	printf("  %-24s nan=%-3s peak=%.3f dur=%.3fs  rms 0/50/150ms=%.4f/%.4f/%.4f  %s%s%s%s\n",
	       label, bad ? "YES" : "no", pk, audible_duration_s(y, sr),
	       r0, r50, r150,
	       (!bad && alive && decays && no_clip) ? "OK" : "CHECK!",
	       no_clip ? "" : " clip", decays ? "" : " tail", alive ? "" : " silent");

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
		{"plaits kick", &drum_voice_plaits_kick, {
			/*  label                pitch  filt   decay  other  secs */
			{"pl_kick_default",       0.0f, 1.0f,  0.5f,  0.3f,  1.5f},
			{"pl_kick_shortdecay",    0.0f, 1.0f,  0.0f,  0.3f,  1.5f},
			{"pl_kick_longdecay",     0.0f, 1.0f,  1.0f,  0.3f,  6.0f},
			{"pl_kick_lowfilter",     0.0f, 0.15f, 0.5f,  0.3f,  1.5f},
			{"pl_kick_clean",         0.0f, 1.0f,  0.5f,  0.0f,  1.5f},
			{"pl_kick_dirty",         0.0f, 1.0f,  0.5f,  1.0f,  1.5f},
			{"pl_kick_up12",         12.0f, 1.0f,  0.5f,  0.3f,  1.5f},
		}},
		{"plaits snare", &drum_voice_plaits_snare, {
			{"pl_snare_default",      0.0f, 1.0f,  0.4f,  0.5f,  1.5f},
			{"pl_snare_shortdecay",   0.0f, 1.0f,  0.0f,  0.5f,  1.5f},
			{"pl_snare_longdecay",    0.0f, 1.0f,  1.0f,  0.5f,  6.0f},
			{"pl_snare_lowfilter",    0.0f, 0.15f, 0.4f,  0.5f,  1.5f},
			{"pl_snare_shell",        0.0f, 1.0f,  0.4f,  0.0f,  1.5f},
			{"pl_snare_rattle",       0.0f, 1.0f,  0.4f,  1.0f,  1.5f},
			{"pl_snare_down7",       -7.0f, 1.0f,  0.4f,  0.5f,  1.5f},
		}},
		{"plaits hihat", &drum_voice_plaits_hihat, {
			{"pl_chat_default",       0.0f, 1.0f,  0.25f, 0.3f,  1.5f},
			{"pl_chat_tight",         0.0f, 1.0f,  0.0f,  0.3f,  1.5f},
			{"pl_ohat_open",          0.0f, 1.0f,  0.6f,  0.3f,  4.0f},
			{"pl_ohat_longdecay",     0.0f, 1.0f,  1.0f,  0.3f, 12.0f},
			{"pl_chat_lowfilter",     0.0f, 0.15f, 0.25f, 0.3f,  1.5f},
			{"pl_chat_metallic",      0.0f, 1.0f,  0.25f, 0.0f,  1.5f},
			{"pl_chat_noisy",         0.0f, 1.0f,  0.25f, 1.0f,  1.5f},
			{"pl_chat_up12",         12.0f, 1.0f,  0.25f, 0.3f,  1.5f},
		}},
	};

	printf("state sizes: kick=%zu snare=%zu hihat=%zu bytes (budget 512)\n\n",
	       drum_voice_plaits_kick.state_size,
	       drum_voice_plaits_snare.state_size,
	       drum_voice_plaits_hihat.state_size);

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

	if (drum_voice_plaits_kick.state_size > 512 ||
	    drum_voice_plaits_snare.state_size > 512 ||
	    drum_voice_plaits_hihat.state_size > 512) {
		printf("\nFAIL: a voice exceeds DRUM_VOICE_STATE_BYTES (512)\n");
		failures++;
	}

	printf("\n%d/%d scenarios passed. WAVs written under %s/\n", total - failures, total,
	       out_dir.c_str());
	return failures ? 1 : 0;
}
