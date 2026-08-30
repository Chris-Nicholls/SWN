/*
 * drum_voice_plaits_hihat.cpp
 *
 * DrumVoiceOps wrapper around Mutable Instruments' 808 hi-hat model
 * (src/plaits/dsp/drums/hi_hat.h), used as-is.
 *
 * We drive the HiHat<> template directly instead of going through
 * plaits::HiHatEngine: that engine instantiates a second, RingModNoise
 * variant purely to fill its `aux` output, and needs a BufferAllocator to
 * hand it a scratch buffer -- together more than drum_ui.h's 512-byte
 * DRUM_VOICE_STATE_BYTES budget allows. The instantiation below is the one
 * HiHatEngine renders into `out`, i.e. the faithful 808.
 *
 * Registered under both the closed- and open-hat categories: with this
 * engine the two are genuinely the same circuit at different decay
 * settings, so the Decay knob is what picks the character.
 *
 * -----------------------------------------------------------------------------
 */

#include <new>

#include "plaits/dsp/dsp.h"
#include "plaits/dsp/drums/hi_hat.h"
#include "plaits/dsp/engine/engine.h"

extern "C" {
#include "drum_voice.h"
}

/* HiHat::Render() wants two caller-supplied scratch buffers. They are dead
 * as soon as it returns and channels render one at a time, so keeping them
 * file-static rather than per-voice is what fits us under the 512-byte
 * per-voice state budget. */
static float s_temp_1[plaits::kMaxBlockSize];
static float s_temp_2[plaits::kMaxBlockSize];

/* The six square oscillators are driven at 2*f0, and their ratio table is
 * normalised around the 808's 414 Hz top oscillator -- so the authentic
 * tuning is f0 ~ 207 Hz. MIDI 60 (~262 Hz) sits a fifth above that, which
 * keeps the hat bright and clearly separated from the snare while leaving
 * plenty of downward range on the pitch offset to reach stock 808. */
#define PLAITS_HIHAT_BASE_NOTE 60.0f

/* Same accent as the plaits kick/snare so the family stays level-matched. */
#define PLAITS_HIHAT_ACCENT 0.8f

/* Plaits' own post-processing gain for this engine (see the
 * RegisterInstance call in plaits/dsp/voice.cc). */
#define PLAITS_HIHAT_OUT_GAIN 0.8f

typedef plaits::HiHat<plaits::SquareNoise, plaits::SwingVCA, true, false> PlaitsHiHat808;

typedef struct {
	PlaitsHiHat808 hi_hat;

	float pitch;       /* semitones from PLAITS_HIHAT_BASE_NOTE */
	float timbre01;    /* -> HiHat::Render() tone, Depth */
	float decay01;     /* -> HiHat::Render() decay, Latitude */
	float other01;     /* -> HiHat::Render() noisiness, Longitude */
	int   pending_trigger;
} PlaitsHiHatState;

extern "C" {

static void plaits_hihat_init(void *state_v)
{
	PlaitsHiHatState *st = (PlaitsHiHatState *)state_v;

	/* Placement-construct into the caller's opaque buffer; every other
	 * field below is assigned outright, so no memset is needed (and
	 * memset over a non-trivially-copyable C++ object would be wrong). */
	new (&st->hi_hat) PlaitsHiHat808();
	st->hi_hat.Init();

	st->pitch           = 0.0f;
	st->timbre01        = 0.7f;
	st->decay01         = 0.25f;
	st->other01         = 0.3f;
	st->pending_trigger = 0;
}

static void plaits_hihat_trigger(void *state_v, float pitch)
{
	PlaitsHiHatState *st = (PlaitsHiHatState *)state_v;

	st->pitch           = pitch;
	st->pending_trigger = 1;
}

static void plaits_hihat_render(void *state_v, float *out, int n)
{
	PlaitsHiHatState *st = (PlaitsHiHatState *)state_v;

	const float f0 = plaits::NoteToFrequency(PLAITS_HIHAT_BASE_NOTE + st->pitch);

	int done = 0;
	while (done < n) {
		int chunk = n - done;
		if (chunk > (int)plaits::kMaxBlockSize)
			chunk = (int)plaits::kMaxBlockSize;

		/* Rising edge on the first chunk only, else the envelope
		 * restarts every 48 samples. */
		const bool trigger = st->pending_trigger != 0;
		st->pending_trigger = 0;

		st->hi_hat.Render(
			false,               /* sustain: we are always one-shot triggered */
			trigger,
			PLAITS_HIHAT_ACCENT,
			f0,
			st->timbre01,
			st->decay01,
			st->other01,
			s_temp_1,
			s_temp_2,
			out + done,
			(size_t)chunk);

		done += chunk;
	}

	for (int i = 0; i < n; i++)
		out[i] *= PLAITS_HIHAT_OUT_GAIN;
}

/* Depth -> Plaits' own `tone` directly (see drum_voice_plaits_kick.cpp
 * for why this family skips the shared post-voice filter): sets both
 * the colouring band-pass and the output high-pass at
 * 150 Hz * 2^(6*tone). 0.7 puts them around 2.7 kHz: a hat rather than
 * a cowbell, without the fizz of a fully open filter. */
static void plaits_hihat_set_filter(void *state_v, float cutoff01)
{
	PlaitsHiHatState *st = (PlaitsHiHatState *)state_v;
	if (cutoff01 < 0.0f) cutoff01 = 0.0f;
	if (cutoff01 > 1.0f) cutoff01 = 1.0f;
	st->timbre01 = cutoff01;
}

/* "decay" -> HiHat's decay: this is the only knob separating a closed hat
 * from an open one on the real circuit, which is why the same voice is
 * registered under both categories. */
static void plaits_hihat_set_decay(void *state_v, float decay01)
{
	PlaitsHiHatState *st = (PlaitsHiHatState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay01 = decay01;
}

/* "other" -> noisiness: crossfades the 6-oscillator metallic noise towards
 * clocked white noise. Not part of the 808 at all, but it is what turns
 * this from a hat into a shaker/cymbal, so it earns the character knob. */
static void plaits_hihat_set_other(void *state_v, float other01)
{
	PlaitsHiHatState *st = (PlaitsHiHatState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->other01 = other01;
}

const DrumVoiceOps drum_voice_plaits_hihat = {
	plaits_hihat_init,
	plaits_hihat_trigger,
	plaits_hihat_render,
	plaits_hihat_set_filter,
	plaits_hihat_set_decay,
	plaits_hihat_set_other,
	sizeof(PlaitsHiHatState),
};

}  /* extern "C" */
