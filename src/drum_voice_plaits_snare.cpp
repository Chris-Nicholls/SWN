/*
 * drum_voice_plaits_snare.cpp
 *
 * DrumVoiceOps wrapper around Mutable Instruments' plaits::SnareDrumEngine
 * (src/plaits/dsp/engine/snare_drum_engine.cc), used as-is. Like the bass
 * drum engine it renders an 808-style analog model into `out` and a
 * "synthetic" one into `aux`; we keep the analog one and discard `aux`.
 *
 * -----------------------------------------------------------------------------
 */

#include <new>

#include "plaits/dsp/dsp.h"
#include "plaits/dsp/engine/snare_drum_engine.h"

extern "C" {
#include "drum_voice.h"
}

/* See drum_voice_plaits_kick.cpp -- same reasoning for keeping the
 * discarded model's buffer out of the per-voice state. */
static float s_aux_scratch[plaits::kMaxBlockSize];

/* MIDI 52 (E3, ~165 Hz): AnalogSnareDrum builds its shell from two modes
 * at f0 and ~1.47*f0, so 165 Hz puts the pair at roughly the 180/250 Hz
 * of a tuned 808 snare while staying clear of the kick. */
#define PLAITS_SNARE_BASE_NOTE 52.0f

/* Matches the kick's accent so a kit built from both families stays
 * balanced without per-voice output trims. */
#define PLAITS_SNARE_ACCENT 0.8f

/* Plaits' own post-processing gain for this engine (see the
 * RegisterInstance call in plaits/dsp/voice.cc) trimmed down from 0.8:
 * with Depth now driving `timbre` directly instead of a lowpass that
 * could only ever attenuate, some corners of the timbre/morph/harmonics
 * space (e.g. timbre=1, high decay+harmonics) genuinely peak louder
 * than before, so this needs a bit more headroom than the other two
 * Plaits voices. */
#define PLAITS_SNARE_OUT_GAIN 0.7f

typedef struct {
	plaits::SnareDrumEngine engine;

	float pitch;       /* semitones from PLAITS_SNARE_BASE_NOTE */
	float timbre01;    /* -> EngineParameters.timbre, Depth */
	float decay01;     /* -> EngineParameters.morph, Latitude */
	float other01;     /* -> EngineParameters.harmonics, Longitude */
	int   pending_trigger;
} PlaitsSnareState;

extern "C" {

static void plaits_snare_init(void *state_v)
{
	PlaitsSnareState *st = (PlaitsSnareState *)state_v;

	/* Placement-construct into the caller's opaque buffer; every other
	 * field below is assigned outright, so no memset is needed (and
	 * memset over a non-trivially-copyable C++ object would be wrong). */
	new (&st->engine) plaits::SnareDrumEngine();
	/* SnareDrumEngine::Init() ignores the allocator (no scratch of its
	 * own), so a null one is safe. */
	st->engine.Init(NULL);

	st->pitch           = 0.0f;
	st->timbre01        = 0.5f;
	st->decay01         = 0.4f;
	st->other01         = 0.5f;
	st->pending_trigger = 0;
}

static void plaits_snare_trigger(void *state_v, float pitch)
{
	PlaitsSnareState *st = (PlaitsSnareState *)state_v;

	st->pitch           = pitch;
	st->pending_trigger = 1;
}

static void plaits_snare_render(void *state_v, float *out, int n)
{
	PlaitsSnareState *st = (PlaitsSnareState *)state_v;

	plaits::EngineParameters p;
	p.note      = PLAITS_SNARE_BASE_NOTE + st->pitch;
	p.timbre    = st->timbre01;
	p.morph     = st->decay01;
	p.harmonics = st->other01;
	p.accent    = PLAITS_SNARE_ACCENT;

	int done = 0;
	while (done < n) {
		int chunk = n - done;
		if (chunk > (int)plaits::kMaxBlockSize)
			chunk = (int)plaits::kMaxBlockSize;

		/* Rising edge on the first chunk only, else the envelope
		 * restarts every 48 samples. */
		p.trigger = st->pending_trigger ? plaits::TRIGGER_RISING_EDGE
		                                : plaits::TRIGGER_LOW;
		st->pending_trigger = 0;

		bool already_enveloped = false;
		st->engine.Render(p, out + done, s_aux_scratch, (size_t)chunk,
		                  &already_enveloped);

		done += chunk;
	}

	for (int i = 0; i < n; i++)
		out[i] *= PLAITS_SNARE_OUT_GAIN;
}

/* Depth -> Plaits' own `timbre` directly (see drum_voice_plaits_kick.cpp
 * for why this family skips the shared post-voice filter): the
 * shell-vs-snare-buzz balance. 0.5 is an even split -- audibly a snare
 * rather than a tom or a noise burst. */
static void plaits_snare_set_filter(void *state_v, float cutoff01)
{
	PlaitsSnareState *st = (PlaitsSnareState *)state_v;
	if (cutoff01 < 0.0f) cutoff01 = 0.0f;
	if (cutoff01 > 1.0f) cutoff01 = 1.0f;
	st->timbre01 = cutoff01;
}

/* "decay" -> morph: AnalogSnareDrum's morph is the shell/noise decay
 * time, the knob's intended meaning. */
static void plaits_snare_set_decay(void *state_v, float decay01)
{
	PlaitsSnareState *st = (PlaitsSnareState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay01 = decay01;
}

/* "other" -> harmonics: the snare/shell mix. Sweeping it takes the voice
 * from a bare tuned shell (rimshot-ish) to almost pure snare rattle,
 * which is the widest character change the engine offers. */
static void plaits_snare_set_other(void *state_v, float other01)
{
	PlaitsSnareState *st = (PlaitsSnareState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->other01 = other01;
}

const DrumVoiceOps drum_voice_plaits_snare = {
	plaits_snare_init,
	plaits_snare_trigger,
	plaits_snare_render,
	plaits_snare_set_filter,
	plaits_snare_set_decay,
	plaits_snare_set_other,
	sizeof(PlaitsSnareState),
};

}  /* extern "C" */
