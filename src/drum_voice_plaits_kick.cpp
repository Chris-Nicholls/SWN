/*
 * drum_voice_plaits_kick.cpp
 *
 * DrumVoiceOps wrapper around Mutable Instruments' plaits::BassDrumEngine
 * (src/plaits/dsp/engine/bass_drum_engine.cc), used as-is. The engine
 * renders two models per call -- an 808-style analog one into `out` and a
 * "synthetic" one into `aux`; we keep the analog one and throw `aux` away.
 *
 * -----------------------------------------------------------------------------
 */

#include <new>

#include "plaits/dsp/dsp.h"
#include "plaits/dsp/engine/bass_drum_engine.h"

extern "C" {
#include "drum_voice.h"
}

/* The discarded "synthetic" model still has to be rendered somewhere.
 * File-static rather than per-voice state because it is dead the moment
 * Render() returns, and the audio path renders channels one at a time --
 * keeping it out of the struct is what leaves us room under drum_ui.h's
 * 512-byte DRUM_VOICE_STATE_BYTES budget. */
static float s_aux_scratch[plaits::kMaxBlockSize];

/* MIDI 36 (C2, ~65 Hz): the conventional 808/909 kick fundamental, and
 * low enough that BassDrumEngine's `drive` term (which fades out above
 * f0 ~ 1/16 of the sample rate) still has authority. */
#define PLAITS_KICK_BASE_NOTE 36.0f

/* Plaits scales both level and punch by accent. 0.8 keeps the transient
 * hard without pushing the engine's internal overdrive into permanent
 * saturation, leaving headroom for set_other to add drive deliberately. */
#define PLAITS_KICK_ACCENT 0.8f

/* Plaits' own post-processing gain for this engine (see the
 * RegisterInstance call in plaits/dsp/voice.cc). */
#define PLAITS_KICK_OUT_GAIN 0.8f

typedef struct {
	plaits::BassDrumEngine engine;

	float pitch;       /* semitones from PLAITS_KICK_BASE_NOTE */
	float timbre01;    /* -> EngineParameters.timbre, Depth */
	float decay01;     /* -> EngineParameters.morph, Latitude */
	float other01;     /* -> EngineParameters.harmonics, Longitude */
	int   pending_trigger;
} PlaitsKickState;

extern "C" {

static void plaits_kick_init(void *state_v)
{
	PlaitsKickState *st = (PlaitsKickState *)state_v;

	/* Placement-construct into the caller's opaque buffer; every other
	 * field below is assigned outright, so no memset is needed (and
	 * memset over a non-trivially-copyable C++ object would be wrong). */
	new (&st->engine) plaits::BassDrumEngine();
	/* BassDrumEngine::Init() never touches the allocator (it owns no
	 * scratch buffers), so a null one is safe here. */
	st->engine.Init(NULL);

	st->pitch           = 0.0f;
	st->timbre01        = 0.5f;
	st->decay01         = 0.5f;
	st->other01         = 0.3f;
	st->pending_trigger = 0;
}

static void plaits_kick_trigger(void *state_v, float pitch)
{
	PlaitsKickState *st = (PlaitsKickState *)state_v;

	st->pitch           = pitch;
	st->pending_trigger = 1;
}

static void plaits_kick_render(void *state_v, float *out, int n)
{
	PlaitsKickState *st = (PlaitsKickState *)state_v;

	plaits::EngineParameters p;
	p.note      = PLAITS_KICK_BASE_NOTE + st->pitch;
	p.timbre    = st->timbre01;
	p.morph     = st->decay01;
	p.harmonics = st->other01;
	p.accent    = PLAITS_KICK_ACCENT;

	int done = 0;
	while (done < n) {
		int chunk = n - done;
		if (chunk > (int)plaits::kMaxBlockSize)
			chunk = (int)plaits::kMaxBlockSize;

		/* The rising edge belongs to one chunk only; holding it would
		 * retrigger the envelope every 48 samples. */
		p.trigger = st->pending_trigger ? plaits::TRIGGER_RISING_EDGE
		                                : plaits::TRIGGER_LOW;
		st->pending_trigger = 0;

		bool already_enveloped = false;
		st->engine.Render(p, out + done, s_aux_scratch, (size_t)chunk,
		                  &already_enveloped);

		done += chunk;
	}

	for (int i = 0; i < n; i++)
		out[i] *= PLAITS_KICK_OUT_GAIN;
}

/* Depth -> Plaits' own `timbre` directly, rather than the shared
 * post-voice SVF every other voice family uses -- for these three
 * Plaits engines, Depth/Latitude/Longitude map straight onto the
 * engine's native timbre/morph/harmonics, matching how the real
 * hardware's three main knobs work. Feeds AnalogBassDrum's `tone`
 * (attack/body brightness). */
static void plaits_kick_set_filter(void *state_v, float cutoff01)
{
	PlaitsKickState *st = (PlaitsKickState *)state_v;
	if (cutoff01 < 0.0f) cutoff01 = 0.0f;
	if (cutoff01 > 1.0f) cutoff01 = 1.0f;
	st->timbre01 = cutoff01;
}

/* "decay" -> morph: in BassDrumEngine morph is the body decay time of
 * both models, which is exactly the knob's intended meaning. */
static void plaits_kick_set_decay(void *state_v, float decay01)
{
	PlaitsKickState *st = (PlaitsKickState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay01 = decay01;
}

/* "other" -> harmonics: the engine fans this one parameter out into
 * attack FM, self FM and overdrive in sequence, so a single 0..1 sweep
 * walks the kick from clean 808 to distorted 909 -- the most characterful
 * knob available. */
static void plaits_kick_set_other(void *state_v, float other01)
{
	PlaitsKickState *st = (PlaitsKickState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->other01 = other01;
}

const DrumVoiceOps drum_voice_plaits_kick = {
	plaits_kick_init,
	plaits_kick_trigger,
	plaits_kick_render,
	plaits_kick_set_filter,
	plaits_kick_set_decay,
	plaits_kick_set_other,
	sizeof(PlaitsKickState),
};

}  /* extern "C" */
