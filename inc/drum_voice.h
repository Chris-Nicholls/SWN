/*
 * drum_voice.h
 *
 * Uniform vtable interface for drum-voice DSP algorithms (see the
 * "New drum-voice abstraction" section of the drum-station plan).
 * Pure standard C, no STM32/HAL dependency, so voice algorithms can
 * be built and tested on a host machine before hardware wiring.
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

#include <stddef.h>

/* Every ported family renders at this rate; the mpump port derives
 * all its exp()/sin() time constants from it directly. */
#define DRUM_VOICE_SAMPLE_RATE 48000.0f

typedef struct DrumVoiceOps {
	void (*init)(void *state);                           /* sets sane defaults on freshly allocated (state_size)-byte storage; call once before any other op */
	void (*trigger)(void *state, float pitch);          /* pitch: semitone offset from the voice's nominal tuning */
	void (*render)(void *state, float *out, int n);      /* fills out[0..n) with mono samples, accumulates internal time */
	void (*set_filter)(void *state, float cutoff01);     /* forwards into the voice's shared post-filter, see drum_shared_filter.h */
	void (*set_decay)(void *state, float decay01);       /* 0..1 -> voice-specific decay/release mapping */
	void (*set_other)(void *state, float other01);       /* 0..1 -> voice-specific character knob, see per-voice .c comment */
	size_t state_size;                                   /* bytes to allocate for the opaque state passed to the above */
} DrumVoiceOps;

/* mpump-style family (Phase 1: kick + snare only). */
extern const DrumVoiceOps drum_voice_mpump_kick;
extern const DrumVoiceOps drum_voice_mpump_snare;
