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
#include <stdint.h>

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

/* mpump-style family. */
extern const DrumVoiceOps drum_voice_mpump_kick;
extern const DrumVoiceOps drum_voice_mpump_snare;
extern const DrumVoiceOps drum_voice_mpump_closed_hat;
extern const DrumVoiceOps drum_voice_mpump_open_hat;
extern const DrumVoiceOps drum_voice_mpump_crash;
extern const DrumVoiceOps drum_voice_mpump_ride;
extern const DrumVoiceOps drum_voice_mpump_rimshot;
extern const DrumVoiceOps drum_voice_mpump_tom;
extern const DrumVoiceOps drum_voice_mpump_cowbell;
extern const DrumVoiceOps drum_voice_mpump_clap;

/* Deluge-style family (Phase 2: kick, snare, closed/open hat, cowbell; cymbal TODO). */
extern const DrumVoiceOps drum_voice_deluge_kick;
extern const DrumVoiceOps drum_voice_deluge_snare;
extern const DrumVoiceOps drum_voice_deluge_closed_hat;
extern const DrumVoiceOps drum_voice_deluge_open_hat;
extern const DrumVoiceOps drum_voice_deluge_cowbell;

/* Chiptune (NES APU) family. */
extern const DrumVoiceOps drum_voice_chip_kick;
extern const DrumVoiceOps drum_voice_chip_snare;
extern const DrumVoiceOps drum_voice_chip_closed_hat;
extern const DrumVoiceOps drum_voice_chip_open_hat;
extern const DrumVoiceOps drum_voice_chip_perc;
extern const DrumVoiceOps drum_voice_chip_cowbell;

/* ROLLER-style family. */
extern const DrumVoiceOps drum_voice_roller_kick;
extern const DrumVoiceOps drum_voice_roller_snare;
extern const DrumVoiceOps drum_voice_roller_closed_hat;
extern const DrumVoiceOps drum_voice_roller_open_hat;
extern const DrumVoiceOps drum_voice_roller_ride;
extern const DrumVoiceOps drum_voice_roller_crash;
extern const DrumVoiceOps drum_voice_roller_perc;
extern const DrumVoiceOps drum_voice_roller_rimshot;

/* Plaits (Mutable Instruments) drum engines. */
extern const DrumVoiceOps drum_voice_plaits_kick;
extern const DrumVoiceOps drum_voice_plaits_snare;
extern const DrumVoiceOps drum_voice_plaits_hihat;

/* Fixed per-channel roles, one category per channel (A=Kick .. F=Other)
 * -- see the "grouped by type" mapping in drum_ui.c's kMvpKit/category
 * table. LFO shape cycles a channel through only its own category's
 * voices, so it always stays "a kick" (etc.) no matter how far you
 * turn it. DRUM_CAT_OTHER is the catch-all for anything that isn't
 * one of the other five (rimshot, tom, cowbell, clap, ride, perc). */
typedef enum DrumVoiceCategory {
	DRUM_CAT_KICK,
	DRUM_CAT_SNARE,
	DRUM_CAT_CLOSED_HAT,
	DRUM_CAT_OPEN_HAT,
	DRUM_CAT_CRASH,
	DRUM_CAT_OTHER,
	NUM_DRUM_CATEGORIES,
} DrumVoiceCategory;

/* All voices, in a fixed, append-only order -- this is what a saved
 * preset's per-channel voice byte indexes, so it survives a rebuild
 * (a raw DrumVoiceOps* would not: it's relink-dependent). Add new
 * voices only at the end; never reorder or remove an entry, or old
 * presets will silently load the wrong sound. `category` is pure
 * metadata (not part of the index), safe to edit freely. */
typedef struct DrumVoiceEntry {
	const DrumVoiceOps *ops;
	const char *name;
	DrumVoiceCategory category;
} DrumVoiceEntry;

extern const DrumVoiceEntry kDrumVoiceRegistry[];
extern const uint8_t kNumDrumVoices;

/* -1 if ops is NULL or not found in the registry (shouldn't happen for
 * any ops actually in use, but callers serializing a preset should
 * treat -1 the same as "silent channel" rather than faulting). */
int8_t drum_voice_registry_index(const DrumVoiceOps *ops);

/* NULL if index is out of range (e.g. a preset saved by a future
 * firmware with more voices, loaded on an older build). */
const DrumVoiceOps *drum_voice_registry_lookup(uint8_t index);
