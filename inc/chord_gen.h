#pragma once

#include <stm32f7xx.h>
#include "globals.h"

// Algorithm constants for harmonic dissonance model
#define CHORD_PEAK_SEMITONES_C2     0.8f
#define CHORD_PEAK_SEMITONES_C6     0.5f
#define CHORD_TAIL_HALFWIDTH_C2     2.35f
#define CHORD_TAIL_HALFWIDTH_C6     1.4f
#define CHORD_SEARCH_HARMONICS      7
#define CHORD_NUM_OVERTONES         7    // fundamental + 6 overtones from sliders

#define C2_FREQ_HZ  65.4064f
#define C6_FREQ_HZ  1046.50f

// Scale masks: 12-bit mask where bit N = 1 means semitone N is in scale (C=0, C#=1, etc.)
// These are relative to C; use rotate_scale_mask() to transpose to a different key.
#define SCALE_MASK_CHROMATIC        0xFFF           // All 12 notes
#define SCALE_MASK_MAJOR            0xAB5           // C D E F G A B = bits 0,2,4,5,7,9,11
#define SCALE_MASK_MINOR_HARMONIC   0x9AD           // C D Eb F G Ab B = bits 0,2,3,5,7,8,11
#define SCALE_MASK_SEMITONES        0xFFF           // Same as chromatic (all notes)

// Default scale penalty (dissonance units added for out-of-scale notes)
#define CHORD_SCALE_PENALTY_DEFAULT 0.5f

// Boundary penalty: dissonance added for notes above extension (dB/octave)
// Penalty slope = 10^(dB/20) - 1, applied per octave above boundary
#define CHORD_ABOVE_EXT_PENALTY_DB_PER_OCT 2.0f

// Rotate a 12-bit scale mask by semitones (positive = up from C)
uint16_t rotate_scale_mask(uint16_t base_mask, int8_t semitones);

// Get scale mask for a SWN scale enum value (rotated by key_semitones)
uint16_t get_scale_mask_for_swn_scale(uint8_t scale_num, int8_t key_semitones);

// Build a harmonic chord using dissonance minimization
// seed_freqs: array of frequencies from plugged jacks (the "given" notes)
// num_seeds: number of seed frequencies
// voices_to_fill: array of channel indices that need to be filled
// num_to_fill: number of voices to fill
// overtone_weights: array of 7 weights [fundamental, h2, h3, h4, h5, h6, h7]
// microtonal: if non-zero, don't quantize to semitones
// min_freq_hz: minimum allowed frequency for candidates
// max_freq_hz: maximum allowed frequency for candidates
// extension_freq_hz: upper boundary for extension penalty (highest seed freq)
// above_ext_db_per_oct: penalty in dB/octave for notes above extension
// scale_mask: 12-bit mask of allowed scale degrees (0xFFF = chromatic/all notes)
// scale_penalty: dissonance penalty added for out-of-scale candidates (0 = disabled)
// chord_out: output array of frequencies for each voice to fill (same order as voices_to_fill)
void build_harmonic_chord(
    float *seed_freqs,
    uint8_t num_seeds,
    uint8_t *voices_to_fill,
    uint8_t num_to_fill,
    float *overtone_weights,
    uint8_t microtonal,
    float min_freq_hz,
    float max_freq_hz,
    float extension_freq_hz,
    float above_ext_db_per_oct,
    uint16_t scale_mask,
    float scale_penalty,
    float *chord_out
);
