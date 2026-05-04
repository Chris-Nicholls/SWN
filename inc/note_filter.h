/*
 * note_filter.h — Adaptive lag processor for stable pitch tracking
 *
 * Ported from Mutable Instruments Rings (rings/dsp/note_filter.h)
 * by Emilie Gillet.  Original: MIT license.
 *
 * Operates in semitone space.  Combines a median-of-4 pre-filter with an
 * adaptive one-pole lag whose coefficient snaps to "fast" on a sharp edge
 * and then smoothly decays toward "slow" over a configurable recovery time.
 *
 * Usage:
 *   NoteFilter nf;
 *   note_filter_init(&nf, 1000.0f);          // call rate in Hz
 *   ...
 *   float filtered = note_filter_process(&nf, raw_note, strum);
 *   float stable   = note_filter_stable_note(&nf);
 */

#ifndef NOTE_FILTER_H_
#define NOTE_FILTER_H_

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- tunables (match Rings' defaults) ---- */
#define NF_ORDER              4      /* median filter order                    */
#define NF_DELAY_SIZE        16      /* power-of-2 delay line for stable_note  */
#define NF_EDGE_THRESHOLD  0.4f      /* semitones – triggers fast-track reset  */

typedef struct {
	float prev[NF_ORDER];

	float note;
	float stable_note;

	/* tiny ring-buffer delay line for stable_note */
	float delay_buf[NF_DELAY_SIZE];
	uint8_t delay_wr;
	uint8_t delay_len;

	/* adaptive coefficients */
	float coeff;
	float stable_coeff;
	float fast_coeff;
	float slow_coeff;
	float lag_coeff;
} NoteFilter;

/* ---- helpers (internal) ---- */

/* Sorting network for exactly 4 elements — branchless-friendly. */
static inline void nf_sort4_(float a[4])
{
#define NF_SWAP_(i, j) \
	do {               \
		if (a[i] > a[j]) { float t_ = a[i]; a[i] = a[j]; a[j] = t_; } \
	} while (0)
	NF_SWAP_(0, 1); NF_SWAP_(2, 3);
	NF_SWAP_(0, 2); NF_SWAP_(1, 3);
	NF_SWAP_(1, 2);
#undef NF_SWAP_
}

/* ---- public API ---- */

/*
 * Initialise a NoteFilter.
 *
 * @param sample_rate  How often note_filter_process() will be called (Hz).
 *                     Rings uses kSampleRate/kMaxBlockSize ≈ 750 Hz.
 *                     For SWN main-loop, ~1000 Hz is a reasonable estimate.
 */
static inline void note_filter_init(NoteFilter *nf, float sample_rate)
{
	/* Rings' Init() parameters:
	 *   fast edge lag:         0.001 s
	 *   steady-state lag:      0.010 s  (originally 0.005 in string_synth_part)
	 *   edge recovery time:    0.050 s
	 *   edge avoidance delay:  0.004 s
	 */
	nf->fast_coeff = 1.0f / (0.001f * sample_rate);
	nf->slow_coeff = 1.0f / (0.010f * sample_rate);
	nf->lag_coeff  = 1.0f / (0.050f * sample_rate);

	nf->note         = 69.0f;   /* A4 */
	nf->stable_note  = 69.0f;
	nf->coeff        = nf->fast_coeff;
	nf->stable_coeff = nf->slow_coeff;

	for (int i = 0; i < NF_ORDER; i++)
		nf->prev[i] = nf->note;

	/* Delay line: ~4 ms worth of samples (matches Rings' 0.004 s). */
	nf->delay_len = (uint8_t)((0.004f * sample_rate) + 0.5f);
	if (nf->delay_len < 1)              nf->delay_len = 1;
	if (nf->delay_len > NF_DELAY_SIZE)  nf->delay_len = NF_DELAY_SIZE;
	nf->delay_wr = 0;
	for (int i = 0; i < NF_DELAY_SIZE; i++)
		nf->delay_buf[i] = nf->stable_note;
}

/*
 * Feed one sample and return the fast-tracking filtered note.
 *
 * @param note   Raw note value in semitones (69 = A4 / 440 Hz).
 * @param strum  Non-zero on a trigger / gate event — forces instant snap.
 */
static inline float note_filter_process(NoteFilter *nf, float note, uint8_t strum)
{
	if (fabsf(note - nf->note) > NF_EDGE_THRESHOLD || strum) {
		/* Sharp edge or explicit trigger — snap to raw value immediately. */
		nf->note         = note;
		nf->stable_note  = note;
		nf->coeff        = nf->fast_coeff;
		nf->stable_coeff = nf->slow_coeff;
		for (int i = 0; i < NF_ORDER; i++)
			nf->prev[i] = note;
		/* Fill delay line so stable_note reads back this value. */
		for (int i = 0; i < NF_DELAY_SIZE; i++)
			nf->delay_buf[i] = note;
	} else {
		/* Shift history ring (rotate left by 1, append new sample). */
		for (int i = 0; i < NF_ORDER - 1; i++)
			nf->prev[i] = nf->prev[i + 1];
		nf->prev[NF_ORDER - 1] = note;

		/* Median of 4 (average of middle two after sort). */
		float sorted[NF_ORDER];
		for (int i = 0; i < NF_ORDER; i++)
			sorted[i] = nf->prev[i];
		nf_sort4_(sorted);
		float median = 0.5f * (sorted[(NF_ORDER - 1) / 2] + sorted[NF_ORDER / 2]);

		/* Adaptive one-pole lag — fast after edge, decays to slow. */
		nf->note += nf->coeff * (median - nf->note);
		nf->stable_note += nf->stable_coeff * (nf->note - nf->stable_note);

		/* Both coeffs decay toward slow_coeff (steady-state one-pole
		 * tracking).  The fast-tracking `note` starts at fast_coeff and
		 * `stable_note` starts at slow_coeff, so after an edge snap the
		 * fast filter catches up quickly while the stable filter
		 * remains the reliable pitch reference. */
		nf->coeff        += nf->lag_coeff * (nf->slow_coeff - nf->coeff);
		nf->stable_coeff += nf->lag_coeff * (nf->slow_coeff - nf->stable_coeff);

		/* Write stable_note into delay line. */
		nf->delay_buf[nf->delay_wr] = nf->stable_note;
		nf->delay_wr = (nf->delay_wr + 1) & (NF_DELAY_SIZE - 1);
	}
	return nf->note;
}

/* Fast-tracking note (tracks real changes in ~1 ms). */
static inline float note_filter_note(const NoteFilter *nf)
{
	return nf->note;
}

/* Heavily-filtered note, delayed by ~4 ms.
 * Useful for capturing the pitch *before* a transient arrives. */
static inline float note_filter_stable_note(const NoteFilter *nf)
{
	uint8_t rd = (nf->delay_wr - nf->delay_len) & (NF_DELAY_SIZE - 1);
	return nf->delay_buf[rd];
}

/* Convert frequency (Hz) to semitones above C-1 / relative to A4.
 * Returns MIDI-like semitone value: 69 = A4 = 440 Hz. */
static inline float nf_freq_to_note(float freq)
{
	if (freq <= 0.0f) return 0.0f;
	return 69.0f + 12.0f * log2f(freq / 440.0f);
}

/* Convert semitones back to frequency (Hz). */
static inline float nf_note_to_freq(float note)
{
	return 440.0f * powf(2.0f, (note - 69.0f) / 12.0f);
}

#ifdef __cplusplus
}
#endif

#endif /* NOTE_FILTER_H_ */
