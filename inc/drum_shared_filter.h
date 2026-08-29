/*
 * drum_shared_filter.h
 *
 * One resonant state-variable filter (Chamberlin topology) shared by
 * every drum voice's `set_filter`/post-render stage, so the Depth
 * control always does something meaningful regardless of which
 * underlying voice algorithm is selected (see drum_voice.h).
 *
 * -----------------------------------------------------------------------------
 */

#pragma once

typedef struct DrumSharedFilter {
	float low;
	float band;
	float f;       /* frequency coefficient, recomputed by set_cutoff */
	float q_inv;   /* 1/Q, fixed resonance amount */
} DrumSharedFilter;

void drum_shared_filter_init(DrumSharedFilter *filt);

/* cutoff01 in [0,1] maps exponentially onto ~80Hz..8kHz. */
void drum_shared_filter_set_cutoff(DrumSharedFilter *filt, float cutoff01);

/* Lowpass output, processed in place. */
void drum_shared_filter_process(DrumSharedFilter *filt, float *buf, int n);
