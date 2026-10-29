/*
 * drum_shared_filter.c
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>

#include "drum_shared_filter.h"
#include "drum_voice.h"

#define DRUM_FILTER_MIN_HZ 80.0f
#define DRUM_FILTER_MAX_HZ 8000.0f

void drum_shared_filter_init(DrumSharedFilter *filt)
{
	filt->low   = 0.0f;
	filt->band  = 0.0f;
	filt->q_inv = 1.0f / 0.8f;   /* mild resonance; enough character without self-oscillating near cutoff01=1 */
	drum_shared_filter_set_cutoff(filt, 1.0f);
}

void drum_shared_filter_set_cutoff(DrumSharedFilter *filt, float cutoff01)
{
	if (cutoff01 < 0.0f) cutoff01 = 0.0f;
	if (cutoff01 > 1.0f) cutoff01 = 1.0f;

	float hz = DRUM_FILTER_MIN_HZ * powf(DRUM_FILTER_MAX_HZ / DRUM_FILTER_MIN_HZ, cutoff01);
	float f  = 2.0f * sinf((float)M_PI * hz / DRUM_VOICE_SAMPLE_RATE);
	if (f > 1.9f) f = 1.9f;   /* Chamberlin SVF stability limit as cutoff approaches Nyquist */
	filt->f = f;
}

void drum_shared_filter_process(DrumSharedFilter *filt, float *buf, int n)
{
	float low = filt->low, band = filt->band;
	float f = filt->f, q_inv = filt->q_inv;

	for (int i = 0; i < n; i++) {
		float in   = buf[i];
		float high = in - low - q_inv * band;
		band += f * high;
		low  += f * band;
		buf[i] = low;
	}

	filt->low  = low;
	filt->band = band;
}
