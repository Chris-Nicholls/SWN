/*
 * drum_roller_metal.c
 *
 * -----------------------------------------------------------------------------
 */

#include <string.h>

#include "drum_roller_metal.h"

static const float kMetalPartials[DRUM_ROLLER_METAL_PARTIALS] = {
	263.0f, 400.0f, 421.0f, 474.0f, 587.0f, 845.0f
};

#define METAL_BP_Q 0.9f
#define METAL_HP_Q 0.7f

void drum_roller_metal_init(DrumRollerMetal *m, uint32_t seed)
{
	memset(m, 0, sizeof(*m));
	roller_noise_seed(&m->rng, seed);
	drum_roller_metal_config(m, 9000.0f, 6400.0f, 2.15f, 0.6f);
	drum_roller_metal_set_env(m, 0.7f, 0.0345f);
}

void drum_roller_metal_config(DrumRollerMetal *m, float bp_freq, float hp_freq,
                              float ratio, float noise_mix)
{
	for (int i = 0; i < DRUM_ROLLER_METAL_PARTIALS; i++)
		m->inc[i] = kMetalPartials[i] * ratio / DRUM_VOICE_SAMPLE_RATE;

	roller_biquad_set(&m->bp, ROLLER_BQ_BANDPASS, bp_freq, METAL_BP_Q);
	roller_biquad_set(&m->hp, ROLLER_BQ_HIGHPASS, hp_freq, METAL_HP_Q);
	roller_biquad_set(&m->noise_hp, ROLLER_BQ_HIGHPASS, hp_freq * 1.05f, METAL_HP_Q);

	m->noise_mix = noise_mix;
}

void drum_roller_metal_set_env(DrumRollerMetal *m, float v, float dec)
{
	m->dec = dec;
	roller_env_set(&m->env, 0.0015f, v, dec, 0.0006f);
}

void drum_roller_metal_reset(DrumRollerMetal *m)
{
	for (int i = 0; i < DRUM_ROLLER_METAL_PARTIALS; i++)
		m->phase[i] = 0.0f;

	roller_biquad_reset(&m->bp);
	roller_biquad_reset(&m->hp);
	roller_biquad_reset(&m->noise_hp);
}

float drum_roller_metal_tick(DrumRollerMetal *m, float t)
{
	float squares = 0.0f;
	for (int i = 0; i < DRUM_ROLLER_METAL_PARTIALS; i++) {
		float p = m->phase[i] + m->inc[i];
		if (p >= 1.0f) p -= 1.0f;
		m->phase[i] = p;
		squares += (p < 0.5f) ? 1.0f : -1.0f;
	}

	float sig = roller_biquad_tick(&m->hp, roller_biquad_tick(&m->bp, squares));

	if (m->noise_mix > 0.0f) {
		float nz = roller_biquad_tick(&m->noise_hp, roller_noise_next(&m->rng));
		if (t <= m->dec + 0.02f)
			sig += m->noise_mix * nz;
	}

	if (t > m->dec + 0.04f)
		return 0.0f;

	return sig * roller_env_at(&m->env, t);
}
