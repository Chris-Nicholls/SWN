/*
 * drum_voice_deluge_kick.c
 *
 * Port of render_kick() from drum_prototype/render_deluge_drums.py:
 * two synced sines ring-modulated together, with an exponential pitch
 * envelope dropping oscA, run through a 4-pole Moog-ladder lowpass and
 * a DC-blocking one-pole highpass.
 *
 * The python reference derives its numeric constants from hex-decoded
 * Deluge patch values; those are baked in here as literals (see the
 * DELUGE_* defines) rather than re-deriving them at runtime.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

/* ---- envelope ------------------------------------------------------------ */

/* Matches adsr() in the python reference: exponential attack to 1,
 * exponential decay to `sustain`, exponential release to 0. */
typedef struct {
	int   a_n, d_n, r_n;
	float a_inv, d_inv, r_inv;
	float sustain;
	int   idx;
} DelugeAdsr;

#define ADSR_CURVE     5.0f
#define ADSR_ATT_NORM  1.0157473f   /* 1/(1-exp(-5)), python's attack renormalisation */

static int adsr_seg_len(float seconds)
{
	int n = (int)(seconds * DRUM_VOICE_SAMPLE_RATE);
	return (n < 1) ? 1 : n;
}

static void adsr_set(DelugeAdsr *e, float attack, float decay, float sustain, float release)
{
	e->a_n = adsr_seg_len(attack);
	e->d_n = adsr_seg_len(decay);
	e->r_n = adsr_seg_len(release);
	e->a_inv = (e->a_n > 1) ? 1.0f / (float)(e->a_n - 1) : 0.0f;
	e->d_inv = (e->d_n > 1) ? 1.0f / (float)(e->d_n - 1) : 0.0f;
	e->r_inv = (e->r_n > 1) ? 1.0f / (float)(e->r_n - 1) : 0.0f;
	e->sustain = sustain;
	e->idx = 0;
}

static int adsr_total(const DelugeAdsr *e)
{
	return e->a_n + e->d_n + e->r_n;
}

static float adsr_ramp(int i, int seg, float seg_inv)
{
	return (seg > 1) ? ((float)i * seg_inv) : 1.0f;
}

static float adsr_next(DelugeAdsr *e)
{
	int i = e->idx;
	float v;

	if (i < e->a_n) {
		v = (1.0f - drum_fast_expf(-ADSR_CURVE * adsr_ramp(i, e->a_n, e->a_inv))) * ADSR_ATT_NORM;
	} else if (i < e->a_n + e->d_n) {
		float u = adsr_ramp(i - e->a_n, e->d_n, e->d_inv);
		v = e->sustain + (1.0f - e->sustain) * drum_fast_expf(-ADSR_CURVE * u);
	} else if (i < adsr_total(e)) {
		float u = adsr_ramp(i - e->a_n - e->d_n, e->r_n, e->r_inv);
		v = e->sustain * drum_fast_expf(-ADSR_CURVE * u);
	} else {
		return 0.0f;
	}

	e->idx++;
	return v;
}

/* ---- moog ladder --------------------------------------------------------- */

/* Padé-style tanh approximation: the ladder's saturation is part of the
 * voice's character, but a real tanhf() per sample per stage is far too
 * expensive for the audio ISR on the F765. */
static float fast_tanh(float x)
{
	if (x < -3.0f) return -1.0f;
	if (x >  3.0f) return  1.0f;
	float x2 = x * x;
	return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

typedef struct {
	float g;
	float k;
	float s[4];
} MoogLadder;

static void moog_set(MoogLadder *lp, float cutoff_hz, float resonance01)
{
	if (cutoff_hz < 20.0f) cutoff_hz = 20.0f;
	if (cutoff_hz > DRUM_VOICE_SAMPLE_RATE * 0.45f) cutoff_hz = DRUM_VOICE_SAMPLE_RATE * 0.45f;
	float g = tanf((float)M_PI * cutoff_hz / DRUM_VOICE_SAMPLE_RATE);
	lp->g = g / (1.0f + g);
	lp->k = resonance01 * 4.0f;
}

static float moog_process1(MoogLadder *lp, float x)
{
	float u = fast_tanh(x - lp->k * lp->s[3]);
	for (int stage = 0; stage < 4; stage++) {
		lp->s[stage] += lp->g * (u - lp->s[stage]);
		u = lp->s[stage];
	}
	return lp->s[3];
}

/* ---- one-pole highpass --------------------------------------------------- */

typedef struct {
	float a;
	float prev_x, prev_y;
} OnePoleHpf;

static void hpf_set(OnePoleHpf *hp, float cutoff_hz)
{
	hp->a = expf(-2.0f * (float)M_PI * cutoff_hz / DRUM_VOICE_SAMPLE_RATE);
}

static float hpf_process1(OnePoleHpf *hp, float x)
{
	float y = hp->a * (hp->prev_y + x - hp->prev_x);
	hp->prev_x = x;
	hp->prev_y = y;
	return y;
}

/* ---- voice --------------------------------------------------------------- */

#define DELUGE_KICK_OSC1_HZ     32.7025f    /* semitones_to_freq(-24) */
#define DELUGE_KICK_OSC2_HZ     73.414630f  /* semitones_to_freq(-10) */
#define DELUGE_KICK_SUSTAIN     0.06f
#define DELUGE_KICK_PITCH_DEC   0.0158489f  /* pitch envelope, from env_time(0.10, 0.01, 1.0) */
#define DELUGE_KICK_PITCH_REL   0.0154170f
#define DELUGE_KICK_LADDER_HZ   127.97f
#define DELUGE_KICK_LADDER_RES  0.19219f
#define DELUGE_KICK_HPF_HZ      15.0f
#define DELUGE_KICK_OUT_GAIN    3.2f        /* ladder at 128Hz plus resonance feedback loses most of the raw level */

/* The python renders a ~7ms blip; the knob is remapped onto a musically
 * useful body length instead. Total audible length is ~1.3x this. */
#define DELUGE_KICK_DECAY_MIN   0.15f
#define DELUGE_KICK_DECAY_MAX   1.9f

typedef struct {
	DrumSharedFilter filt;
	MoogLadder ladder;
	OnePoleHpf hpf;
	DelugeAdsr amp_env;
	DelugeAdsr pitch_env;

	float decay;        /* seconds, from set_decay */
	float drop_octaves; /* from set_other */

	float pitch_ratio;
	float osc1_phase;
	float osc2_phase;
	float osc1_base_hz;
	float osc2_hz;

	int n_samples;
	int sample_idx;
} DelugeKickState;

static void kick_trigger(void *state_v, float pitch)
{
	DelugeKickState *st = (DelugeKickState *)state_v;

	st->pitch_ratio  = powf(2.0f, pitch / 12.0f);
	st->osc1_base_hz = DELUGE_KICK_OSC1_HZ * st->pitch_ratio;
	st->osc2_hz      = DELUGE_KICK_OSC2_HZ * st->pitch_ratio;
	st->osc1_phase   = 0.0f;
	st->osc2_phase   = 0.0f;

	adsr_set(&st->pitch_env, 0.0f, DELUGE_KICK_PITCH_DEC, DELUGE_KICK_SUSTAIN, DELUGE_KICK_PITCH_REL);
	adsr_set(&st->amp_env, 0.0005f, st->decay, DELUGE_KICK_SUSTAIN, st->decay * 0.3f);

	moog_set(&st->ladder, DELUGE_KICK_LADDER_HZ, DELUGE_KICK_LADDER_RES);
	st->ladder.s[0] = st->ladder.s[1] = st->ladder.s[2] = st->ladder.s[3] = 0.0f;
	hpf_set(&st->hpf, DELUGE_KICK_HPF_HZ);
	st->hpf.prev_x = st->hpf.prev_y = 0.0f;

	st->n_samples  = adsr_total(&st->amp_env);
	st->sample_idx = 0;
}

static void kick_render(void *state_v, float *out, int n)
{
	DelugeKickState *st = (DelugeKickState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		float pe = adsr_next(&st->pitch_env);
		float f1 = st->osc1_base_hz * drum_fast_exp2f(st->drop_octaves * pe);

		st->osc1_phase += f1 * inv_sr;
		if (st->osc1_phase >= 1.0f) st->osc1_phase -= 1.0f;
		st->osc2_phase += st->osc2_hz * inv_sr;
		if (st->osc2_phase >= 1.0f) st->osc2_phase -= 1.0f;

		float ring = drum_fast_sin_turns(st->osc1_phase) *
			drum_fast_sin_turns(st->osc2_phase);

		float sig = ring * adsr_next(&st->amp_env);
		sig = moog_process1(&st->ladder, sig);
		out[i] = hpf_process1(&st->hpf, sig) * DELUGE_KICK_OUT_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void kick_set_filter(void *state_v, float cutoff01)
{
	DelugeKickState *st = (DelugeKickState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void kick_set_decay(void *state_v, float decay01)
{
	DelugeKickState *st = (DelugeKickState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = DELUGE_KICK_DECAY_MIN + decay01 * (DELUGE_KICK_DECAY_MAX - DELUGE_KICK_DECAY_MIN);
}

/* "other" -> pitch-envelope depth in octaves: with a ring-mod pair the
 * ladder cutoff barely moves the timbre (everything already sits under
 * 128Hz), whereas the oscA pitch drop is what turns the voice from a
 * flat sine thud into a full 808-style "boom" and simultaneously
 * resweeps the ring-mod sidebands. */
static void kick_set_other(void *state_v, float other01)
{
	DelugeKickState *st = (DelugeKickState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->drop_octaves = other01 * 2.5f;
}

static void kick_init(void *state_v)
{
	DelugeKickState *st = (DelugeKickState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->decay        = 0.5f;
	st->drop_octaves = 0.675f;   /* python's derived default (2.5 * 0.27) */
}

const DrumVoiceOps drum_voice_deluge_kick = {
	.init       = kick_init,
	.trigger    = kick_trigger,
	.render     = kick_render,
	.set_filter = kick_set_filter,
	.set_decay  = kick_set_decay,
	.set_other  = kick_set_other,
	.state_size = sizeof(DelugeKickState),
};
