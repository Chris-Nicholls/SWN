/*
 * drum_voice_deluge_closed_hat.c
 *
 * Port of render_closed_hat() from drum_prototype/render_deluge_drums.py:
 * a pair of detuned squares buried under white noise, a single very
 * short envelope on the sum, and a one-pole highpass that leaves only
 * the metallic top end. No ladder filter in this recipe.
 *
 * -----------------------------------------------------------------------------
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "drum_fast_math.h"
#include "drum_voice.h"
#include "drum_shared_filter.h"

/* ---- envelope ------------------------------------------------------------ */

typedef struct {
	int   a_n, d_n, r_n;
	float a_inv, d_inv, r_inv;
	float sustain;
	int   idx;
} DelugeAdsr;

#define ADSR_CURVE     5.0f
#define ADSR_ATT_NORM  1.0157473f

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

/* ---- one-pole highpass --------------------------------------------------- */

typedef struct {
	float a;
	float prev_x, prev_y;
} OnePoleHpf;

static void hpf_set(OnePoleHpf *hp, float cutoff_hz)
{
	if (cutoff_hz < 5.0f) cutoff_hz = 5.0f;
	if (cutoff_hz > DRUM_VOICE_SAMPLE_RATE * 0.45f) cutoff_hz = DRUM_VOICE_SAMPLE_RATE * 0.45f;
	hp->a = expf(-2.0f * (float)M_PI * cutoff_hz / DRUM_VOICE_SAMPLE_RATE);
}

static float hpf_process1(OnePoleHpf *hp, float x)
{
	float y = hp->a * (hp->prev_y + x - hp->prev_x);
	hp->prev_x = x;
	hp->prev_y = y;
	return y;
}

/* ---- noise --------------------------------------------------------------- */

typedef struct { uint32_t s; } Xorshift32;

static float xorshift_uniform(Xorshift32 *rng)
{
	uint32_t x = rng->s;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	rng->s = x;
	return ((float)(x & 0xFFFFFF) / (float)0x800000) - 1.0f;
}

/* ---- voice --------------------------------------------------------------- */

#define DELUGE_CHAT_SQ1_HZ      130.81f     /* semitones_to_freq(0) */
#define DELUGE_CHAT_SQ2_HZ      195.99355f  /* semitones_to_freq(7) */
#define DELUGE_CHAT_HPF_MIN     1000.0f
#define DELUGE_CHAT_HPF_SPAN    11000.0f    /* other01 = 0.62 reproduces the python's 7844Hz */
#define DELUGE_CHAT_OUT_GAIN    0.9f

#define DELUGE_CHAT_DECAY_MIN   0.015f
#define DELUGE_CHAT_DECAY_MAX   0.35f

typedef struct {
	DrumSharedFilter filt;
	OnePoleHpf hpf;
	DelugeAdsr env;
	Xorshift32 rng;

	float decay;
	float hpf_hz;

	float sq1_phase, sq2_phase;
	float sq1_hz, sq2_hz;

	int n_samples;
	int sample_idx;
} DelugeChatState;

static void chat_trigger(void *state_v, float pitch)
{
	DelugeChatState *st = (DelugeChatState *)state_v;

	float r = powf(2.0f, pitch / 12.0f);
	st->sq1_hz = DELUGE_CHAT_SQ1_HZ * r;
	st->sq2_hz = DELUGE_CHAT_SQ2_HZ * r;
	st->sq1_phase = 0.0f;
	st->sq2_phase = 0.0f;

	/* python uses a slightly shorter release than decay (0.008 vs 0.02 knob) */
	adsr_set(&st->env, 0.0003f, st->decay, 0.0f, st->decay * 0.5f);

	hpf_set(&st->hpf, st->hpf_hz);
	st->hpf.prev_x = st->hpf.prev_y = 0.0f;

	st->n_samples  = adsr_total(&st->env);
	st->sample_idx = 0;
}

static void chat_render(void *state_v, float *out, int n)
{
	DelugeChatState *st = (DelugeChatState *)state_v;
	const float inv_sr = 1.0f / DRUM_VOICE_SAMPLE_RATE;

	for (int i = 0; i < n; i++) {
		if (st->sample_idx >= st->n_samples) {
			out[i] = 0.0f;
			continue;
		}

		st->sq1_phase += st->sq1_hz * inv_sr;
		if (st->sq1_phase >= 1.0f) st->sq1_phase -= 1.0f;
		st->sq2_phase += st->sq2_hz * inv_sr;
		if (st->sq2_phase >= 1.0f) st->sq2_phase -= 1.0f;

		float tone = 0.5f * (((st->sq1_phase < 0.5f) ? 1.0f : -1.0f) +
		                     ((st->sq2_phase < 0.5f) ? 1.0f : -1.0f));
		float nz = xorshift_uniform(&st->rng);

		float sig = (0.25f * tone + 0.9f * nz) * adsr_next(&st->env);
		out[i] = hpf_process1(&st->hpf, sig) * DELUGE_CHAT_OUT_GAIN;
		st->sample_idx++;
	}

	drum_shared_filter_process(&st->filt, out, n);
}

static void chat_set_filter(void *state_v, float cutoff01)
{
	DelugeChatState *st = (DelugeChatState *)state_v;
	drum_shared_filter_set_cutoff(&st->filt, cutoff01);
}

static void chat_set_decay(void *state_v, float decay01)
{
	DelugeChatState *st = (DelugeChatState *)state_v;
	if (decay01 < 0.0f) decay01 = 0.0f;
	if (decay01 > 1.0f) decay01 = 1.0f;
	st->decay = DELUGE_CHAT_DECAY_MIN + decay01 * (DELUGE_CHAT_DECAY_MAX - DELUGE_CHAT_DECAY_MIN);
}

/* "other" -> highpass corner: this voice is a broadband noise burst, so
 * where the highpass sits *is* the sound -- low corner gives a "tsk"
 * with body, high corner the thin 909 sizzle. It is also the only tone
 * control the python recipe has for this voice. */
static void chat_set_other(void *state_v, float other01)
{
	DelugeChatState *st = (DelugeChatState *)state_v;
	if (other01 < 0.0f) other01 = 0.0f;
	if (other01 > 1.0f) other01 = 1.0f;
	st->hpf_hz = DELUGE_CHAT_HPF_MIN + other01 * DELUGE_CHAT_HPF_SPAN;
	hpf_set(&st->hpf, st->hpf_hz);
}

static void chat_init(void *state_v)
{
	DelugeChatState *st = (DelugeChatState *)state_v;
	memset(st, 0, sizeof(*st));
	drum_shared_filter_init(&st->filt);
	st->rng.s  = 0xB5297A4Du;
	st->decay  = 0.05f;
	st->hpf_hz = DELUGE_CHAT_HPF_MIN + 0.62f * DELUGE_CHAT_HPF_SPAN;
	hpf_set(&st->hpf, st->hpf_hz);
}

const DrumVoiceOps drum_voice_deluge_closed_hat = {
	.init       = chat_init,
	.trigger    = chat_trigger,
	.render     = chat_render,
	.set_filter = chat_set_filter,
	.set_decay  = chat_set_decay,
	.set_other  = chat_set_other,
	.state_size = sizeof(DelugeChatState),
};
