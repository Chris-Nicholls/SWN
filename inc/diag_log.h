/*
 * diag_log.h - firehose event log + FSK packet source
 *
 * TEMPORARY diagnostic infrastructure for pinpointing CPU overruns in
 * the oscillator ISRs.  Events are enqueued from any ISR into a
 * lock-free ring buffer, then drained by the FSK encoder and streamed
 * out the right audio channel as a Manchester/FSK bit stream (see
 * diag_fsk.h).  The FSK signal only cares about zero-crossings and
 * run-lengths, so it survives AC coupling, amplitude attenuation, and
 * bit-depth reduction — i.e. it is robust through a real audio output
 * path recorded by a DAW.  Same approach as Mutable Instruments'
 * stm_audio_bootloader FSK flasher.
 *
 * On-wire event format (3 bytes, big-endian):
 *     byte 0 : [evt_type:2][cycles_hi:6]
 *     byte 1 : [cycles_mid:8]
 *     byte 2 : [cycles_lo:8]
 *   -> 22-bit cycles, saturates at 0x3FFFFF (~19.4 ms at 216 MHz).
 *
 * Producers call diag_log(evt, cycles).  Under normal operation we're
 * bandwidth-limited (FSK carries ~250 events/s), so the log filters
 * by threshold: only events whose duration exceeds a per-type
 * threshold are enqueued (SEEDLERP is always logged, as it marks
 * triggers).  Tune the thresholds at runtime via diag_thresh_cycles[].
 *
 * Capture: record the right audio jack at 48 kHz (16 or 24-bit, AC
 * or DC coupled — doesn't matter) and decode with
 * app/diag_decode.py.
 */

#pragma once

#include <stdint.h>
#include "stm32f7xx.h"

typedef enum {
	DIAG_EVT_ADVANCE  = 0u, /* halo_advance_cycle duration */
	DIAG_EVT_SEEDLERP = 1u, /* halo_seed_lerp duration     */
	DIAG_EVT_OSCTIM   = 2u, /* update_oscillators total duration  */
	DIAG_EVT_AUDIOISR = 3u, /* process_audio_block_codec duration */
	DIAG_EVT_COUNT    = 4u,
} diag_evt_t;

/* 22-bit cycle field → saturation at 0x3FFFFF = 4_194_303 cycles
 * ≈ 19.42 ms at 216 MHz.  Plenty for the events we track; anything
 * longer than that is "pathological — went to lunch" and lumps into
 * the max bucket. */
#define DIAG_CYCLES_BITS   22u
#define DIAG_CYCLES_MASK   ((1u << DIAG_CYCLES_BITS) - 1u)
#define DIAG_EVT_SHIFT     DIAG_CYCLES_BITS
#define DIAG_EVT_MASK      0x3u

/* Ring buffer of packed 24-bit events (stored in the low 24 bits of
 * uint32_t slots).  2048 × 4 B = 8 kB.  At the expected 200-500
 * events/s steady-state, this buffers >4 s of bursts — the FSK TX
 * will drain it well before overflow. */
#define DIAG_LOG_SIZE 2048u
#define DIAG_LOG_MASK (DIAG_LOG_SIZE - 1u)

extern volatile uint32_t diag_log_buf[DIAG_LOG_SIZE];
extern volatile uint32_t diag_log_head;    /* producers */
extern volatile uint32_t diag_log_tail;    /* consumer (FSK TX) */
extern volatile uint32_t diag_log_dropped; /* ring-buffer overflow */

/* Per-type "log only if cycles >= threshold" floor.  Set to 0 to log
 * every event of that type (only affordable for SEEDLERP).  Default
 * thresholds keep the average rate within FSK bandwidth while still
 * catching every outlier we care about. */
extern volatile uint32_t diag_thresh_cycles[DIAG_EVT_COUNT];

/* Per-type totals (including filtered-out events) — useful to verify
 * the ISRs are running at the expected rate even when thresholds
 * hide most events. */
extern volatile uint32_t diag_evt_total[DIAG_EVT_COUNT];

/* Master enable.  When 0 the audio ISR leaves the right channel
 * alone (carries the normal synth output). */
extern volatile uint8_t diag_log_enabled;

/* Producer.  Safe from any ISR priority.  Briefly disables IRQs to
 * claim a slot in the ring buffer. */
static inline void diag_log(diag_evt_t evt, uint32_t cycles)
{
	/* Count every event, whether or not we transmit it. */
	diag_evt_total[evt]++;

	if (!diag_log_enabled) return;
	if (cycles < diag_thresh_cycles[evt]) return;

	if (cycles > DIAG_CYCLES_MASK) cycles = DIAG_CYCLES_MASK;
	uint32_t packed = ((uint32_t)evt << DIAG_EVT_SHIFT) | cycles; /* 24 bits */

	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	uint32_t h = diag_log_head;
	uint32_t next = (h + 1u) & DIAG_LOG_MASK;
	if (next != diag_log_tail) {
		diag_log_buf[h] = packed;
		diag_log_head = next;
	} else {
		diag_log_dropped++;
	}
	if (!primask) __enable_irq();
}

/* Consumer helper for the FSK packetizer.  Returns 1 and writes *out
 * if an event was dequeued, else 0.  Single caller (FSK TX state
 * machine, running at audio-ISR priority), so the ring buffer is
 * SPSC from its perspective. */
static inline int diag_log_pop_packed(uint32_t *out)
{
	uint32_t t = diag_log_tail;
	if (t == diag_log_head) return 0;
	*out = diag_log_buf[t] & 0xFFFFFFu;
	diag_log_tail = (t + 1u) & DIAG_LOG_MASK;
	return 1;
}
