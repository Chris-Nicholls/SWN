/*
 * diag_fsk.c - FSK transmitter for the diagnostic firehose.
 *
 * Runs entirely from the audio ISR (priority 0,0).  No locks, no
 * dynamic allocation, all state in this TU.  See diag_fsk.h for the
 * wire format.
 *
 * State machine (driven once per audio frame):
 *   - sample_hold_remaining > 0: continue holding current polarity.
 *   - else                       : flip polarity, advance to next bit.
 *
 * The "next bit" comes from a bit cursor over a byte buffer.  When
 * the buffer is exhausted we try to build a fresh packet from the
 * diag_log ring buffer.  If the queue is empty we emit pause
 * symbols indefinitely.
 *
 * One audio frame = one call to diag_fsk_next_sample(), so the
 * decoder sees identical timing as long as the audio block is
 * dispatched on schedule (which we are using this very firehose
 * to verify!).  An OSC_TIM stall doesn't perturb the FSK clock
 * because the audio ISR runs at strictly higher priority.
 */

#include "diag_fsk.h"
#include "diag_log.h"

/* ── Internal symbol enum ───────────────────────────────────────── */
typedef enum {
	FSK_SYM_ZERO  = 0,
	FSK_SYM_ONE   = 1,
	FSK_SYM_PAUSE = 2,
} fsk_sym_t;

/* ── Bit-stream buffer ──────────────────────────────────────────── */
static uint8_t  s_pkt_buf[DIAG_FSK_PACKET_MAX_BYTES];
static uint16_t s_pkt_len_bytes = 0;
static uint16_t s_pkt_bit_idx   = 0; /* total bits emitted from s_pkt_buf */

/* ── Carrier state ──────────────────────────────────────────────── */
static int32_t  s_state             = DIAG_FSK_AMP_POS;
static uint16_t s_sample_hold_count = 1; /* primed with one pause sample */
static uint8_t  s_initialised       = 0;

/* ── CRC-16-CCITT (poly 0x1021, init 0xFFFF) ────────────────────── */
static uint16_t crc16_ccitt(uint16_t crc, const uint8_t *data, uint32_t n)
{
	for (uint32_t i = 0; i < n; ++i) {
		crc ^= ((uint16_t)data[i]) << 8;
		for (int b = 0; b < 8; ++b) {
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
			                       : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

/* ── Pull events from diag_log into a fresh on-wire packet.  ────
 * Returns the packet length in bytes (0 if queue was empty). */
static uint16_t build_packet(void)
{
	/* Drain up to MAX events from diag_log into the payload region
	 * (offset 3 = sync0, sync1, len). */
	uint8_t *payload = &s_pkt_buf[3];
	uint16_t n_events = 0;
	while (n_events < DIAG_FSK_MAX_EVENTS_PER_PACKET) {
		uint32_t packed;
		if (!diag_log_pop_packed(&packed)) break;
		/* Big-endian: byte0 has high bits (evt + cycles top). */
		payload[3 * n_events + 0] = (uint8_t)((packed >> 16) & 0xFFu);
		payload[3 * n_events + 1] = (uint8_t)((packed >>  8) & 0xFFu);
		payload[3 * n_events + 2] = (uint8_t)( packed        & 0xFFu);
		n_events++;
	}
	if (n_events == 0) return 0;

	uint16_t payload_bytes = (uint16_t)(3u * n_events);

	s_pkt_buf[0] = DIAG_FSK_SYNC0;
	s_pkt_buf[1] = DIAG_FSK_SYNC1;
	s_pkt_buf[2] = (uint8_t)payload_bytes;

	/* CRC is computed over [len][payload]. */
	uint16_t crc = crc16_ccitt(0xFFFFu, &s_pkt_buf[2], (uint32_t)(1u + payload_bytes));
	s_pkt_buf[3 + payload_bytes + 0] = (uint8_t)((crc >> 8) & 0xFFu);
	s_pkt_buf[3 + payload_bytes + 1] = (uint8_t)( crc       & 0xFFu);

	return (uint16_t)(3u + payload_bytes + 2u);
}

/* ── Symbol selection ──────────────────────────────────────────── */
static fsk_sym_t next_symbol(void)
{
	/* Prefer to send packet bits.  When the current packet is done,
	 * try to build the next.  If nothing to send, emit a pause. */
	if (s_pkt_bit_idx >= (uint16_t)(s_pkt_len_bytes * 8u)) {
		s_pkt_len_bytes = build_packet();
		s_pkt_bit_idx   = 0;
		if (s_pkt_len_bytes == 0) return FSK_SYM_PAUSE;
	}
	uint16_t byte_idx = s_pkt_bit_idx >> 3;
	uint8_t  bit_pos  = (uint8_t)(7u - (s_pkt_bit_idx & 7u)); /* MSB-first */
	uint8_t  bit      = (uint8_t)((s_pkt_buf[byte_idx] >> bit_pos) & 1u);
	s_pkt_bit_idx++;
	return bit ? FSK_SYM_ONE : FSK_SYM_ZERO;
}

static uint16_t period_for(fsk_sym_t sym)
{
	switch (sym) {
		case FSK_SYM_ZERO:  return DIAG_FSK_ZERO_PERIOD;
		case FSK_SYM_ONE:   return DIAG_FSK_ONE_PERIOD;
		default:            return DIAG_FSK_PAUSE_PERIOD;
	}
}

/* ── Public API ─────────────────────────────────────────────────── */
void diag_fsk_init(void)
{
	s_pkt_len_bytes     = 0;
	s_pkt_bit_idx       = 0;
	s_state             = DIAG_FSK_AMP_POS;
	s_sample_hold_count = 1;
	s_initialised       = 1;
}

int32_t diag_fsk_next_sample(void)
{
	if (!s_initialised) diag_fsk_init();

	if (s_sample_hold_count == 0) {
		/* Boundary: flip polarity, fetch next symbol. */
		s_state = -s_state;
		fsk_sym_t sym = next_symbol();
		s_sample_hold_count = period_for(sym);
	}
	s_sample_hold_count--;
	return s_state;
}
