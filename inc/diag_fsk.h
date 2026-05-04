/*
 * diag_fsk.h - FSK transmitter for diagnostic firehose
 *
 * Audio-band FSK encoder, modeled on Mutable Instruments'
 * stm_audio_bootloader scheme.  Survives AC coupling, amplitude
 * attenuation, and bit-depth truncation in the audio path because
 * only zero-crossings and run-lengths are decoded.
 *
 * Symbol → on-wire shape:
 *     '0'   : hold current state for ZERO_PERIOD samples, then flip
 *     '1'   : hold current state for ONE_PERIOD  samples, then flip
 *     'P'   : hold current state for PAUSE_PERIOD samples, then flip
 * State alternates between +AMP and -AMP.  Every symbol is followed
 * by a flip, so adjacent symbols are easy to delimit at the decoder
 * (run length identifies the symbol).
 *
 * Periods chosen so the loudest run (period = 2 × ZERO_PERIOD = 8
 * samples) sits at 6 kHz @ 48 kHz Fs, well below Nyquist and well
 * within most audio paths.
 *
 * Packet format (built once per drain of the diag_log queue):
 *     [0xA5][0x5A]  - sync prefix (lots of transitions, easy to find)
 *     [len]         - payload length in bytes (multiple of 3)
 *     [payload]     - 'len' bytes, packed 3-byte events
 *     [crc16_hi]    - CRC-16-CCITT(0xFFFF) over [len][payload],
 *     [crc16_lo]      polynomial 0x1021, big-endian on the wire
 *
 * Each event = 24 bits = [evt:2][cycles:22] (see diag_log.h).
 *
 * The transmitter is driven once per audio frame from
 * process_audio_block_codec(): call diag_fsk_next_sample() to obtain
 * the int32_t to write to the right channel.  When the diag_log
 * queue is empty it emits pause symbols (so receivers can keep their
 * PLL locked even during quiet periods).
 */

#pragma once

#include <stdint.h>

/* Symbol periods (in audio samples @ 48 kHz). */
#define DIAG_FSK_ZERO_PERIOD  4u
#define DIAG_FSK_ONE_PERIOD   8u
#define DIAG_FSK_PAUSE_PERIOD 16u

/* Bipolar amplitude: ±0x600000 ≈ -3 dBFS at 24-bit signed.  Chosen
 * with headroom so any soft-saturation, AC-coupling overshoot, or
 * codec gain compression doesn't clip and create false zero-crossings
 * at the decoder. */
#define DIAG_FSK_AMP_POS  ( 0x600000)
#define DIAG_FSK_AMP_NEG  (-0x600000)

/* Packet framing constants. */
#define DIAG_FSK_SYNC0    0xA5u
#define DIAG_FSK_SYNC1    0x5Au

/* Maximum events per packet.  4 events = 12-byte payload, 17-byte
 * total packet, ~136 bits → ~816 audio samples (~17 ms) per packet
 * at average 6 samples/bit.  Yields ~230 events/s sustained. */
#define DIAG_FSK_MAX_EVENTS_PER_PACKET 4u

/* Packet buffer is 2 sync + 1 len + (3 * MAX_EVENTS) payload + 2 crc. */
#define DIAG_FSK_PACKET_MAX_BYTES \
	(3u + (3u * DIAG_FSK_MAX_EVENTS_PER_PACKET) + 2u)

/* Initialise the transmitter (idempotent).  Call once before any
 * call to diag_fsk_next_sample(). */
void diag_fsk_init(void);

/* Produce the next audio sample for the right channel.  Called from
 * the audio ISR.  Always returns a valid 24-bit signed int32_t,
 * either ±DIAG_FSK_AMP_POS/NEG (data) or alternating ±AMP at the
 * pause rate (idle).  Cheap enough to call once per audio frame. */
int32_t diag_fsk_next_sample(void);
