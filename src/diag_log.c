/*
 * diag_log.c - backing storage + threshold defaults for the
 *              diagnostic firehose.  Enqueue/dequeue logic is
 *              inline in diag_log.h.
 *
 * Default thresholds chosen so steady-state output stays within FSK
 * bandwidth (~250 events/s) while still capturing every outlier.
 * Override at runtime by writing to diag_thresh_cycles[] from main()
 * or the debugger.
 *
 *   ADVANCE  > 50 µs  (10800 cycles @ 216 MHz)  — physics tick
 *   SEEDLERP > 0      (every trigger, rare and important)
 *   OSCTIM   > 300 µs (64800 cycles)            — OSC_TIM total
 *   AUDIOISR > 200 µs (43200 cycles)            — audio ISR total
 */

#include "diag_log.h"

volatile uint32_t diag_log_buf[DIAG_LOG_SIZE];
volatile uint32_t diag_log_head    = 0;
volatile uint32_t diag_log_tail    = 0;
volatile uint32_t diag_log_dropped = 0;
volatile uint32_t diag_evt_total[DIAG_EVT_COUNT] = { 0, 0, 0, 0 };

volatile uint32_t diag_thresh_cycles[DIAG_EVT_COUNT] = {
	[DIAG_EVT_ADVANCE]  = 10800u, /*  50 µs */
	[DIAG_EVT_SEEDLERP] = 0u,     /* always */
	[DIAG_EVT_OSCTIM]   = 64800u, /* 300 µs */
	[DIAG_EVT_AUDIOISR] = 43200u, /* 200 µs */
};

volatile uint8_t diag_log_enabled = 1;
