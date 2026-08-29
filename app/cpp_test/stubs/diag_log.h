/*
 * diag_log.h — host stub.  No-op replacement for the firmware's FSK
 * diagnostic logger.
 */
#pragma once

#include <stdint.h>

typedef enum {
    DIAG_EVT_ADVANCE  = 0,
    DIAG_EVT_SEEDLERP = 1,
    DIAG_EVT_OSCTIM   = 2,
    DIAG_EVT_AUDIOISR = 3,
    DIAG_EVT_COUNT    = 4,
} diag_evt_t;

static inline void diag_log(diag_evt_t evt, uint32_t cycles) {
    (void)evt;
    (void)cycles;
}
