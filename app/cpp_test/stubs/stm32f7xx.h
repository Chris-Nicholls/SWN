/*
 * stm32f7xx.h — host-side stub.
 *
 * The firmware uses DWT->CYCCNT for cheap cycle counts.  On the host we
 * just hand back a monotonically increasing counter so the diagnostic
 * code in halo.c compiles unchanged.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    volatile uint32_t CYCCNT;
} DWT_Type;

extern DWT_Type _host_dwt;
#define DWT (&_host_dwt)

/* IRQ enable/disable are no-ops on the host. */
static inline void __disable_irq(void)             {}
static inline void __enable_irq(void)              {}
static inline uint32_t __get_PRIMASK(void)         { return 0; }
static inline void __set_PRIMASK(uint32_t pri)     { (void)pri; }

#ifdef __cplusplus
}
#endif
