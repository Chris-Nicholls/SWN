/*
 * globals.h — host stub.  Just the few defines that halo.h /
 * halo.c actually reference.
 */
#pragma once

#define NUM_CHANNELS  6
#define SAMPLERATE    48000
#define F_SAMPLERATE  48000.0f

/* Memory-section attributes are firmware-specific; on the host they
 * map to nothing. */
#define DMABUFFER
#define SRAM1DATA
