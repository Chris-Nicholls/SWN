/*
 * oscillator.h — host stub.
 *
 * halo.c references `extern o_wt_osc wt_osc` so it can latch a
 * triggerPending flag on `wt_osc.halo_state[chan]` in
 * halo_request_trigger().  All we need is a minimal definition
 * with a halo_state array.  The C++ test driver does NOT route through
 * this path (it owns its own o_halo instance and triggers
 * synchronously), but the symbol must exist for the firmware C file to
 * link.
 */
#pragma once

#include "globals.h"
#include "sphere.h"
#include "halo.h"

typedef struct o_wt_osc {
    o_halo halo_state[NUM_CHANNELS];
} o_wt_osc;
