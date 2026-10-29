# Velvet Reverb Port — SWN Implementation Plan

**Status:** design / not yet implemented
**Goal:** Replace the MI Clouds plate reverb on the SWN (STM32F765, Cortex-M7 @ 216 MHz)
with the three-stage sparse velvet-noise convolution reverb from the DLD firmware
(`../DLD/velvet_reverb.{c,h}`), and remove the Plaits synthesis engine to free the
SRAM1 needed for the velvet T2 delay line.

This document is the implementation contract. Source line references point at the
current DLD `velvet_reverb.c` / `velvet_reverb.h` and the SWN tree as of this writing.

---

## 1. Why this is a port, not a drop-in

The DLD velvet reverb was written for a different chip and a different audio
architecture. Four things differ and each drives a section of this plan:

| Concern | DLD (STM32F427, M4) | SWN (STM32F765, M7) |
|---|---|---|
| T2 storage | 512 KB **external SDRAM** @ `0xD2000000` | **no SDRAM** — must live in internal SRAM1 |
| Hot RAM section | `.ccmdata` (M4 Core-Coupled Memory) | no CCM; has DTCM (128 K) + SRAM1 (384 K) |
| T2 fetch | DMA2-Stream1 double-buffer prefetch (hides SDRAM latency) | direct cached reads; DMA unnecessary **and** a D-cache coherency hazard |
| Audio model | per-sample ISR push + main-loop `poll()` (16 spl @ 24 kHz) + per-sample ISR pull | one synchronous stereo block (48 spl @ 48 kHz) in the SAI DMA ISR |

CPU and memory-bandwidth are **not** blockers (the M7 is faster than the M4, T2
traffic is ~3–4 MB/s vs GB/s available, and once T2 is internal the fetch latency
the DMA was hiding effectively disappears — see §6). The blockers are memory
*placement* and the integration/threading model.

---

## 2. Target architecture (synchronous block model)

The DLD splits the reverb across three execution contexts. On the SWN we collapse
it into **one synchronous call inside the audio block**, matching how Clouds is
invoked today.

### 2.1 The key sizing identity

- SWN audio block = `MONO_BUFSZ` = 48 samples/channel @ 48 kHz (1 ms,
  `inc/drivers/codec_sai.h`).
- Velvet runs internally at 24 kHz (2:1 decimation of the codec rate) — keep this;
  it halves T2 size and MAC count, and a reverb is bandwidth-limited anyway.
- 48 codec samples → **24** reverb samples after 2:1 decimate.
- Velvet's 2× linear-interp upsampler turns 24 reverb samples → **48** codec samples.

So set **`REVERB_BLOCK = 24`** (currently 16). Then:
- exactly **one** velvet block is produced per SWN audio block,
- `REVERB_OUT_BLOCK = 48` = `MONO_BUFSZ` — the finalize stage emits exactly one SWN
  block of stereo output,
- no ring buffering, no `block_ready` flag, no `poll()`/`out_left()`/`out_right()`
  machinery, no output double-buffer needed.

`REVERB_BLOCK` is even (required by the dual-16 SIMD pair reads), so 24 is fine.

### 2.2 New public API (replaces push/poll/out)

Replace the three-context API with a single block processor mirroring
`Reverb_Process`:

```c
// velvet_reverb.h (SWN variant)
void Reverb_Init(void);                                    // wraps velvet_reverb_init()
void Reverb_SetParams(float time, float diffusion, float lp);   // -> velvet macros (§8)
void Reverb_Process(float *left, float *right, size_t size);    // size == MONO_BUFSZ (48)
```

`Reverb_Process` body (new `velvet_reverb_process_block`):

1. **Decimate + convert to mono int16** — fold the existing `push_sample`
   2:1 averaging into a loop: `in[k] = clip16( (L[2k]+R[2k] + L[2k+1]+R[2k+1]) * 0.5 * 32767 )`
   for k in 0..23. Velvet is mono-in (Clouds sums L+R too — `reverb.h:104`).
2. **Run the existing phase chain once** on this block (no `poll()` wrapper):
   `do_input_write_t0` → `do_t0_phase` → `do_t0_recirc` → `do_bridge_t0_to_t1`
   → `do_t1_phase` → `do_t1_recirc` → `do_bridge_t1_to_t2` → `do_t2_phase`
   → `do_t2_recirc` → `apply_pre_t2_sat` → `do_finalize`.
   These are unchanged except T2 storage/fetch (§6) and `diag_log` (§7).
3. **`do_finalize`** already produces 48 int16 L/R samples through the HPF/LPF
   biquads. Change it to write **float ±1.0** directly into the caller's `left[]`/
   `right[]` (×1/32768), replacing the int16 output double-buffer. The reverb is
   added to the dry bus by `oscillator.c` (it already does
   `arm_add_f32(output_buffer_*, reverb_send_*, ...)`), so write the wet result back
   into `reverb_send_L/R` in-place exactly like Clouds does.

Net effect: `velvet_reverb_push_sample`, `velvet_reverb_poll`,
`velvet_reverb_out_left/right`, the `input_buf_*`/`block_ready` plumbing, and the
`outL/outR` double-buffers are all **deleted** on the SWN.

### 2.3 Where the expensive recompute runs

`background_eff_gains_update()` (the per-block, round-robin effGains recompute,
`velvet_reverb.c:1394`) is the heaviest discretionary cost (~75 µs for one stage).
Keep it **out of the audio ISR**: call it from the SWN main super-loop
(`src/main.c` while(1)), throttled, the same way the DLD runs it outside the block
timer. The audio ISR then only pays for `update_morph_state()` (cheap) + the three
convolution stages. This preserves the DLD's cost structure and keeps the ISR lean.
(If main-loop jitter proves too coarse for smooth morphs, fall back to running one
stage's recompute per audio block inside the ISR — measure first, §11.)

---

## 3. Memory plan

Free SRAM1 by removing Plaits (§9), then place the velvet buffers:

| Buffer | Size | Placement | Section |
|---|---|---|---|
| `t0_ring` | 8192 × i16 = 16 KB | DTCM | default (DTCM is default RAM) |
| `t1_ring` | 16384 × i16 = 32 KB | DTCM | default |
| tap tables, ladders, effGains, accumulators, biquads | ~5 KB | DTCM | default |
| **`t2_ring`** | **see below** | **SRAM1** | `SRAM1DATA` (`inc/globals.h:79`) |

### 3.1 T2 sizing (the one real tradeoff)

T2 must be a power of two (`T2_RING_MASK = T2_RING_SAMPLES - 1` is a cheap `&`).
At 24 kHz:

| `T2_RING_SAMPLES` | bytes | tail length | note |
|---|---|---|---|
| 65536 | 128 KB | ~2.7 s | recommended starting point |
| 131072 | 256 KB | ~5.5 s | if SRAM1 budget allows after Plaits removal |

SRAM1 is 384 KB. Removing Plaits frees 96 KB (`voice_buffers`) + the 32 KB Clouds
tank = 128 KB, on top of whatever is already free. Start at **65536 (128 KB)** for
comfortable headroom; bump to 131072 once the build links and SRAM1 usage is
confirmed. Note the DLD default `T2_DURATION_MAX_SAMPLES` (192000, 8 s) must be
reduced to ≤ `T2_RING_SAMPLES`, and the macro bound `MP_T2_DURATION` hi value
(`velvet_reverb.c:250`, currently 3.0 s) capped to the new max.

### 3.2 Section attribute remap (`CCM_ATTR`)

DLD uses `__attribute__((aligned(4), section(".ccmdata")))`. On the SWN there is no
`.ccmdata`; DTCM is the default RAM. So for the SWN build:

```c
#define CCM_ATTR __attribute__((aligned(4)))      // lands in DTCM .bss by default
```

`t2_ring` instead gets `SRAM1DATA alignas(32)` (32-byte align = one M7 cache line per
tap read, §6). Like the Clouds tank, **zero it in `Reverb_Init`** — `.sram1data` is
NOLOAD, so the CRT does not clear it and stale RAM would dump a ghost-reverb burst
on power-up (`src/reverb.cc:23-44` documents this exact hazard).

---

## 4. Source plan

Bring the DLD files into the SWN tree, guarded so the DLD original is untouched:

- Copy `velvet_reverb.c` → `src/velvet_reverb.c`, `velvet_reverb.h` → `inc/velvet_reverb.h`.
- Introduce a build define `VELVET_T2_INTERNAL` (set for the SWN build) gating:
  - the `t2_ring` placement (SRAM1 vs SDRAM pointer),
  - the DMA-free `do_t2_phase` (§6),
  - `CCM_ATTR` definition,
  - removal of the DMA2 init block (`velvet_reverb.c:1040-1047`).
- Replace `src/reverb.cc` with a thin C wrapper (or rename) exposing
  `Reverb_Init/SetParams/Process` backed by velvet instead of `plaits::Reverb`.
  Keep `inc/reverb.h` unchanged so `oscillator.c` needs no include changes.
- Keep `src/plaits/dsp/fx/{reverb.h,fx_engine.h}` only if still referenced; once
  `reverb.cc` no longer wraps Clouds they can be dropped.

Makefile: add `src/velvet_reverb.c`, add `-DVELVET_T2_INTERNAL` (and a 24 kHz /
`REVERB_BLOCK=24` define if not hard-coded), remove the Clouds wrapper if renamed.

---

## 5. Execution-context summary

After the refactor, everything lives in two contexts (vs the DLD's three):

| Work | Context | Cost | Notes |
|---|---|---|---|
| decimate → conv stages (T0/T1/T2) → recirc → finalize | SAI DMA ISR (1 ms block) | hot path — must fit budget (§11) | one velvet block per audio block |
| `update_morph_state()` (IIR + recirc offsets) | SAI DMA ISR | cheap (~5 µs) | runs every block |
| `background_eff_gains_update()` (effGains recompute) | main super-loop, throttled | ~75 µs/stage, amortized | kept out of the ISR (§2.3) |
| `Reverb_SetParams` → macro setters (deadbanded) | audio ISR (per block) | trivial | actual recompute deferred to main loop |

---

## 6. T2 in SRAM1 — remove DMA, read the ring directly

On the DLD, `do_t2_phase` (`velvet_reverb.c:1654`) prefetches each tap from SDRAM via
DMA2-Stream1 double-buffering. On the SWN this is **counterproductive**: D-cache is
enabled (`src/main.c:104 SCB_EnableDCache()`), so DMA into/out of cached SRAM is a
coherency bug, and the latency the DMA hid is gone for internal RAM. Also SWN's SAI
already owns DMA2-Stream2.

Replace with direct cached reads (one M7 cache line = 32 B = 16 i16 covers an
aligned 16-sample run; a `__builtin_prefetch`/PLD on the next tap is the
cache-friendly analog of the old double-buffer):

```c
static inline void do_t2_pass(const uint16_t *effOff, int32_t *acc)
{
    int count = t2TapCount;
    for (int t = 0; t < count; t++) {
        uint32_t gain = (uint32_t)(int32_t)effGains_t2[t];
        uint32_t base = (block_write_idx - (uint32_t)effOff[t]) & T2_RING_MASK;
        if (t + 1 < count) {
            uint32_t nb = (block_write_idx - (uint32_t)effOff[t+1]) & T2_RING_MASK;
            __builtin_prefetch(&t2_ring[nb]);          // PLD on M7; NOP elsewhere
        }
        uint32_t until_wrap = T2_RING_SAMPLES - base;
        uint32_t n1 = (until_wrap < (uint32_t)REVERB_BLOCK) ? until_wrap : (uint32_t)REVERB_BLOCK;
        const int16_t *src = &t2_ring[base];
        int i = 0;
        for (uint32_t k = 0; k < n1; k += 2, i += 2) {
            uint32_t s = *((const u32_alias *)(src + k));
            acc[i]   = qadd_sat(acc[i],   smulbb(s, gain));
            acc[i+1] = qadd_sat(acc[i+1], smultb(s, gain));
        }
        if (n1 < (uint32_t)REVERB_BLOCK) {
            src = &t2_ring[0];
            uint32_t n2 = (uint32_t)REVERB_BLOCK - n1;
            for (uint32_t k = 0; k < n2; k += 2, i += 2) {
                uint32_t s = *((const u32_alias *)(src + k));
                acc[i]   = qadd_sat(acc[i],   smulbb(s, gain));
                acc[i+1] = qadd_sat(acc[i+1], smultb(s, gain));
            }
        }
    }
}
static void do_t2_phase(void) {
    if (t2TapCount == 0) return;
    do_t2_pass(effOffT2L, accL);
    do_t2_pass(effOffT2R, accR);
}
```

This reuses the even-offset invariant already maintained by the tap generator
(`& ~1`) and the wrap-split pattern from `do_t1_phase`. Delete `dma_scratch`,
`dma_buf_idx`, `dma2_kick/wait/fetch_tap`, and the DMA2 init.

**Access pattern is unchanged from the original design:** scattered reads per tap
into the contiguous accumulators; the only ring *writes* are the contiguous,
one-per-sample writes of the freshly-advanced block (bridge / recirc feedback /
pre-T2 sat). No scattered writes — so the write-back cache only dirties ~1–2 lines
per block and all scattered traffic is read-side line fills.

---

## 7. F4 → F7 portability fixes

1. **DSP intrinsics** — `smulbb`, `smultb`, `qadd` (inline asm, `velvet_reverb.c:78-97`)
   are ARMv7E-M DSP instructions; the Cortex-M7 supports them unchanged. No action.
2. **`diag_log`** — DLD-specific (`#include "diag_log.h"`, calls throughout `poll()`).
   On the SWN, `#ifdef` these out and instead bracket `Reverb_Process` with the
   existing DWT pattern (`diag_reverb_peak_cycles`, `src/oscillator.c:425-430`,
   `src/timekeeper.c:82`). Add per-stage peaks if finer visibility is wanted.
3. **`stm32f4xx.h`** include → `stm32f7xx.h` for the SWN build (only used for `DWT`,
   DMA regs which we're removing, and `__DMB`). Gate with `VELVET_T2_INTERNAL` /
   existing `VELVET_REVERB_HOST`.
4. **D-cache** — no maintenance needed once DMA is removed (all CPU access). T2 in
   normal cacheable SRAM1 is correct as-is.

---

## 8. Parameter mapping (SWN params → velvet macros)

Keep the existing `params.reverb_*` fields and the `reverb_ui.c` slider UI
unchanged; only reinterpret three params inside the new `Reverb_SetParams`:

| SWN param (range) | Velvet control | Rationale |
|---|---|---|
| `reverb_time` [0,1] | `velvet_reverb_apply_decay_macro(time)` | Decay macro drives T2 duration + decay shape + recirc (`velvet_reverb.c:267-275`) |
| `reverb_diffusion` [0,1] | `velvet_reverb_apply_density_macro(diffusion)` | Density macro drives tap counts + recirc — the closest analog to allpass smear |
| `reverb_lp` [0,1] | `velvet_reverb_apply_tone_macro(lp)` | Tone macro drives output LPF/HPF (`velvet_reverb.c:276-279`) |
| `reverb_input_gain` | `reverb_send` global / pre-T2 sat drive | velvet has `SAT_PRE_T2_PREGAIN`; expose if drive control is wanted |
| `reverb_output_level` | applied by `oscillator.c` `arm_scale_f32` (unchanged) | already scales wet before mix-back |
| `reverb_send[chan]` | folded into the mono send bus (unchanged) | already summed pre-reverb |

The macro deadband + throttle (`set_macro_value`, `velvet_reverb.c:345`) already
guards against per-block ADC jitter. `Reverb_SetParams` just calls the three
`apply_*_macro` setters; the actual recompute happens in the throttled main-loop
`background_eff_gains_update` (§2.3). Note the LPF/HPF macro bounds are currently
**pinned** for a click investigation (`velvet_reverb.c:253-257`) — unpin
`MP_LPF`/`MP_HPF` so `reverb_lp`/Tone actually moves the filters, and verify no
periodic-click regression on the SWN.

---

## 9. Plaits removal

Plaits engines are already excluded from the build (`Makefile INCLUDE_PLAITS_ENGINES ?= 0`);
the live footprint to reclaim is the SRAM1 voice buffers. Steps:

1. **Reclaim SRAM1:** remove `voice_buffers[6][16384]` (96 KB `SRAM1_DATA`) and the
   `plaits::Voice voices[6]` array from `src/plaits_shim.cpp`. Confirm the active
   voice engine is the Halo physics engine (the audio path calls `halo_fill_block`,
   `oscillator.c:273`, not `Plaits_Render`), so removing the Plaits voices does not
   touch live synthesis.
2. **Keep the LPG:** `Shim_LPG_Init/Trigger/Process/GetEnvelope` and
   `plaits::LowPassGate`/`LPGEnvelope` stay — used by `envout_pwm.c` and
   `oscillator.c:905`. Trim `plaits_shim.{h,cpp}` down to the LPG stubs.
3. **Build:** the `INCLUDE_PLAITS_ENGINES` block can stay at 0 (or be deleted along
   with the engine source globs, `Makefile:43-45`). Keep stmlib (LPG depends on it).
4. **Graceful-degradation refs** (params/sphere/wavetable code that skips bank
   [100,123]) can be left as-is — loading a Plaits sphere already falls back to a
   factory sphere. Optional cleanup later.
5. **Preset compatibility:** `plaits_params[NUM_CHANNELS]` in the preset struct
   (`inc/params_update.h:248`) — leave the field reserved (do **not** shrink the
   struct) to avoid a preset-format/version bump, unless a version bump is already
   planned.

This is independent of the reverb work and can land first as its own change.

---

## 10. Integration steps (ordered)

1. **Plaits removal** (§9) — frees SRAM1, lands independently, verify firmware still
   boots and synthesizes.
2. **Vendor velvet files** into `src/`/`inc/` with `VELVET_T2_INTERNAL` guards (§4).
3. **Memory + section remap** (§3): `CCM_ATTR`, `t2_ring` → `SRAM1DATA`, sizes,
   `T2_RING_SAMPLES`/duration caps. Confirm it links and SRAM1/DTCM fit (`.map`).
4. **DMA removal + direct T2 reads** (§6).
5. **Block-model refactor** (§2): `REVERB_BLOCK = 24`, new
   `velvet_reverb_process_block`, float boundary in `do_finalize`, delete push/poll/out.
   Retune morph alphas for the 1000 Hz block rate (24 kHz / 24): multiply the three
   `ALPHA_*_MORPH` constants (`velvet_reverb.c:360-362`) by 1.5.
6. **Wrapper swap**: point `Reverb_Init/SetParams/Process` (`inc/reverb.h`) at velvet;
   `oscillator.c` is unchanged.
7. **Param mapping** (§8) + unpin LPF/HPF macros.
8. **Recompute scheduling** (§2.3): call throttled `background_eff_gains_update` from
   `src/main.c` super-loop.
9. **Diag** (§7): wire `diag_reverb_peak_cycles` around `Reverb_Process`; measure.

---

## 11. CPU budget & risks

- **Budget:** audio block = 1 ms = 216,000 cycles @ 216 MHz. Clouds costs ~80 µs.
  Velvet is materially heavier (three sparse-conv stages + recirc + biquads). The
  audio ISR also runs the Halo voices + mixing; check `diag_audio_isr_peak_cycles`
  headroom **before** porting. The convolution scales with active tap counts (T0≤40,
  T1≤40, T2≤32 ×2 passes) — start with conservative density (lower macro defaults)
  and measure.
- **Primary risk — ISR overrun.** Mitigations, in order: (a) keep
  `background_eff_gains_update` in the main loop (§2.3); (b) reduce `MAX_T*_TAPS`;
  (c) reduce T2 tap count via the Density bound; (d) last resort, move the whole
  reverb out of the ISR into a buffered main-loop task (reintroduces the DLD-style
  split — avoid unless necessary).
- **Secondary risk — periodic click** from the pinned LPF/HPF macros once unpinned
  (§8); the DLD code has hysteresis (`do_finalize:1814`) but verify on SWN.
- **Non-risks (confirmed earlier):** memory bandwidth (~3–4 MB/s ≪ GB/s), T2 fetch
  latency (cached SRAM line fills ≈ 3–7 µs/block, ~0 with PLD), DSP intrinsics.

---

## 12. Validation

- **Host harness:** the DLD code already builds for host under `VELVET_REVERB_HOST`
  (`host_t2_ring_storage`, `host_shim.h`). Use it to verify the block-model refactor
  (REVERB_BLOCK=24, float boundary, direct T2 reads) produces sane output offline
  before flashing — feed an impulse/sweep, dump the wet `.wav`, compare tail shape.
- **On-target:** flash, confirm no power-on ghost burst (T2 zeroed), sweep each
  reverb slider, watch `diag_reverb_peak_cycles` / `diag_audio_isr_peak_cycles` on
  the CPU LED display, confirm no audio dropouts (ISR < ~700 µs leaving margin).
- **A/B:** keep a Clouds build switch during bring-up for direct comparison.

---

## 13. Open decisions

- T2 size: 128 KB (2.7 s) vs 256 KB (5.5 s) — pick after measuring freed SRAM1.
- Whether to expose `reverb_input_gain` → velvet pre-T2 saturation drive, or leave
  the velvet sat hardcoded and repurpose that slider.
- Whether to keep 24 kHz internal (memory-optimal) or run velvet at 48 kHz
  (simpler, no resampler, but 2× T2 RAM and ~2× MACs) — **recommend 24 kHz** given
  the SRAM1 constraint.
