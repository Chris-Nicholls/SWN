# Organic Synth Idea — Functional Requirements

## 1. Overview

A single-cycle waveform stored in a circular buffer of N samples (N = 1024 or 2048) evolves continuously over time. The evolution produces a slowly shifting harmonic spectrum — individual harmonics appear, grow, and recede one at a time, creating a smooth, non-periodic timbral animation.

Because the buffer represents exactly one period of the waveform, its DFT bins correspond to integer multiples of the fundamental. Non-integer (inharmonic) frequencies cannot exist in the buffer by construction.

## 2. Initialisation

The buffer is initialised from an arbitrary wavetable (sine, saw, square, or user-drawn). The algorithm reads this initial shape and begins evolving it from the first call to `advanceCycle()`.

## 3. Spectral evolution

### 3.1 Isolated harmonics
Starting from a simple waveform (e.g. a pure sine at mode 1), new harmonics should appear **individually** — not all at once. At any given moment during the evolution only a small number of harmonics should be significantly active. The order need not follow the harmonic series; jumping from mode 1 → 6 → 12 → 4 → 2 is perfectly valid. The key constraint is that only one or two modes are growing at any time.

The evolution should resemble:

> mode 1 → mode 1 + growing mode 6 → mode 6 dominant → mode 6 + growing mode 3 → …

rather than:

> mode 1 → all harmonics up to some rolloff (saw-like)

When the initial buffer is harmonically rich (e.g. a saw or square wave), the output should retain the **fingerprint** of that waveform for a time — the same set of partials, with their relative amplitudes gradually shifting. Over many cycles the content should eventually settle into the same isolated-harmonic wandering behaviour, as higher partials decay and the waveform simplifies.

### 3.2 Smooth morphing
The transition between harmonic states should be continuous and gradual. No sudden jumps or clicks. The rate of evolution is controllable

### 3.3 Non-periodic
The spectral trajectory should not cycle back to the starting point on any fixed period. It may wander, drift, or cascade indefinitely (within the bounds of the buffer's Nyquist limit at N/2).

### 3.4 Self-driven
The evolution should be **self-organising** — driven by the current spectral content of the buffer, not by an external LFO or clock. The waveform's own shape determines what happens next.

## 4. Amplitude preservation

### 4.1 No collapse
The output amplitude (RMS of the buffer) must not decay toward zero over time, even at high drive settings. Some gentle decay is acceptable if a separate sustain parameter controls it, but the evolution mechanism itself must not drain energy.

### 4.2 No blow-up
The output amplitude must not grow without bound. The system must be stable for all parameter settings across all input waveforms.

### 4.3 Conservation target
The perceptually relevant quantity to preserve is approximately **Σ kα |Aₖ|²** for some α ∈ [1, 2], where Aₖ is the amplitude of harmonic k. This weights higher harmonics more — it is acceptable for total amplitude (Σ |Aₖ|²) to decrease slightly as energy moves to higher harmonics, as long as perceived loudness / brightness remains roughly constant.

## 5. Spectral purity

### 5.1 No new broadband content
The algorithm must not introduce broadband spectral content. Because the buffer is exactly one period long, DFT bins already correspond to integer harmonics — inharmonic content cannot arise by construction. The concern is therefore about *broadband filling*: the algorithm should not spread energy uniformly across many harmonics.

- **Octaves of existing modes are allowed.** If mode k exists at initialisation, modes 2k, 4k, k/2 (if integer) etc. may be created freely. Other integer multiples can also be created, but are less preferred.
- Energy should remain concentrated in a small number of harmonics at any given time, not smeared across the full spectrum.

### 5.1a Waveform character preservation
The evolution should not immediately erase the harmonic character of the initial waveform. For example, a square wave (odd harmonics only) should retain its odd-harmonic dominance for a significant number of cycles before even harmonics grow in. A saw wave's 1/k rolloff shape should remain recognisable early on. The waveform's identity may eventually dissolve, but not abruptly.

### 5.2 Broadband inputs
If the initial buffer is broadband (e.g. saw wave with all harmonics), the algorithm may keep all that content. It should gradually **emphasise different parts** of the spectrum over time — spotlighting individual harmonics or groups — rather than smearing them together. Over many cycles the higher partials should naturally decay, ultimately leaving a sine wave (the fundamental), at which point the isolated-harmonic cascade behaviour (§3.1) takes over.

### 5.3 Clean partials
Each active harmonic should be a clean sinusoidal partial, not a broad spectral bump. The DFT of the buffer at any point should show sharp peaks at integer modes.

## 6. Performance

- Must run in real-time on an ARM Cortex-M7 at audio rate (one `advanceCycle()` call per sample period at up to ~48 kHz equivalent throughput).
- Memory budget: ≤ 6 × N floats of working storage (including the output buffer).
- No FFT, no per-sample transcendentals beyond what can be replaced by table lookup or recurrence.

## 7. Waveform compatibility

The algorithm must produce musically useful results from:
- Pure sine (the primary test case for cascade behaviour)
- Sawtooth
- Square wave
- Arbitrary user wavetables (smooth or discontinuous)

For already-harmonically-rich inputs (saw, square), the evolution may redistribute existing energy rather than creating new harmonics from silence. The key requirement is that the redistribution is **selective** (individual modes shift), not **broadband** (everything smears).

## 8. Non-requirements

- The algorithm does **not** need to model any specific physical system.
- The spectral trajectory does **not** need to be deterministic or reproducible given the same input — slight parameter-dependent drift is acceptable.
- Perfect energy conservation is **not** required — bounded drift (< ~50% over hundreds of cycles) is acceptable as long as collapse and blow-up are prevented.

## 9. Implementation preferences (soft requirements)

- **Time-domain preferred.** The core algorithm should operate on the circular buffer directly, not via FFT/IFFT or frequency-domain manipulation.
- **No explicit modulation sources.** Avoid hard-coded LFOs, ADSRs, or fixed modulation frequencies. Any modulation, drift, or spectral movement should arise as a **natural emergent property** of the system's dynamics — not from an injected oscillator or envelope.
