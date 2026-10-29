# Organic Synth — Functional Test Requirements

> **Status:** Test requirements only — actual test implementations are deferred until the algorithm's parameter space is finalised.

These tests validate the requirements in [synth-requirements.md](synth-requirements.md). Each test section references the corresponding requirement. Where a test says "run for M cycles" this means M consecutive calls to `advanceCycle()`.

---

## T1 — Initialisation

### T1.1 Buffer identity after init
Initialise the buffer with each reference waveform (sine, saw, square). Before any call to `advanceCycle()`, the buffer contents must exactly match the input waveform (bit-exact or within float rounding tolerance). *(Req §2)*

### T1.2 Arbitrary wavetable round-trip
Initialise with a user-drawn wavetable containing both smooth and discontinuous regions. Verify the buffer matches the input before evolution begins. *(Req §2, §7)*

### T1.3 Evolution begins on first call
Call `advanceCycle()` once after init with a pure sine. The buffer must differ from the initial state (i.e. evolution has begun). *(Req §2)*

---

## T2 — Spectral Evolution: Isolated Harmonics

### T2.1 Sine → isolated harmonic emergence
Initialise with a pure sine (mode 1). Run for a moderate number of cycles. At sampled checkpoints, compute the DFT and count harmonics whose amplitude exceeds a significance threshold (e.g. > −40 dB relative to the strongest mode). At no checkpoint should more than a small number of harmonics (e.g. ≤ 3) be simultaneously significant. *(Req §3.1)*

### T2.2 Peak count over time — histogram
Collect the "number of significant harmonics" at every checkpoint over a long run. The distribution should be concentrated at low counts (1–3), not broadly spread. *(Req §3.1)*

### T2.3 Rich waveform fingerprint retention
Initialise with a saw wave. Snapshot the harmonic amplitudes at init. After a short run, the correlation between the initial and current amplitude profiles should remain high (the saw "fingerprint" is preserved). After a much longer run, the profile should have diverged significantly as the waveform simplifies. *(Req §3.1)*

### T2.4 Rich → sparse convergence
Initialise with a saw wave. Run for a very long duration. Eventually the number of significant harmonics should drop to a small count, entering the isolated-cascade regime. *(Req §3.1, §5.2)*

---

## T3 — Spectral Evolution: Smooth Morphing

### T3.1 No discontinuities in harmonic amplitudes
Run the algorithm from a sine init. At each cycle, record the amplitude of every harmonic. The first derivative (Δ per cycle) of each harmonic's amplitude must remain below a smoothness bound — no single-cycle jumps exceeding a defined threshold. *(Req §3.2)*

### T3.2 No audible clicks — zero-crossing rate
Compute the sample-to-sample difference of the output buffer at each cycle. The maximum absolute difference must stay below a click threshold for all cycles. *(Req §3.2)*

### T3.3 Rate control authority
Run the algorithm at the minimum and maximum evolution-rate settings. The rate of change of the spectral centroid (or similar summary statistic) must be measurably different between the two settings — confirming the rate parameter has effect. *(Req §3.2)*

---

## T4 — Spectral Evolution: Non-periodicity

### T4.1 No spectral recurrence
Initialise with a sine. Run for a long duration, recording the harmonic-amplitude vector at each cycle. Compute the autocorrelation of this trajectory. No peak (other than lag 0) should exceed a periodicity threshold. *(Req §3.3)*

### T4.2 Trajectory divergence over time
Record the spectral state at cycle C₀. Continue for a long run. The Euclidean distance between the spectral state and C₀ should not return to near-zero more than once. *(Req §3.3)*

---

## T6 — Amplitude Preservation: No Collapse

### T6.1 RMS floor after long run
Initialise with each reference waveform. Run for a very large number of cycles. The RMS of the buffer must never fall below a minimum fraction (e.g. 20%) of the initial RMS. *(Req §4.1)*

### T6.2 Sustained energy across parameter extremes
Repeat T6.1 at all corners of the parameter space (min/max of every parameter). None must collapse. *(Req §4.1)*

---

## T7 — Amplitude Preservation: No Blow-up

### T7.1 RMS ceiling after long run
Initialise with each reference waveform. Run for a very large number of cycles. The RMS of the buffer must never exceed a maximum multiple (e.g. 3×) of the initial RMS. *(Req §4.2)*

### T7.2 Stability across parameter extremes
Repeat T7.1 at all corners of the parameter space. None must blow up. *(Req §4.2)*

### T7.3 Pathological input — DC + impulse
Initialise the buffer with a DC offset plus a single-sample impulse. Run for many cycles. The output must remain bounded. *(Req §4.2)*

---

## T8 — Amplitude Preservation: Perceptual Conservation

### T8.1 Brightness-weighted energy tracking
At each checkpoint, compute $E_\alpha = \sum_k k^\alpha |A_k|^2$ for $\alpha$ in the range [1, 2]. Over a long run, $E_\alpha$ must stay within ±50% of its initial value (matching the bounded-drift tolerance in §8). *(Req §4.3)*

### T8.2 Spectral centroid stability
Compute the spectral centroid at each checkpoint. It should vary (confirming evolution) but its long-term average should remain in a bounded range — not monotonically increasing or decreasing. *(Req §4.3)*

---

## T9 — Spectral Purity: No Broadband Filling

### T9.1 Sine does not become broadband
Initialise with a pure sine (mode 1). Run for many cycles. At sampled checkpoints, compute the DFT. The number of harmonics above a significance threshold should remain small (consistent with T2.1). The spectrum must not converge toward a saw-like 1/k envelope or a flat noise-like distribution. *(Req §5.1)*

### T9.2 Square wave character preservation
Initialise with a square wave (odd harmonics: 1, 3, 5, 7 …). Run for a moderate number of cycles. At checkpoints, compute the ratio of energy in odd harmonics to energy in even harmonics. This ratio must remain well above 1.0 for a significant initial period, confirming the square-wave character is preserved and the waveform does not immediately become saw-like. *(Req §5.1a)*

### T9.3 Saw wave character preservation
Initialise with a saw wave. Run for a moderate number of cycles. At checkpoints, compute the correlation between the current harmonic amplitude profile and the ideal 1/k saw envelope. The correlation must remain high for a significant initial period — the saw character should not immediately dissolve into a different spectral shape. *(Req §5.1a)*

### T9.4 Energy concentration over time
For each reference waveform, run for many cycles. At each checkpoint, measure the fraction of total spectral energy contained in the top 3 harmonics. This fraction should generally be high (energy concentrated, not spread), consistent with isolated-harmonic behaviour rather than broadband filling. *(Req §5.1)*

---

## T10 — Spectral Purity: Broadband Input Behaviour

### T10.1 Saw wave — selective emphasis
Initialise with a saw wave. At periodic checkpoints, identify the dominant harmonic (highest amplitude). The identity of the dominant harmonic should change over time, confirming selective emphasis rather than uniform smearing. *(Req §5.2)*

### T10.2 Saw wave — eventual simplification
Initialise with a saw wave. Run for a very long duration. The number of significant harmonics should monotonically trend downward (with noise), eventually reaching a small count. *(Req §5.2)*

---

## T12 — Performance

### T12.1 Cycle budget — wall-clock timing
Measure the execution time of a single `advanceCycle()` call on the target ARM Cortex-M7, averaged over many calls. It must complete within one sample period at the target sample rate (e.g. ≤ 20.8 µs at 48 kHz). *(Req §6)*

### T12.2 Memory footprint
Inspect the total working memory allocated by the algorithm (excluding code). It must not exceed 6 × N floats. *(Req §6)*

### T12.3 No FFT in hot path
Static analysis / code review: confirm that `advanceCycle()` does not call any FFT routine or per-sample transcendental function (sin, cos, exp, log) that isn't backed by a lookup table or recurrence. *(Req §6)*

---

## T13 — Waveform Compatibility

### T13.1 Musically useful output — sine
Initialise with a pure sine. After evolution, the output buffer must still be a valid single-cycle waveform (no DC drift, no wraparound discontinuity beyond the initial state). *(Req §7)*

### T13.2 Musically useful output — saw
Same as T13.1 but initialised with a saw wave. *(Req §7)*

### T13.3 Musically useful output — square
Same as T13.1 but initialised with a square wave. *(Req §7)*

### T13.4 Musically useful output — arbitrary wavetable
Same as T13.1 but initialised with a user-drawn wavetable containing sharp transitions. *(Req §7)*

### T13.5 Selective redistribution for rich inputs
For saw and square inits, measure the Gini coefficient (or similar inequality metric) of the harmonic amplitude distribution at checkpoints. It should increase over time (energy concentrating into fewer modes), confirming selective rather than broadband redistribution. *(Req §7)*

---

## T14 — Boundary & Edge Cases

### T14.1 Minimum buffer size
Run the algorithm with the smallest supported N. All stability and purity tests (T6–T9) must still pass. *(Req §4, §5)*

### T14.2 Maximum buffer size
Run the algorithm with the largest supported N. Performance test (T12) must still pass. *(Req §6)*

### T14.3 Silent buffer input
Initialise with an all-zero buffer. The algorithm must not blow up (output remains zero or near-zero) and must not produce NaN/Inf. *(Req §4.2)*

### T14.4 DC-only buffer input
Initialise with a constant (non-zero DC) buffer. The algorithm must remain bounded and must not introduce spectral content at non-DC frequencies. *(Req §4.2, §5.1)*

### T14.5 Single-sample impulse
Initialise with a single non-zero sample surrounded by zeros. The algorithm must remain bounded and must not produce NaN/Inf. *(Req §4.2)*

---

## Notes

- **Thresholds and cycle counts** are left as placeholders (e.g. "many cycles", "significance threshold") until the algorithm's parameter space and typical time constants are known.
- **Buffer size** is fixed at N = 1024 or 2048. Because the buffer is exactly one period, DFT bins correspond to integer harmonics by construction — inharmonic content cannot arise.
- **DFT-based checks** (T2, T9) are for offline validation only — the algorithm itself must not use FFT at runtime (§6).
- Tests are designed to be automatable: each produces a scalar pass/fail metric once thresholds are defined.
