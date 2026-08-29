/**
 * rcx_test_vectors.js
 * -------------------
 * Runs RingCouplerX in Node.js for a set of test cases and writes the buffer
 * after each cycle to stdout as a JSON array.
 *
 * Usage:
 *   node app/rcx_test_vectors.js > app/rcx_test_vectors.json
 */

'use strict';

const fs = require('fs');
const path = require('path');

// ── Load synths.js by eval-wrapping it (no module system in that file) ──────
const synthsSource = fs.readFileSync(path.join(__dirname, 'synths.js'), 'utf8');
// Wrap so that class declarations become accessible in the returned object.
const synthsExports = (new Function(
    synthsSource + '\n return { RingCouplerX };'
))();
const { RingCouplerX } = synthsExports;

// ── Deterministic test signal generators ────────────────────────────────────

function sineWave(N) {
    const a = new Float32Array(N);
    for (let i = 0; i < N; i++) a[i] = Math.sin(2 * Math.PI * i / N);
    return a;
}

function sawWave(N) {
    const a = new Float32Array(N);
    for (let i = 0; i < N; i++) a[i] = 2 * i / (N - 1) - 1;
    return a;
}

function squareWave(N) {
    const a = new Float32Array(N);
    for (let i = 0; i < N; i++) a[i] = i < N / 2 ? 1 : -1;
    return a;
}

// Sweep 5 harmonics with specified amplitudes
function multiSine(N) {
    const a = new Float32Array(N);
    const amps = [1, 0.5, 0.33, 0.25, 0.2];
    for (let i = 0; i < N; i++)
        for (let k = 0; k < amps.length; k++)
            a[i] += amps[k] * Math.sin(2 * Math.PI * (k + 1) * i / N);
    // Normalise
    const peak = a.reduce((m, v) => Math.max(m, Math.abs(v)), 0) || 1;
    return a.map(v => v / peak);
}

// ── Test case factory ─────────────────────────────────────────────────────────

const R = 4;

/**
 * Build test cases.
 * Each case: { name, N, n_cycles, params, signal }
 * For N/phiFracs: use values that give exact integer phis so Python fractional
 * interpolation is identical to JS integer indexing.
 */
function buildTestCases() {
    // N=128: fracs [0.25, 0.5, 0.75, 0.125] → phis [32, 64, 96, 16] ✓
    const phiExact128 = [0.25, 0.5, 0.75, 0.125];
    // N=256: same fracs, phis [64, 128, 192, 32] ✓
    const phiExact256 = [0.25, 0.5, 0.75, 0.125];

    const noNoise = { noiseFloor: 0.0 };
    const noDrift = { driftDepths: [0, 0, 0, 0], driftBases: [0, 0, 0, 0] };

    return [
        // ── 1. Pure coupling, sine input ─────────────────────────────────────
        {
            name: 'sine_default',
            N: 128, n_cycles: 10,
            signal: 'sine',
            params: {
                ...RingCouplerX.defaultParams(),
                phiFracs: phiExact128,
                ...noDrift, ...noNoise,
            },
        },

        // ── 2. Saw wave, non-trivial etasCub ─────────────────────────────────
        {
            name: 'saw_nonlin',
            N: 128, n_cycles: 10,
            signal: 'saw',
            params: {
                ...RingCouplerX.defaultParams(),
                phiFracs: phiExact128,
                etasAB: [0.03, 0.02, 0.01, 0.005],
                etasCub: [0.02, -0.01, 0.005, 0.0],
                ...noDrift, ...noNoise,
            },
        },

        // ── 3. Spectral tilt ─────────────────────────────────────────────────
        {
            name: 'sine_tilt',
            N: 128, n_cycles: 10,
            signal: 'sine',
            params: {
                ...RingCouplerX.defaultParams(),
                phiFracs: phiExact128,
                tilts: [0.4, 0.3, 0.2, 0.1],
                ...noDrift, ...noNoise,
            },
        },

        // ── 4. Dispersion ────────────────────────────────────────────────────
        {
            name: 'sine_dispersion',
            N: 128, n_cycles: 10,
            signal: 'sine',
            params: {
                ...RingCouplerX.defaultParams(),
                phiFracs: phiExact128,
                dispCoeff: 0.7,
                dispStages: 3,
                ...noDrift, ...noNoise,
            },
        },

        // ── 5. B↔B Givens (xModThetas non-zero) ─────────────────────────────
        {
            name: 'sine_xmod_givens',
            N: 128, n_cycles: 10,
            signal: 'sine',
            params: {
                ...RingCouplerX.defaultParams(),
                phiFracs: phiExact128,
                xModThetas: [0.015, 0.01, 0.0, 0.0],
                ...noDrift, ...noNoise,
            },
        },

        // ── 6. Cross-ring FM (xModStrengths non-zero) ────────────────────────
        {
            name: 'sine_xmod_fm',
            N: 128, n_cycles: 10,
            signal: 'sine',
            params: {
                ...RingCouplerX.defaultParams(),
                phiFracs: phiExact128,
                xModStrengths: [0.04, -0.02, 0.01, 0.0],
                ...noDrift, ...noNoise,
            },
        },

        // ── 7. Quasiperiodic drift (non-zero drift) ───────────────────────────
        {
            name: 'sine_drift',
            N: 128, n_cycles: 20,
            signal: 'sine',
            params: {
                ...RingCouplerX.defaultParams(),
                phiFracs: phiExact128,
                driftBases: [0.18, 0.291, 0.471, 0.762],
                driftDepths: [0.05, 0.05, 0.05, 0.05],
                ...noNoise,
            },
        },

        // ── 8. Everything on, N=256 ───────────────────────────────────────────
        {
            name: 'multisine_full',
            N: 256, n_cycles: 20,
            signal: 'multisine',
            params: {
                ...RingCouplerX.defaultParams(),
                phiFracs: phiExact256,
                etasAB: [0.03, 0.02, 0.01, 0.005],
                etasCub: [0.01, 0.0, 0.0, 0.0],
                tilts: [0.2, 0.1, 0.0, 0.0],
                dispCoeff: 0.5,
                dispStages: 3,
                xModThetas: [0.01, 0.0, 0.0, 0.0],
                xModStrengths: [0.03, 0.0, 0.0, 0.0],
                driftBases: [0.18, 0.291, 0.471, 0.762],
                driftDepths: [0.05, 0.05, 0.05, 0.05],
                ...noNoise,
            },
        },

        // ── 9. Square wave, heavy nonlinearity (stress test) ─────────────────
        {
            name: 'square_heavy',
            N: 128, n_cycles: 15,
            signal: 'square',
            params: {
                ...RingCouplerX.defaultParams(),
                phiFracs: phiExact128,
                thetas: [0.04, 0.025, 0.016, 0.01],
                strengths: [1.0, 1.0, 1.0, 1.0],
                etasA: [0.04, -0.02, 0.01, 0.0],
                etasAB: [0.04, 0.02, 0.01, 0.005],
                etasSq: [0.015, 0.01, 0.005, 0.0],
                etasCub: [0.04, -0.02, 0.01, 0.0],
                thetaClamp: 0.1,
                ...noDrift, ...noNoise,
            },
        },

        // ── 10. FIR shape: heavy-outer (HF rolloff emphasis) ─────────────────
        {
            name: 'sine_fir_outer',
            N: 128, n_cycles: 10,
            signal: 'sine',
            params: {
                ...RingCouplerX.defaultParams(),
                firTaps: [0.1, 0.2, 0.3, 0.25, 0.2],   // outer-heavy, pre-normalised by RCX ctor
                phiFracs: phiExact128,
                ...noDrift, ...noNoise,
            },
        },
        // ── 11. Long-run: 1000 cycles, exact-phi, no noise ────────────────────
        //    Used for: amplitude/spectral stability test over deep evolution.
        //    Python should stay within float32 precision (no drift, exact phi).
        {
            name: 'long_run_1000',
            N: 128, n_cycles: 1000,
            signal: 'sine',
            params: {
                ...RingCouplerX.defaultParams(),
                phiFracs: phiExact128,
                etasAB: [0.03, 0.02, 0.01, 0.005],
                etasCub: [0.01, 0.0, 0.0, 0.0],
                tilts: [0.1, 0.05, 0.0, 0.0],
                dispCoeff: 0.3,
                dispStages: 2,
                ...noDrift, ...noNoise,
            },
        },];
}

// ── Run a test case ──────────────────────────────────────────────────────────

function runCase(tc) {
    const { name, N, n_cycles, signal, params } = tc;

    const signals = { sine: sineWave, saw: sawWave, square: squareWave, multisine: multiSine };
    const initBuf = signals[signal](N);
    const buffer = new Float32Array(N);
    buffer.set(initBuf);

    const rcx = new RingCouplerX(buffer, { params });

    // Capture hidden B snapshots BEFORE any advances so Python can start
    // from an identical state.
    const bInits = rcx.hiddenSnapshot().map(b => Array.from(b));

    // Record initial state (before any advanceCycle)
    const cycles = [Array.from(buffer)];

    for (let c = 0; c < n_cycles; c++) {
        rcx.advanceCycle();
        cycles.push(Array.from(buffer));
    }

    return {
        name,
        N,
        n_cycles,
        signal,
        params: JSON.parse(JSON.stringify(params)),   // deep copy
        initial_buffer: Array.from(initBuf),
        b_inits: bInits,
        cycles,   // [n_cycles+1] arrays, index 0 = before first advance
    };
}

// ── Main ─────────────────────────────────────────────────────────────────────

const results = buildTestCases().map(runCase);
process.stdout.write(JSON.stringify(results, null, 2));
process.stderr.write(`\nWrote ${results.length} test cases.\n`);
