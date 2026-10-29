// ========================================================================
//  ALGORITHM CLASSES (Model)
//
//  Each class receives pre-built buffers and parameters from the driver.
//  Minimal interface:
//    .buffer          — Float32Array of output samples (read by driver)
//    .advanceCycle()  — evolve state by one cycle
//    .snapshot()      — return a Float32Array for visualization
// ========================================================================

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

/**
 * Spring-force boundary clamping for PDE fields.
 * Pulls each field toward its equilibrium via quadratic-tapered spring.
 */
function applyBoundaryClamping({ fields, equilibria, N, clampL, clampR, taper, dt }) {
    if (clampL <= 0 && clampR <= 0) return;
    const zone = Math.max(1, Math.round(N * clamp(taper, 1, 50) / 100));
    for (let j = 0; j < zone; j++) {
        const t = j / zone;
        const w = (1 - t) * (1 - t);
        const kL = clampL * w * dt;
        const kR = clampR * w * dt;
        const ri = N - 1 - j;
        for (let f = 0; f < fields.length; f++) {
            const eq = equilibria[f];
            fields[f][j] += (eq - fields[f][j]) * kL;
            fields[f][ri] += (eq - fields[f][ri]) * kR;
        }
    }
}


// --- Nonlinear Acoustics (two-field: pressure + velocity) ---------------
//  ∂p/∂t + c²·∂v/∂x = ν·∂²p/∂x²
//  ∂v/∂t + ∂p/∂x + α·v·∂v/∂x = ν·∂²v/∂x²
class AcousticsSynth {
    static group = 'Acoustics';
    static baseParams = [
        { key: 'viscosity', label: 'Viscosity', min: 0.0001, max: 2, default: 0.1, step: 0.001, format: v => Number(v).toFixed(4) },
        { key: 'nonlinearity', label: 'Nonlinearity', min: 0, max: 1, default: 0.1, step: 0.01, format: v => Number(v).toFixed(2) },
        { key: 'stepsPerCycle', label: 'Steps/Cycle', min: 1, max: 16, default: 4, step: 1, isInt: true },
    ];
    static createFromOpts(buffer, opts, { isDoubleEnded }) {
        const velocity = new Float32Array(buffer.length);
        return new AcousticsSynth(buffer, velocity, {
            isOpen: !!isDoubleEnded,
            nu: clamp(opts.viscosity ?? 0.1, 0.0001, 2.0),
            alpha: clamp(opts.nonlinearity ?? 0.1, 0, 1),
            stepsPerCycle: Math.max(1, Math.round(opts.stepsPerCycle ?? 4)),
            clampL: clamp(opts.clampL ?? 0, 0, 1),
            clampR: clamp(opts.clampR ?? 0, 0, 1),
            taper: opts.taper ?? 5,
        });
    }

    constructor(pressure, velocity, { isOpen, nu, alpha, stepsPerCycle, clampL, clampR, taper }) {
        this.buffer = pressure;   // pressure IS the output buffer
        this.v = velocity;
        this.N = pressure.length;
        this.isOpen = isOpen;
        this.nu = nu;
        this.alpha = alpha;
        this.stepsPerCycle = stepsPerCycle;
        this.clampL = clampL;
        this.clampR = clampR;
        this.taper = taper;
        this._pP = new Float32Array(pressure.length);
        this._vP = new Float32Array(velocity.length);
    }

    advanceCycle() {
        const { buffer: p, v, nu, alpha, stepsPerCycle, isOpen, N,
            clampL, clampR, taper } = this;

        for (let step = 0; step < stepsPerCycle; step++) {
            let maxSpeed = 0;
            for (let n = 0; n < N; n++) {
                const av = alpha * v[n];
                const spd = 0.5 * (Math.abs(av) + Math.sqrt(av * av + 4));
                if (spd > maxSpeed) maxSpeed = spd;
            }
            const dt = 0.4 / Math.max(maxSpeed, 1e-6);

            const pP = this._pP; pP.set(p);
            const vP = this._vP; vP.set(v);

            for (let n = 0; n < N; n++) {
                let pL, vL, pR, vR;
                if (isOpen) {
                    pL = n > 0 ? pP[n - 1] : pP[0];
                    vL = n > 0 ? vP[n - 1] : -vP[0];
                    pR = n < N - 1 ? pP[n + 1] : pP[N - 1];
                    vR = n < N - 1 ? vP[n + 1] : -vP[N - 1];
                } else {
                    pL = pP[(n - 1 + N) % N]; vL = vP[(n - 1 + N) % N];
                    pR = pP[(n + 1) % N]; vR = vP[(n + 1) % N];
                }
                const pC = pP[n], vC = vP[n];

                const fp_L = vL, fv_L = pL + alpha * vL * vL * 0.5;
                const fp_C = vC, fv_C = pC + alpha * vC * vC * 0.5;
                const fp_R = vR, fv_R = pR + alpha * vR * vR * 0.5;

                const avC = alpha * vC, avR2 = alpha * vR, avL2 = alpha * vL;
                const spdC = 0.5 * (Math.abs(avC) + Math.sqrt(avC * avC + 4));
                const aR = Math.max(spdC, 0.5 * (Math.abs(avR2) + Math.sqrt(avR2 * avR2 + 4)));
                const aL = Math.max(0.5 * (Math.abs(avL2) + Math.sqrt(avL2 * avL2 + 4)), spdC);

                p[n] = pC - dt * (0.5 * (fp_C + fp_R) - 0.5 * aR * (pR - pC)
                    - 0.5 * (fp_L + fp_C) + 0.5 * aL * (pC - pL));
                v[n] = vC - dt * (0.5 * (fv_C + fv_R) - 0.5 * aR * (vR - vC)
                    - 0.5 * (fv_L + fv_C) + 0.5 * aL * (vC - vL));

                p[n] += nu * dt * (pL - 2 * pC + pR);
                v[n] += nu * dt * (vL - 2 * vC + vR);
            }

            applyBoundaryClamping({
                fields: [p, v], equilibria: [0, 0],
                N, clampL, clampR, taper, dt,
            });
        }
    }

    snapshot() { return new Float32Array(this.buffer); }
}

// --- 1-D Maxwell Equations (FDTD Yee scheme) ----------------------------
//  ∂E/∂t = (1/ε) ∂H/∂x − (σ/ε) E
//  ∂H/∂t = (1/μ) ∂E/∂x
// Two staggered fields: E (electric) at integer nodes, H (magnetic) at half-integer.
// Output is E field. Wave speed c = 1/√(εμ), conductivity σ provides loss.
class MaxwellSynth {
    static group = 'Maxwell';
    static baseParams = [
        { key: 'speed', label: 'Wave Speed', min: 0.1, max: 10, default: 1.0, step: 0.1, format: v => Number(v).toFixed(1) },
        { key: 'conductivity', label: 'Conductivity', min: 0, max: 2.0, default: 0, step: 0.001, format: v => Number(v).toFixed(3) },
        { key: 'damping', label: 'Damping', min: 0, max: 0.5, default: 0, step: 0.001, format: v => Number(v).toFixed(3) },
        { key: 'stepsPerCycle', label: 'Steps/Cycle', min: 1, max: 16, default: 1, step: 1, isInt: true },
    ];
    static createFromOpts(buffer, opts, { isDoubleEnded }) {
        const isCircular = !isDoubleEnded;
        const N = buffer.length;
        const NH = isCircular ? N : N - 1;
        const H = new Float32Array(NH);
        const outBuf = new Float32Array(N);
        for (let i = 0; i < N; i++) outBuf[i] = buffer[i];
        return new MaxwellSynth(buffer, H, outBuf, {
            isCircular,
            c: clamp(opts.speed ?? 1.0, 0.1, 10),
            sigma: clamp(opts.conductivity ?? 0, 0, 2.0),
            damping: clamp(opts.damping ?? 0, 0, 0.5),
            stepsPerCycle: Math.max(1, Math.round(opts.stepsPerCycle ?? 1)),
            clampL: clamp(opts.clampL ?? 0, 0, 1),
            clampR: clamp(opts.clampR ?? 0, 0, 1),
            taper: opts.taper ?? 5,
        });
    }

    constructor(E, H, outputBuffer, { isCircular, c, sigma, damping, stepsPerCycle,
        clampL, clampR, taper }) {
        this.E = E;
        this.H = H;
        this.buffer = outputBuffer;
        this.N = E.length;          // E has N points
        this.NH = H.length;         // H has N-1 (open) or N (circular) points
        this.isCircular = isCircular;
        this.c = c;
        this.sigma = sigma;
        this.damping = damping;     // spatial diffusion coefficient for HF damping
        this.stepsPerCycle = stepsPerCycle;
        this.clampL = clampL;
        this.clampR = clampR;
        this.taper = taper;
        this._Ep = new Float32Array(E.length);
    }

    advanceCycle() {
        const { E, H, N, NH, isCircular, c, sigma, damping, stepsPerCycle,
            clampL, clampR, taper } = this;

        // CFL: dt ≤ dx/c, with dx = 1
        const dt = 0.9 / Math.max(c, 1e-6);
        // Loss coefficient (semi-implicit for stability)
        const lossE = 1 / (1 + 0.5 * sigma * dt);
        const lossEprev = (1 - 0.5 * sigma * dt) * lossE;
        const dtc = c * dt;

        for (let step = 0; step < stepsPerCycle; step++) {
            // --- Update H (staggered at i+0.5) ---
            // H[i] sits between E[i] and E[i+1]
            if (isCircular) {
                for (let i = 0; i < NH; i++) {
                    H[i] += dtc * (E[(i + 1) % N] - E[i]);
                }
            } else {
                for (let i = 0; i < NH; i++) {
                    H[i] += dtc * (E[i + 1] - E[i]);
                }
            }

            // --- Update E ---
            if (isCircular) {
                for (let i = 0; i < N; i++) {
                    const hR = H[i % NH];
                    const hL = H[((i - 1) + NH) % NH];
                    E[i] = lossEprev * E[i] + lossE * dtc * (hR - hL);
                }
            } else {
                // Reflective boundaries via ghost cells:
                // E mirrors at wall (like pressure), H negates (like velocity).
                // This gives a rigid-wall reflection where E has an antinode at
                // the boundary — waves bounce back naturally.
                for (let i = 0; i < N; i++) {
                    const hR = i < NH ? H[i] : -H[NH - 1];
                    const hL = i > 0 ? H[i - 1] : -H[0];
                    E[i] = lossEprev * E[i] + lossE * dtc * (hR - hL);
                }
            }

            // --- HF damping via spatial diffusion on E ---
            // Applies a Laplacian smoothing: E[i] += d*(E[i-1] - 2*E[i] + E[i+1])
            // This selectively attenuates high spatial frequencies (short wavelengths).
            if (damping > 0) {
                const Ep = this._Ep; Ep.set(E);
                for (let i = 0; i < N; i++) {
                    const eL = isCircular ? Ep[(i - 1 + N) % N] : (i > 0 ? Ep[i - 1] : Ep[0]);
                    const eR = isCircular ? Ep[(i + 1) % N] : (i < N - 1 ? Ep[i + 1] : Ep[N - 1]);
                    E[i] += damping * dt * (eL - 2 * Ep[i] + eR);
                }
            }

            // Boundary clamping (for double-ended mode)
            // Clamp both E and H to prevent reflected energy from re-entering.
            if (!isCircular) {
                applyBoundaryClamping({
                    fields: [E], equilibria: [0],
                    N, clampL, clampR, taper, dt,
                });
                applyBoundaryClamping({
                    fields: [H], equilibria: [0],
                    N: NH, clampL, clampR, taper, dt,
                });
            }
        }

        // Copy E to output buffer
        for (let i = 0; i < N; i++) this.buffer[i] = E[i];
    }

    snapshot() { return new Float32Array(this.buffer); }
}

// --- Waveguide (circular & fixed ends) ----------------------------------
// Two counter-propagating delay lines R (right-going) and L (left-going).
//
// Circular mode: R and L are stored statically; propagation is tracked via
// accumulating offsets.  buffer[i] = R[(i−off)%N] + L[(i+off)%N].
// This guarantees the output buffer is always periodic (no seam artifacts)
// and propagation is visible as the two waves slide past each other.
//
// Fixed-end mode: physical shift registers with reflection/LP at the ends.
class WaveguideSynth {
    static isWaveguide = true;
    static group = 'Waveguide';
    static _sharedParams = [
        { key: 'wgSteps', label: 'Steps/Cycle', min: 1, max: 16, default: 4, step: 1, isInt: true },
        { key: 'wgLPL', label: 'LP Left', min: 0, max: 1, default: 0, step: 0.01, format: v => Number(v).toFixed(2) },
        { key: 'wgLPR', label: 'LP Right', min: 0, max: 1, default: 0, step: 0.01, format: v => Number(v).toFixed(2) },
    ];
    static _circularParams = [
        { key: 'wgAP', label: 'AP Coeff (L→R)', min: 0, max: 1, default: 0.5, step: 0.01, format: v => Number(v).toFixed(2) },
        { key: 'wgAPR', label: 'AP Coeff (R→L)', min: 0, max: 1, default: 0.5, step: 0.01, format: v => Number(v).toFixed(2) },
    ];
    static _fixedParams = [
        { key: 'wgDecayTime', label: 'Decay (s)', min: 0.1, max: 30, default: 2.0, step: 0.1, format: v => Number(v).toFixed(1) + 's' },
        { key: 'wgBalance', label: 'Balance', min: -1, max: 1, default: 0, step: 0.01, format: v => Number(v).toFixed(2) },
        { key: 'wgClamp', label: 'End Clamp', min: 0, max: 1, default: 0.5, step: 0.01, format: v => Number(v).toFixed(2) },
    ];

    constructor(R, L, outputBuffer, { isCircular, reflL, reflR,
        lpCoeffL, lpCoeffR, clampStrength, stepsPerCycle }) {
        this.R = R;
        this.L = L;
        this.buffer = outputBuffer;
        this.N = R.length;
        this.isCircular = isCircular;
        this.reflL = reflL;
        this.reflR = reflR;
        this.lpCoeffL = lpCoeffL;
        this.lpCoeffR = lpCoeffR;
        this.stepsPerCycle = stepsPerCycle;

        // Circular mode: rotation offset (accumulates over cycles)
        this._offset = 0;

        // Allpass-based boundary model (fixed-end mode only)
        this.apCoeff = clamp(2 * clampStrength - 1, -0.995, 0.995);
        this.apPrevInL = 0;
        this.apPrevOutL = 0;
        this.apPrevInR = 0;
        this.apPrevOutR = 0;

        this.lpStateL = 0;
        this.lpStateR = 0;
    }

    advanceCycle() {
        const { R, L, N, isCircular, reflL, reflR,
            lpCoeffL, lpCoeffR, stepsPerCycle, buffer } = this;

        if (isCircular) {
            // --- Circular mode: offset-based rotation ---
            this._offset = (this._offset + stepsPerCycle) % N;

            // Apply LP damping to R and L (two-pass preserves ring continuity).
            // R: two-pass LP with lpCoeffR
            if (lpCoeffR > 1e-6) {
                const g = 1 - lpCoeffR, lp = lpCoeffR;
                let prev = R[N - 1];
                for (let i = 0; i < N; i++) { R[i] = g * R[i] + lp * prev; prev = R[i]; }
                prev = R[0];
                for (let i = N - 1; i >= 0; i--) { R[i] = g * R[i] + lp * prev; prev = R[i]; }
            }
            // L: two-pass LP with lpCoeffL
            if (lpCoeffL > 1e-6) {
                const g = 1 - lpCoeffL, lp = lpCoeffL;
                let prev = L[N - 1];
                for (let i = 0; i < N; i++) { L[i] = g * L[i] + lp * prev; prev = L[i]; }
                prev = L[0];
                for (let i = N - 1; i >= 0; i--) { L[i] = g * L[i] + lp * prev; prev = L[i]; }
            }

            // R-coupled all-pass on L: modulated by odd-symmetric R.
            const a = this.apCoeff;
            if (Math.abs(a) > 1e-6) {
                const half = N >> 1;
                let prevIn = L[N - 1];
                let prevOut = L[N - 1];
                for (let lap = 0; lap < 2; lap++) {
                    for (let i = 0; i < N; i++) {
                        var b = R[i] + 1;
                        const y = a * b * (L[i] - prevOut) + prevIn;
                        prevIn = L[i];
                        prevOut = y;
                    }
                }
                for (let i = 0; i < N; i++) {
                    const b = R[i] + 1;
                    const y = a * b * (L[i] - prevOut) + prevIn;
                    prevIn = L[i];
                    prevOut = y;
                    L[i] = y;
                }
            }

            // L-coupled all-pass on R: modulated by odd-symmetric L, runs in reverse.
            const aR = this.apCoeffR;
            if (Math.abs(aR) > 1e-6) {
                const half = N >> 1;
                let prevIn = R[0];
                let prevOut = R[0];
                for (let lap = 0; lap < 2; lap++) {
                    for (let i = N - 1; i >= 0; i--) {
                        const b = (L[i] - L[(i + half) % N]) * 0.5;
                        const y = aR * b * (R[i] - prevOut) + prevIn;
                        prevIn = R[i];
                        prevOut = y;
                    }
                }
                for (let i = N - 1; i >= 0; i--) {
                    const b = (L[i] - L[(i + half) % N]) * 0.5;
                    const y = aR * b * (R[i] - prevOut) + prevIn;
                    prevIn = R[i];
                    prevOut = y;
                    R[i] = y;
                }
            }

            // Composite into output buffer.
            for (let i = 0; i < N; i++) {
                buffer[i] = L[i];
            }
        } else {
            // --- Fixed-end mode: physical shift with reflections ---
            const a = this.apCoeff;

            for (let tick = 0; tick < stepsPerCycle; tick++) {
                const outR = R[N - 1];
                const outL = L[0];

                // // Shift R right, L left
                for (let j = N - 1; j > 1; j--) R[j] = R[j - 2];
                for (let j = 0; j < N - 1; j++) L[j] = L[j + 1];

                // LP filter → allpass → negate → scale
                const filtL = (1 - lpCoeffL) * outL + lpCoeffL * this.lpStateL;
                this.lpStateL = filtL;
                const apOutL = a * filtL + this.apPrevInL - a * this.apPrevOutL;
                this.apPrevInL = filtL;
                this.apPrevOutL = apOutL;
                R[0] = -reflL * apOutL;

                const filtR = (1 - lpCoeffR) * outR + lpCoeffR * this.lpStateR;
                this.lpStateR = filtR;
                const apOutR = a * filtR + this.apPrevInR - a * this.apPrevOutR;
                this.apPrevInR = filtR;
                this.apPrevOutR = apOutR;
                L[N - 1] = -reflR * apOutR;
            }

            // LP smoothing on open-ended arrays
            const lp = Math.max(lpCoeffL, lpCoeffR);
            if (lp > 1e-6) {
                const g = 1 - lp;
                let prev = R[0];
                for (let i = 1; i < N; i++) {
                    R[i] = g * R[i] + lp * prev;
                    prev = R[i];
                }
                prev = L[N - 1];
                for (let i = N - 2; i >= 0; i--) {
                    L[i] = g * L[i] + lp * prev;
                    prev = L[i];
                }
            }

            // Write R+L superposition to output buffer
            for (let i = 0; i < N; i++) buffer[i] = R[i] + L[i];
        }
    }

    snapshot() {
        if (this.isCircular) {
            const { R, L, N, _offset: off } = this;
            const snap = new Float32Array(N);
            for (let i = 0; i < N; i++) {
                snap[i] = L[i]
            }
            return snap;
        }
        const snap = new Float32Array(this.N);
        for (let i = 0; i < this.N; i++) snap[i] = this.R[i] + this.L[i];
        return snap;
    }
}

// --- Simple Nonlinear (delay-line with nonlinear read offset) -----------
//  Sample-by-sample circular delay line with amplitude-dependent read
//  offset (generates even harmonics / "growl") and one-pole LP damping.
class SimpleNonlinearSynth {
    static group = 'Nonlinear';
    static baseParams = [
        { key: 'alpha', label: 'NL Alpha', min: 0, max: 50, default: 5, step: 0.1, format: v => Number(v).toFixed(1) },
        { key: 'damping', label: 'Damping', min: 0, max: 0.999, default: 0.5, step: 0.001, format: v => Number(v).toFixed(3) },
        { key: 'stepsPerCycle', label: 'Steps/Cycle', min: 1, max: 16, default: 1, step: 1, isInt: true },
    ];
    static createFromOpts(buffer, opts) {
        return new SimpleNonlinearSynth(buffer, {
            alpha: clamp(opts.alpha ?? 5, 0, 50),
            damping: clamp(opts.damping ?? 0.5, 0, 0.999),
            stepsPerCycle: Math.max(1, Math.round(opts.stepsPerCycle ?? 1)),
        });
    }

    constructor(buffer, { alpha, damping, stepsPerCycle }) {
        this.buffer = buffer;
        this.N = buffer.length;
        this.alpha = alpha;         // Nonlinear read-offset intensity
        this.damping = damping;     // One-pole LP coefficient (0=bright, 1=dark)
        this.stepsPerCycle = stepsPerCycle || 1;
        this.lpState = 0;
    }

    advanceCycle() {
        const { buffer, N, alpha, damping, stepsPerCycle } = this;

        for (let step = 0; step < stepsPerCycle; step++) {
            for (let i = 0; i < N; i++) {
                // 1. Dynamic read offset from current sample (even harmonics)
                const cur = buffer[i];
                const offset = alpha * (cur * cur);

                // 2. Read with fractional offset (linear interpolation)
                let readIdx = ((i + 1 + offset) % N);
                if (readIdx < 0) readIdx += N;
                const i0 = Math.floor(readIdx);
                const i1 = (i0 + 1) % N;
                const frac = readIdx - i0;
                let sample = buffer[i0] * (1 - frac) + buffer[i1] * frac;

                // 3. One-pole LP filter (HF damping)
                this.lpState = sample * (1 - damping) + this.lpState * damping;
                sample = this.lpState;

                // 4. Feedback with tiny global loss
                buffer[i] = sample * 0.999;
            }
        }
    }

    snapshot() { return new Float32Array(this.buffer); }
}

// ── CustomSynth ──────────────────────────────────────────────────────────────
// Passthrough skeleton — replace advanceCycle() with your own algorithm.
// The buffer is a circular Float32Array of length N (the delay line).
// Each call to advanceCycle() runs once per rendered chunk (≈ every 128 samples).
class CustomSynth {
    // ── Registry metadata (read by the UI + driver automatically) ──────────
    static group = 'Custom';
    static baseParams = [
        { key: 'gain', label: 'Gain', min: 0, max: 1, default: 0.999, step: 0.001, format: v => Number(v).toFixed(3) },
        { key: 'param1', label: 'Param 1', min: 0, max: 1, default: 0.5, step: 0.01, format: v => Number(v).toFixed(2) },
        { key: 'param2', label: 'Param 2', min: 0, max: 1, default: 0.5, step: 0.01, format: v => Number(v).toFixed(2) },
    ];
    static createFromOpts(buffer, opts) {
        return new CustomSynth(buffer, {
            gain: clamp(opts.gain ?? 0.999, 0, 1),
            param1: clamp(opts.param1 ?? 0.5, 0, 1),
            param2: clamp(opts.param2 ?? 0.5, 0, 1),
        });
    }

    // ── Instance ────────────────────────────────────────────────────────────
    constructor(buffer, { gain, param1, param2 }) {
        this.buffer = buffer;
        this.N = buffer.length;
        // TODO: add any state you need (e.g. filter memory, position counter…)
        this.gain = gain;    // Overall output scale (0–1)
        this.param1 = param1;  // Replace with a meaningful parameter
        this.param2 = param2;  // Replace with a meaningful parameter
    }

    advanceCycle() {
        const { buffer, N, gain } = this;

        for (let i = 0; i < N; i++) {
            buffer[i] = buffer[i] * gain;
        }
    }

    snapshot() { return new Float32Array(this.buffer); }
}


// ── EvenGradientSynth ───────────────────────────────────────────────────────
// Dissipative sample-space waveform evolution with hidden even-only state.
//
// The visible buffer q is treated as one cycle of waveform, not as a physical
// string displacement.  A hidden field h tracks half-cycle-symmetric local
// features (q², slope², curvature²), and is fed back into q to emphasise even
// harmonics.  A second hidden field r acts as slow fatigue/adaptation so the
// tone can keep drifting instead of locking to a static attractor quickly.
//
// Everything is O(N) per cycle.  The only non-local lookup is n + N/2, which
// is still linear-time overall and gives exact even-harmonic symmetry.
class EvenGradientSynth {
    static group = 'Gradient Flow';
    static baseParams = [
        { key: 'evenGain', label: 'Even Gain', min: 0, max: 0.45, default: 0.08, step: 0.001, format: v => Number(v).toFixed(3) },
        { key: 'fundamentalKeep', label: 'Fundamental', min: 0.85, max: 1.0, default: 0.985, step: 0.0005, format: v => Number(v).toFixed(4) },
        { key: 'smooth', label: 'Smooth', min: 0, max: 0.5, default: 0.08, step: 0.001, format: v => Number(v).toFixed(3) },
        { key: 'decay', label: 'Decay', min: 0.9, max: 1.0, default: 0.9992, step: 0.0001, format: v => Number(v).toFixed(4) },
        { key: 'featureSlope', label: 'Feat. Slope', min: 0, max: 4, default: 0.2, step: 0.01, format: v => Number(v).toFixed(2) },
        { key: 'featureCurve', label: 'Feat. Curve', min: 0, max: 4, default: 0.35, step: 0.01, format: v => Number(v).toFixed(2) },
        { key: 'hiddenRate', label: 'Hidden Rate', min: 0, max: 0.5, default: 0.08, step: 0.001, format: v => Number(v).toFixed(3) },
        { key: 'hiddenDiffusion', label: 'Hidden Diff.', min: 0, max: 0.25, default: 0.015, step: 0.001, format: v => Number(v).toFixed(3) },
        { key: 'hiddenDrift', label: 'Hidden Drift', min: 0, max: 0.25, default: 0.03, step: 0.001, format: v => Number(v).toFixed(3) },
        { key: 'latentCoupling', label: 'Latent Cpl', min: 0, max: 0.25, default: 0.035, step: 0.001, format: v => Number(v).toFixed(3) },
        { key: 'latentDecay', label: 'Latent Decay', min: 0.9, max: 1.0, default: 0.997, step: 0.0005, format: v => Number(v).toFixed(4) },
    ];
    static createFromOpts(buffer, opts) {
        return new EvenGradientSynth(buffer, {
            evenGain: clamp(opts.evenGain ?? 0.08, 0, 0.45),
            fundamentalKeep: clamp(opts.fundamentalKeep ?? 0.985, 0.85, 1.0),
            smooth: clamp(opts.smooth ?? 0.08, 0, 0.5),
            decay: clamp(opts.decay ?? 0.9992, 0.9, 1.0),
            featureSlope: clamp(opts.featureSlope ?? 0.2, 0, 4),
            featureCurve: clamp(opts.featureCurve ?? 0.35, 0, 4),
            hiddenRate: clamp(opts.hiddenRate ?? 0.08, 0, 0.5),
            hiddenDiffusion: clamp(opts.hiddenDiffusion ?? 0.015, 0, 0.25),
            hiddenDrift: clamp(opts.hiddenDrift ?? 0.03, 0, 0.25),
            latentCoupling: clamp(opts.latentCoupling ?? 0.035, 0, 0.25),
            latentDecay: clamp(opts.latentDecay ?? 0.997, 0.9, 1.0),
        });
    }

    constructor(buffer, {
        evenGain, fundamentalKeep, smooth, decay,
        featureSlope, featureCurve,
        hiddenRate, hiddenDiffusion, hiddenDrift,
        latentCoupling, latentDecay,
    }) {
        this.buffer = buffer;
        this.N = buffer.length;
        this.H = buffer.length >> 1;
        this.evenGain = evenGain;
        this.fundamentalKeep = fundamentalKeep;
        this.smooth = smooth;
        this.decay = decay;
        this.featureSlope = featureSlope;
        this.featureCurve = featureCurve;
        this.hiddenRate = hiddenRate;
        this.hiddenDiffusion = hiddenDiffusion;
        this.hiddenDrift = hiddenDrift;
        this.latentCoupling = latentCoupling;
        this.latentDecay = latentDecay;

        this.h = new Float32Array(this.H);
        this._hNext = new Float32Array(this.H);
        this.z = new Float32Array(this.H);
        this._zNext = new Float32Array(this.H);
        this._seamTmp = new Float32Array(17);

        // Seed the hidden even field from the initial waveform's sign-blind
        // local energy so the starting wavetable strongly influences evolution.
        for (let p = 0; p < this.H; p++) {
            const n = p;
            const m = p + this.H;
            const nL = (n - 1 + this.N) % this.N;
            const nR = (n + 1) % this.N;
            const mL = (m - 1 + this.N) % this.N;
            const mR = (m + 1) % this.N;
            const d1n = buffer[nR] - buffer[n];
            const d1m = buffer[mR] - buffer[m];
            const d2n = buffer[nR] - 2 * buffer[n] + buffer[nL];
            const d2m = buffer[mR] - 2 * buffer[m] + buffer[mL];
            const feat = 0.5 * (buffer[n] * buffer[n] + buffer[m] * buffer[m])
                + 0.5 * featureSlope * (d1n * d1n + d1m * d1m)
                + 0.5 * featureCurve * (d2n * d2n + d2m * d2m);
            this.h[p] = Math.tanh(2.0 * feat - 0.5);
            this.z[p] = 0.5 * (buffer[m] - buffer[n]);
        }
    }

    advanceCycle() {
        const { buffer: q, h, _hNext: hNext, z, _zNext: zNext, N, H,
            evenGain, fundamentalKeep, smooth, decay,
            featureSlope, featureCurve, hiddenRate, hiddenDiffusion, hiddenDrift,
            latentCoupling, latentDecay } = this;

        // Pass 1 (half-buffer): update hidden even state and directly mix each
        // half-cycle pair using an even hidden field h and a latent odd-state z.
        // h encourages equality across the pair (even harmonics), while z stores
        // and advects part of the pair difference around the ring so the
        // fundamental can live longer and drift instead of collapsing straight
        // to zero.
        for (let p = 0; p < H; p++) {
            const n = p;
            const m = p + H;
            const nL = (n - 1 + N) % N;
            const nR = (n + 1) % N;
            const mL = (m - 1 + N) % N;
            const mR = (m + 1) % N;
            const pL = (p - 1 + H) % H;
            const pR = (p + 1) % H;

            const d1n = q[nR] - q[n];
            const d1m = q[mR] - q[m];
            const d2n = q[nR] - 2 * q[n] + q[nL];
            const d2m = q[mR] - 2 * q[m] + q[mL];
            const feat = 0.5 * (q[n] * q[n] + q[m] * q[m])
                + 0.5 * featureSlope * (d1n * d1n + d1m * d1m)
                + 0.5 * featureCurve * (d2n * d2n + d2m * d2m);

            const pairGrad = (q[mR] - q[nR]) - (q[mL] - q[nL]);
            const target = Math.tanh(2.0 * feat - 0.5 + 0.5 * hiddenDrift * pairGrad);
            const hp = h[p];
            hNext[p] = hp + hiddenRate * (target - hp)
                + hiddenDiffusion * (h[pL] - 2 * hp + h[pR]);

            const even = 0.5 * (q[n] + q[m]);
            const odd = 0.5 * (q[m] - q[n]);

            const zp = z[p];
            const zLap = z[pL] - 2 * zp + z[pR];
            const zAdv = zp + hiddenDrift * (z[pL] - zp);
            const couple = latentCoupling * (0.25 + 0.75 * hNext[p]);

            const oddCoupled = odd + couple * zAdv;
            zNext[p] = latentDecay * (zAdv - couple * odd)
                + 0.5 * hiddenDiffusion * zLap;

            const g = clamp(evenGain * (0.5 + 0.5 * hNext[p]), 0, 0.35);
            const evenNext = even + g * oddCoupled;
            const oddNext = fundamentalKeep * oddCoupled;

            q[n] = evenNext - oddNext;
            q[m] = evenNext + oddNext;
        }
        this.h = hNext;
        this._hNext = h;
        this.z = zNext;
        this._zNext = z;

        // Quick seam fix: locally smooth around the two effective join points
        // created by half-cycle pair forcing (0 and H). This reduces value,
        // slope, and curvature kinks without changing the global algorithm.
        const seamHalfWidth = 8;
        const seamCenters = [0, H];
        const seamTmp = this._seamTmp;
        const seamBlend = 0.35;
        for (let s = 0; s < seamCenters.length; s++) {
            const center = seamCenters[s];
            for (let k = -seamHalfWidth; k <= seamHalfWidth; k++) {
                const n = (center + k + N) % N;
                const nL = (n - 1 + N) % N;
                const nR = (n + 1) % N;
                seamTmp[k + seamHalfWidth] = 0.25 * q[nL] + 0.5 * q[n] + 0.25 * q[nR];
            }
            for (let k = -seamHalfWidth; k <= seamHalfWidth; k++) {
                const n = (center + k + N) % N;
                q[n] = q[n] * (1 - seamBlend) + seamTmp[k + seamHalfWidth] * seamBlend;
            }
        }

        // Pass 2 (full buffer): a single cheap one-pole smoothing pass gives
        // Karplus-Strong-like loss and a little directional motion.
        let prev = q[N - 1];
        for (let n = 0; n < N; n++) {
            const cur = q[n];
            q[n] = decay * ((1 - smooth) * cur + smooth * prev);
            prev = cur;
        }
    }

    snapshot() { return new Float32Array(this.buffer); }
}


class LagrangianAcoustics {
    static group = 'Lagrangian';
    static baseParams = [
        { key: 'lpfCutoff', label: 'LPF Cutoff (harmonic)', min: 1, max: 42, default: 32, step: 1, isInt: true },
        { key: 'damping', label: 'Damping', min: 0.01, max: 1.0, default: 0.2, step: 0.01, format: v => Number(v).toFixed(2) },
        { key: 'nonlinearity', label: 'Nonlinearity', min: 0.0, max: 0.1, default: 0.01, step: 0.001, format: v => Number(v).toFixed(3) },
        // Noise ADSR envelope
        { key: 'noiseLevel', label: 'Noise Level', min: 0, max: 1, default: 0.02, step: 0.01, format: v => Number(v).toFixed(2) },
        { key: 'noiseAttack', label: 'Noise Attack', min: 0, max: 200, default: 5, step: 1, isInt: true },
        { key: 'noiseDecay', label: 'Noise Decay', min: 0, max: 500, default: 50, step: 1, isInt: true },
        { key: 'noiseSustain', label: 'Noise Sustain', min: 0, max: 1, default: 0.3, step: 0.01, format: v => Number(v).toFixed(2) },
        { key: 'noiseRelease', label: 'Noise Release', min: 0, max: 500, default: 100, step: 1, isInt: true },
        { key: 'noiseColor', label: 'Noise Color', min: 0, max: 1, default: 0.2, step: 0.01, format: v => Number(v).toFixed(2) },
        { key: 'wtAttack', label: 'WT Attack', min: 0, max: 1, default: 0, step: 0.01, format: v => { const n = Number(v); return n < 0.01 ? 'instant' : (n * 100).toFixed(0) + '%'; } },
    ];
    static createFromOpts(buffer, opts, { isDoubleEnded }) {
        return new LagrangianAcoustics(buffer, {
            lpfCutoff: Math.max(1, Math.round(opts.lpfCutoff ?? 8)),
            damping: clamp(opts.damping ?? 0.5, 0, 1),
            nonlinearity: clamp(opts.nonlinearity, 0, 1),
            noiseLevel: clamp(opts.noiseLevel ?? 0.02, 0, 1),
            noiseAttack: Math.max(0, Math.round(opts.noiseAttack ?? 5)),
            noiseDecay: Math.max(0, Math.round(opts.noiseDecay ?? 50)),
            noiseSustain: clamp(opts.noiseSustain ?? 0.3, 0, 1),
            noiseRelease: Math.max(0, Math.round(opts.noiseRelease ?? 100)),
            noiseColor: clamp(opts.noiseColor ?? 0, 0, 1),
            wtAttack: clamp(opts.wtAttack ?? 0, 0, 1),
            // SWN-port stabilizers (default off — preserves original JS behaviour)
            stabilize: !!opts.stabilize,
        });
    }

    constructor(pressure, options) {
        this.buffer = pressure;                          // Position (q)
        this.v = new Float32Array(pressure.length);      // Momentum (p)
        this.N = pressure.length;
        // Enables SWN-port stabilizers (dtt clamp + per-cycle q DC removal +
        // q safety clamp).  Off in the original JS reference; the SWN-match
        // subclass turns it on.  Pitch-adaptive buffer sizing is handled by
        // the subclass *before* construction (it passes a pre-resampled
        // buffer in as `pressure`), so this constructor stays simple.
        this._stabilize = !!options.stabilize;

        // WT Attack: 0 = seed fully at start (instant), 1 = inject per-cycle only
        this.wtAttack = clamp(options.wtAttack ?? 0, 0, 1);
        // Keep a copy of the original wavetable for per-cycle injection
        this._wtOriginal = new Float32Array(pressure.length);
        for (let n = 0; n < this.N; n++) this._wtOriginal[n] = pressure[n];

        // Noise params (set up before initial seeding so RNG is available)
        this.noiseLevel = clamp(options.noiseLevel ?? 0.02, 0, 1);
        this.noiseColor = options.noiseColor ?? 0;
        this._rngState = (Math.random() * 0xFFFFFFFF) >>> 0 || 1;
        this._noiseFilterState = 0;

        // Initial seeding: proportional to (1 - wtAttack)
        const seedAmt = 1 - this.wtAttack;
        if (seedAmt > 1e-6) {
            this._inject(seedAmt, true);
        } else {
            for (let n = 0; n < this.N; n++) {
                this.buffer[n] = 0;
                this.v[n] = 0;
            }
        }

        // Physics params
        this.lpfCutoff = options.lpfCutoff; // LPF cutoff harmonic number
        this.damping = options.damping;      // LPF wet/dry mix (shared for position & velocity)
        this.nonlinearity = options.nonlinearity;

        // ADSR envelope (times in cycles)
        this.noiseAttack = options.noiseAttack ?? 5;    // cycles
        this.noiseDecay = options.noiseDecay ?? 50;   // cycles
        this.noiseSustain = options.noiseSustain ?? 0.3;  // level
        this.noiseRelease = options.noiseRelease ?? 100;  // cycles
        this._envPhase = 'attack';  // 'attack' | 'decay' | 'sustain' | 'release' | 'off'
        this._envLevel = 0;         // current envelope amplitude [0,1]
        this._envCycle = 0;         // cycles elapsed in current phase
        this._gate = true;          // key is held

        // Cached one-pole LP coefficient for circular solve
        this._lpfCachedCutoff = -1;
        this._lpfAlpha = 0;       // one-pole coefficient
        this._lpfAlphaN = 0;      // α^N (cached)
        this._lpfScratch = new Float32Array(this.N); // temp for wet/dry mix
    }

    // Inject (1-noiseLevel)*WT + noiseLevel*noise into q (and v).
    //   amt: overall amplitude multiplier
    //   set: true → overwrite buffer (initial seed), false → add to buffer (per-cycle)
    _inject(amt, set) {
        const { buffer: q, v, N, damping } = this;
        const wtAmp = amt * (1 - this.noiseLevel);
        const wt = this._wtOriginal;
        const colorParam = this.noiseColor;
        const alpha = colorParam < 0.01 ? 0 : 1 - Math.pow(10, -4 * colorParam);
        // One-pole LP with coeff α scales RMS by √((1-α)/(1+α)).
        // Compensate so filtered noise has the same power as white noise.
        const colorGain = alpha < 1e-6 ? 1 : Math.sqrt((1 + alpha) / (1 - alpha));
        const noiseAmp = amt * this.noiseLevel * colorGain / 2;

        let state = this._noiseFilterState;
        state = 0;
        let sumNoise = 0;
        for (let n = 0; n < N; n++) {
            const white = this._xorRand();
            state = alpha * state + (1 - alpha) * white;
            const inj = wtAmp * wt[n] + noiseAmp * state;
            sumNoise += inj;
            if (set) {
                q[n] = inj;
                v[n] = 0;
            } else {
                // q[n] += inj;
                v[n] += inj * damping * 10;
            }
        }
        this._noiseFilterState = state;
        // Remove DC from noise component
        if (Math.abs(sumNoise) > 1e-9) {
            const dc = sumNoise / N;
            if (set) {
                for (let n = 0; n < N; n++) q[n] -= dc;
            } else {
                for (let n = 0; n < N; n++) {
                    v[n] -= dc * damping * 10;
                }
            }
        }
    }

    // Circular cascaded two-pole LPF (−12 dB/oct above cutoff).
    //   cutoff: harmonic number (1=fundamental, 8=8th harmonic, etc.)
    //   mix ∈ [0,1]: 0 = bypass, 1 = full filter
    //
    // Two cascaded one-pole filters, each with its own circular solve.
    // Each pole: y[n] = α·y[n-1] + (1-α)·x[n], s₀ = f / (1 − α^N).
    // Cost: ~5N (save+pole1_p1, pole1_p2, pole2_p1, pole2_p2+mix).
    _lpf(arr, cutoff, mix) {
        if (mix <= 0) return;
        const N = this.N;

        // Recompute coefficient when cutoff changes
        if (this._lpfCachedCutoff !== cutoff) {
            const omega0 = 2 * Math.PI * cutoff / N;
            this._lpfAlpha = Math.exp(-omega0);
            let aN = 1;
            const a = this._lpfAlpha;
            for (let i = 0; i < N; i++) aN *= a;
            this._lpfAlphaN = aN;
            this._lpfCachedCutoff = cutoff;
        }

        const a = this._lpfAlpha;
        const b = 1 - a;
        const invDenom = (Math.abs(1 - this._lpfAlphaN) > 1e-15) ? 1 / (1 - this._lpfAlphaN) : 0;
        const orig = this._lpfScratch;

        // Save original + pole 1 forced response (combined pass)
        let s = 0;
        for (let n = 0; n < N; n++) {
            orig[n] = arr[n];
            s = a * s + b * arr[n];
        }
        // Pole 1 circular state + filter pass
        s *= invDenom;
        for (let n = 0; n < N; n++) {
            s = a * s + b * arr[n];
            arr[n] = s;
        }

        // Pole 2 forced response
        s = 0;
        for (let n = 0; n < N; n++) s = a * s + b * arr[n];
        // Pole 2 circular state + filter + wet/dry mix
        s *= invDenom;
        const dry = 1 - mix;
        for (let n = 0; n < N; n++) {
            s = a * s + b * arr[n];
            arr[n] = dry * orig[n] + mix * s;
        }
    }

    // Advance ADSR by one cycle, return current envelope level [0,1].
    _advanceEnvelope() {
        switch (this._envPhase) {
            case 'attack': {
                const a = this.noiseAttack;
                this._envLevel = a > 0 ? Math.min(1, this._envCycle / a) : 1;
                this._envCycle++;
                if (this._envLevel >= 1) { this._envPhase = 'decay'; this._envCycle = 0; }
                break;
            }
            case 'decay': {
                const d = this.noiseDecay;
                const s = this.noiseSustain;
                this._envLevel = d > 0 ? 1 - (1 - s) * Math.min(1, this._envCycle / d) : s;
                this._envCycle++;
                if (this._envCycle >= d) { this._envPhase = 'sustain'; this._envCycle = 0; }
                break;
            }
            case 'sustain':
                this._envLevel = this.noiseSustain;
                // Stays here until noteOff()
                break;
            case 'release': {
                const r = this.noiseRelease;
                const startLevel = this._envReleaseStart ?? this._envLevel;
                this._envLevel = r > 0 ? startLevel * (1 - Math.min(1, this._envCycle / r)) : 0;
                this._envCycle++;
                if (this._envCycle >= r) { this._envPhase = 'off'; this._envLevel = 0; }
                break;
            }
            case 'off':
                this._envLevel = 0;
                break;
        }
        return this._envLevel;
    }

    // Xorshift32 → uniform float in [-1, 1]
    _xorRand() {
        let s = this._rngState;
        s ^= s << 13; s ^= s >>> 17; s ^= s << 5;
        this._rngState = s >>> 0;
        return (s / 0x80000000);  // [-1, 1]
    }

    // Called by audio-processor on key release
    noteOff() {
        if (this._envPhase !== 'release' && this._envPhase !== 'off') {
            this._envReleaseStart = this._envLevel;  // capture level at release
            this._envPhase = 'release';
            this._envCycle = 0;
            this._gate = false;
        }
    }

    advanceCycle() {
        const { buffer: q, v, N, nonlinearity, damping } = this;
        const dt = 0.02;
        let dtt = dt + q[0] * dt / 2;
        // SWN port adds a clamp here to bound the integrator timestep when
        // q[0] drifts large (can happen with strong injection / high damping).
        if (this._stabilize) {
            if (dtt < 0.005) dtt = 0.005;
            if (dtt > 0.05)  dtt = 0.05;
        }

        // Per-cycle injection: proportional to wtAttack, scaled by ADSR envelope
        const envLevel = this._advanceEnvelope();
        const injAmt = this.wtAttack * envLevel * dt;
        if (injAmt > 1e-9) {
            this._inject(injAmt, false);
        }

        for (let n = 0; n < N / 2; n++) {
            const nR = (n + N / 2) % N;
            let d = (q[nR] - q[n]);
            const f = d * nonlinearity;
            v[n]  += f * dtt - q[n]  * dtt * 0.1;
            v[nR] -= f * dtt + q[nR] * dtt * 0.1;
        }
        for (let n = 0; n < N; n++) {
            if (v[n] > 1.0) v[n] = 1.0;
            if (v[n] < -1.0) v[n] = -1.0;
            if (Math.abs(v[n]) > 0.5) {
                v[n] *= 0.99;
            }
        }
        for (let n = 0; n < N; n++) {
            q[n] += dtt * v[n];
        }

        // SWN port: per-cycle DC removal on q[] + safety clamp.  These were
        // added to the firmware to keep q from drifting / blowing up under
        // the harshest parameter combinations; the original JS does not have
        // them, so they're gated behind the stabilize flag.
        if (this._stabilize) {
            let dcQ = 0;
            for (let n = 0; n < N; n++) dcQ += q[n];
            dcQ /= N;
            for (let n = 0; n < N; n++) {
                let qn = q[n] - dcQ;
                if (qn >  2) qn =  2;
                else if (qn < -2) qn = -2;
                q[n] = qn;
            }
        }

        this._lpf(v, this.lpfCutoff / 2, this.damping);
        this._lpf(q, this.lpfCutoff, this.damping);
    }

    snapshot() { return new Float32Array(this.buffer); }
    velocitySnapshot() { return new Float32Array(this.v); }
}


// ──────────────────────────────────────────────────────────────────────────
//  Lagrangian Acoustics — SWN-Match
//
//  Variant of LagrangianAcoustics that mirrors the audio path used by the
//  4MS SWN port (src/halo.c).  Use this side-by-side with the regular
//  'Lagrangian Acoustics' algorithm to A/B audible differences caused by
//  the firmware port choices.
//
//  Differences from the original JS class:
//
//   1. Pitch-adaptive physics buffer.  Instead of always running on the
//      driver's 1024-sample buffer, the buffer is resized at trigger time
//      to physN ≈ Fs / f, snapped down to a multiple of 16 and clamped to
//      [96, 512] (RS_M_MIN, RS_M_STEP, RS_N from inc/halo.h).  The
//      input wavetable is resampled into that smaller buffer with the same
//      linear interpolation as fill_wt_from_seeds().
//
//        - At low pitches the buffer caps at 512 → audio reads at < 1
//          sample / output (oversampled), like the original 1024-buffer JS.
//        - At medium pitches (~Fs / f ≤ 512) the buffer matches the audible
//          period, so audio reads at ~1.0 sample / output (no
//          downsampling, no read-side anti-alias).
//        - At very high pitches the buffer floors at 96 → audio reads at
//          > 1, just like the JS variant always does.
//
//   2. Stabilizers added in the C port (gated behind `stabilize: true`):
//        - dtt clamp to [0.005, 0.05]
//        - per-cycle DC removal on q[]
//        - per-cycle safety clamp |q[n]| ≤ 2
//
//   3. Round-robin physics throttle.  In the firmware advance_cycle is
//      driven by OSC_TIM at 1.8 kHz with at most one channel processed
//      per tick — so physics_rate = min(audible_freq, 1800 / num_active).
//      Exposed here as a `roundRobinHz` slider (default 1800 = single-voice
//      firmware load; dial down to 300 to model a 6-voice chord, or to 0
//      to disable and recover the original JS behaviour).  Below the cap
//      physics runs every audible cycle; above it advanceCycle() is
//      skipped on enough cycles to hold the long-term physics rate at
//      roundRobinHz, and the audio simply replays the previous buffer
//      cycle in the meantime — exactly the behaviour the SAI ISR ends up
//      with on the module when a wrap fires before OSC_TIM has reached
//      this channel.
//
//   4. Output soft-clip / compressor.  Mirrors src/compressor.c which is
//      configured by init_compressor(COMPRESS_SIGNED_24BIT, 0.90) and
//      applied per sample after master_gain in oscillator.c's audio ISR.
//      In unit space the curve is y = 1 − 0.09/|x| above |x| = 0.9 and
//      bypass below.  Exposed as a `compressorDrive` slider (default 0 =
//      off, matches the original JS).  Set to ~1 to engage on peaks
//      already past 0.9 from the synth, ~2 to simulate the firmware's
//      master-gain pushing a few-voice mix to ~2× threshold, and higher
//      values to simulate fuller chord loads.  The saturation is mostly
//      odd-harmonic and transient-shaving — the audible signature is a
//      smoother attack and a more "sustained / organ" character on
//      multi-voice material.
//
//  Things that stay the same (and therefore *aren't* sources of audible
//  drift): the dt = 0.02 cycle clock, the antipodal coupling (q[nR]−q[n])
//  with nR = (n+N/2)%N, the 0.1·q friction term in the velocity update,
//  the |v|>0.5 → ×0.99 soft compressor, and the cascaded one-pole circular
//  LPF (cutoff/2 on v, cutoff on q, mix = damping).
//
//  Things still divergent that we do NOT emulate here, but worth knowing:
//
//   - In the firmware the noise envelope is driven by the LPG/LFO output
//     (rs->externalEnvLevel), not the JS-internal ADSR.  This testbed keeps
//     the JS ADSR so single-note auditioning works in the browser.  If you
//     want flat envelopes for testing, set Noise Decay/Release very large
//     and Sustain = 1.
//
//   - The firmware also live-refreshes _wtOriginal in a round-robin in
//     OSC_TIM so per-cycle injection follows the browse encoder during a
//     held note.  The JS testbed re-seeds only at note start.
//
//   - The firmware closes-form αⁿ via expf(−2π·cutoff) and clamps cutoff
//     to N/2 − 1 to keep the LPF stable at small physN.  The JS LPF uses
//     an iterative αⁿ and never clamps; with `physN: 96` a high lpfCutoff
//     can therefore go past Nyquist of the buffer.  We keep that visible
//     so the difference is observable.
// ──────────────────────────────────────────────────────────────────────────
/** Compute the SWN-style physics buffer length for a given pitch.
 *  Mirrors the choice made in oscillator.c when a note triggers.
 *  Always returns an integer in [96, 512].  Constants are inlined
 *  to avoid any module-scope binding issues in AudioWorklet contexts. */
function physNForPitchSWN(freq, sampleRate) {
    // Hard-coded mirror of inc/halo.h: RS_N=512, RS_M_MIN=96, RS_M_STEP=16
    const RS_N = 512, RS_M_MIN = 96, RS_M_STEP = 16;
    let f = +freq, s = +sampleRate;
    if (!isFinite(f) || !isFinite(s) || f <= 0 || s <= 0) return RS_N;
    let m = Math.round(s / f);
    if (!isFinite(m)) return RS_N;
    m = (m / RS_M_STEP) | 0;        // truncating integer division
    m = m * RS_M_STEP;
    if (m < RS_M_MIN) m = RS_M_MIN;
    if (m > RS_N)     m = RS_N;
    // Final paranoid clamp via min/max (defends against any unexpected
    // earlier branch bypass) and force to integer.
    m = m | 0;
    if (m < RS_M_MIN) m = RS_M_MIN;
    if (m > RS_N)     m = RS_N;
    return m;
}

/** Resample a Float32Array to a new length using linear interpolation,
 *  matching halo.c::fill_wt_from_seeds()'s wrap-on-i1 behaviour.
 *  Returns null on allocation failure so the caller can fall back to the
 *  original (non-resampled) buffer instead of crashing. */
function _resampleLinear(src, dstLen) {
    const N = (dstLen | 0);
    const safeN = N < 1 ? 1 : (N > 8192 ? 8192 : N);
    const srcN = (src && src.length) | 0;
    let dst;
    try {
        dst = new Float32Array(safeN);
    } catch (e) {
        // eslint-disable-next-line no-console
        console.error('[Lagrangian-SWN] resample alloc failed', { safeN, srcN, error: String(e) });
        return null;
    }
    if (srcN === 0) return dst;
    const ratio = srcN / safeN;
    for (let n = 0; n < safeN; n++) {
        const pos = n * ratio;
        const i0 = Math.floor(pos);
        const i1 = (i0 + 1) % srcN;
        const t = pos - i0;
        dst[n] = src[i0] * (1 - t) + src[i1] * t;
    }
    return dst;
}

class LagrangianAcousticsSWN extends LagrangianAcoustics {
    static group = 'Lagrangian (SWN-Match)';
    // Same UI knobs as the parent class plus a round-robin throttle slider.
    //
    // 0    = throttle off (= original JS behaviour, physics every cycle).
    // 1800 = OSC_TIM rate (single voice held in firmware) — default.
    // 300  = OSC_TIM / 6 (six voices held — full chord-mode load).
    //
    // The firmware runs advance_cycle in OSC_TIM at 1800 Hz, processing at
    // most one channel per tick.  When you turn this knob down, you are
    // simulating "more voices contending for the same fixed budget" — which
    // is the firmware's main pitch-dependent timbre delta vs the unthrottled
    // JS implementation, especially noticeable above 600 Hz or so.
    static baseParams = [
        ...LagrangianAcoustics.baseParams,
        {
            key: 'roundRobinHz', label: 'Round-robin Hz',
            min: 0, max: 4000, default: 1800, step: 50, isInt: true,
            format: v => Number(v) === 0 ? 'off' : Number(v).toFixed(0) + ' Hz',
        },
        // Soft-clip / compressor drive — mirrors the SWN's
        // init_compressor(COMPRESS_SIGNED_24BIT, 0.90).  0 = bypass; 1 =
        // unit-gain into the threshold (compressor only engages on
        // peaks that already exceed 0.9 from the synth); 2 = simulates
        // ~2-3 voices' worth of mixed signal pushed by the firmware's
        // master_gain so peaks hit roughly 2× threshold; higher values
        // simulate heavier mixed-load saturation.
        {
            key: 'compressorDrive', label: 'Compressor Drive',
            min: 0, max: 6, default: 0, step: 0.05,
            format: v => Number(v) === 0 ? 'off' : Number(v).toFixed(2) + '×',
        },
    ];

    static createFromOpts(buffer, opts, { isDoubleEnded, sampleRate, frequency }) {
        // Compute the SWN-style buffer size and resample.  In the
        // AudioWorkletGlobalScope we've observed `new Float32Array(N)` for
        // small N (~336) intermittently failing with "Array buffer
        // allocation failed" even when a 1024-sample alloc just succeeded
        // — appears to be a Chrome AudioWorklet allocator quirk.  If the
        // resample alloc fails we silently fall back to the original
        // 1024-sample buffer (so the synth still produces sound and we
        // can at least A/B the SWN stabilizers); see
        // _resampleLinear()'s null return path.
        const physN = physNForPitchSWN(frequency, sampleRate);
        const srcN = (buffer && buffer.length) | 0;
        let resampled = buffer;
        if (srcN > 0 && physN > 0 && physN !== srcN && physN <= 8192) {
            const r = _resampleLinear(buffer, physN);
            if (r) resampled = r;
        }
        return new LagrangianAcousticsSWN(resampled, {
            lpfCutoff: Math.max(1, Math.round(opts.lpfCutoff ?? 8)),
            damping: clamp(opts.damping ?? 0.5, 0, 1),
            nonlinearity: clamp(opts.nonlinearity, 0, 1),
            noiseLevel: clamp(opts.noiseLevel ?? 0.02, 0, 1),
            noiseAttack: Math.max(0, Math.round(opts.noiseAttack ?? 5)),
            noiseDecay: Math.max(0, Math.round(opts.noiseDecay ?? 50)),
            noiseSustain: clamp(opts.noiseSustain ?? 0.3, 0, 1),
            noiseRelease: Math.max(0, Math.round(opts.noiseRelease ?? 100)),
            noiseColor: clamp(opts.noiseColor ?? 0, 0, 1),
            wtAttack: clamp(opts.wtAttack ?? 0, 0, 1),
            stabilize: true,
            roundRobinHz: Math.max(0, Math.round(opts.roundRobinHz ?? 1800)),
            compressorDrive: clamp(+opts.compressorDrive || 0, 0, 6),
            frequency,
        });
    }

    constructor(pressure, options) {
        super(pressure, options);
        // Soft-clip drive: read by SynthEngine in driver.js to enable the
        // per-sample compressor curve (= firmware compress()).  0 = off.
        this.softClipDrive = +options.compressorDrive || 0;
        // Round-robin physics throttle.
        //
        // Firmware behaviour: OSC_TIM ticks at OSC_TIM_HZ (= 1800).  Each
        // tick processes at most one channel's advance_cycle (only if its
        // cycleRequest flag is set by the audio ISR on a buffer wrap).
        // Multiple wraps between ticks coalesce into a single flag, so
        // the long-term physics rate per channel is
        //
        //     min(audible_freq, OSC_TIM_HZ / num_active_channels)
        //
        // We model that with a fractional per-cycle counter:
        //
        //     threshold = max(1, audible_freq / roundRobinHz)
        //     each audible cycle:  counter += 1
        //         if counter >= threshold: counter -= threshold; run physics
        //         else:                    skip physics, replay last cycle
        //
        // Below the cap (audible_freq ≤ rrHz), threshold clamps to 1 so we
        // run physics every cycle — identical to the unthrottled class.
        // Above the cap, on average we run physics rrHz times/sec
        // regardless of pitch, exactly matching the firmware's OSC_TIM
        // ceiling.  The audible signal in skipped cycles is the previous
        // buffer cycle replayed verbatim — which is what the SAI ISR ends
        // up doing on the module when cycleRequest is set but OSC_TIM
        // hasn't reached this channel yet.
        const rrHz = +options.roundRobinHz || 0;
        const freq = +options.frequency || 0;
        if (rrHz > 0 && freq > 0) {
            const ratio = freq / rrHz;
            this._rrThreshold = ratio > 1 ? ratio : 1;
        } else {
            this._rrThreshold = 1; // no throttle
        }
        this._rrCounter = 0;
    }

    advanceCycle() {
        this._rrCounter += 1;
        if (this._rrCounter < this._rrThreshold) {
            // Physics throttled this cycle: leave q[] / v[] untouched so
            // the audio read head replays the previous buffer cycle.
            // Still tick the noise ADSR — the firmware's LPG/LFO envelope
            // is real-time and independent of the physics rate, so its
            // shape shouldn't change with pitch.  This keeps the JS
            // testbed's perceived envelope timing identical whether the
            // throttle is active or not.
            this._advanceEnvelope();
            return;
        }
        this._rrCounter -= this._rrThreshold;
        super.advanceCycle();
    }
}


// --- Manifold Bloom (reservoir-coupled morphing string) -----------------
//  Symplectic reservoir coupling with controllable harmonic evolution.
//  Forces on the main circular chain:
//    1. Linear spring (Laplacian) — wave propagation
//    2. α·grad²·sign(grad) — nonlinear harmonic generator
//    3. −β·Δ²q — curvature cost (biharmonic) biases toward fundamental
//    4. κ·tanh(q²)·(w−q) — breathing reservoir bridge
//
//  Non-destructive output LP: bidirectional IIR on the output buffer
//    (controlled by damping slider) leaves the physics state untouched.
//
//  Kick-drift-kick Störmer-Verlet.  Circular topology only.
class ManifoldBloom {
    constructor(buffer, {
        alpha, asymmetry, costWeight, coupling, resTension, resDamping,
        damping, sustain, stepsPerCycle, morphRate, step
    }) {
        this._q = new Float32Array(buffer.length);  // physics state
        this._q.set(buffer);
        this.buffer = buffer;               // output (driver reads this)
        this.N = buffer.length;
        this.v = new Float32Array(this.N);  // Main velocity
        this.w = new Float32Array(this.N);  // Reservoir position
        this.z = new Float32Array(this.N);  // Reservoir velocity

        // Seed reservoir as 90°-shifted, half-amplitude copy of the
        // initial waveform.  This gives the reservoir immediate energy
        // content so that:
        //   - resTension meaningfully shapes its internal mode structure
        //   - the bridge coupling has real amplitude to exchange
        //   - we don't need strong coupling just to "fill" the reservoir
        // The 90° shift ensures the bridge force isn't zero at t=0
        // (q and w are out of phase, so w[n]-q[n] ≠ 0).
        const phaseShift = Math.round(this.N / 4);
        for (let i = 0; i < this.N; i++)
            this.w[i] = this._q[(i + phaseShift) % this.N] * 0.5;

        this.alpha = alpha ?? 0.1;     // Odd harmonic generation (3rd,5th…)
        this.asymmetry = asymmetry ?? 0.1;     // Even harmonic generation (2nd,4th…)
        this.costWeight = costWeight ?? 0.02;    // Curvature cost (→ fundamental)
        this.coupling = coupling ?? 0.005;   // Reservoir bridge strength
        this.resTension = resTension ?? 0.1;     // Reservoir spatial diffusion
        this.resDamping = resDamping ?? 0.001;   // Reservoir-only loss
        this.damping = damping ?? 1.0;     // Output LP strength (0–5)
        this.sustain = sustain ?? 0.9999;  // Per-step amplitude decay (q)
        this.stepsPerCycle = stepsPerCycle || 4;
        this.morphRate = morphRate ?? 0.5;  // reservoir offset drift (samples per substep)
        this._morphPhase = 0;               // accumulated reservoir shift (float)
        this._morphShift = 0;               // current integer shift

        // Pitch compensation: higher pitch → more advanceCycle() calls
        // per second, so scale coupling and morphRate by 1/step to keep
        // the per-second effect constant.  Normalised so step=1 is unity.
        this._pitchScale = 1 / Math.max(step || 1, 0.01);

        this._tmp = new Float32Array(this.N);
        this._qSmooth = new Float32Array(this.N);  // scratch for curvature cost

        // Gradient gain: normalises the discrete gradient d = q[n+1]−q[n]
        // so that a unit-amplitude sine has G·d_max ≈ 1.  Without this,
        // d_max ≈ 2πA/N ≈ 0.006 for N=1024, A=1, and tanh never leaves
        // its linear regime.  G·d remaps gradients into tanh's nonlinear
        // sweet spot.  The 1/G factor in the force keeps the potential
        // well-defined:  V(d) = d²/2 − ln(cosh(Gd))/G²  (odd),
        //                V(d) = −(d − tanh(Gd)/G)       (even).
        this._G = this.N / (2 * Math.PI);
        this._invG = 1 / this._G;

        // Pre-smooth position buffer so noise doesn't inflate initial energy.
        // 8 passes of strong LP removes sample-rate noise while barely
        // touching the low harmonics that define the waveform shape.
        for (let pass = 0; pass < 8; pass++) this._smooth(this._q, 0.45);

        // Initialise velocity consistent with a rightward-travelling wave.
        // For a circular chain with spring constant 1, wave speed c ≈ 1,
        // so v[n] ≈ −dq/dx ≈ −(q[n+1]−q[n−1])/2.  This avoids the violent
        // transient from launching with v=0 against a shaped waveform.
        for (let n = 0; n < this.N; n++) {
            const nL = (n - 1 + this.N) % this.N;
            const nR = (n + 1) % this.N;
            this.v[n] = -0.5 * (this._q[nR] - this._q[nL]);
        }

        // Initialise reservoir velocity as a travelling wave too.
        // Wave speed in the reservoir is c_res = √(resTension), so
        // z[n] = −c_res · dw/dx.  This lets the reservoir propagate
        // immediately rather than shock-splitting into standing waves.
        {
            const cRes = Math.sqrt(this.resTension);
            for (let n = 0; n < this.N; n++) {
                const nL = (n - 1 + this.N) % this.N;
                const nR = (n + 1) % this.N;
                this.z[n] = -0.5 * cRes * (this.w[nR] - this.w[nL]);
            }
        }

    }

    // Force on main-wave node n.
    // Uses tanh-bounded hardening spring with gradient gain G = N/(2π).
    // Derives from potential V(d) = d²/2 + α·(d²/2 − ln(cosh(Gd))/G²),
    // so the Verlet integrator conserves total energy automatically.
    // Even harmonics are handled per-cycle (see advanceCycle), not here.
    _forceQ(n) {
        const { _q: q, w, N, alpha, coupling, _G: G, _invG: invG } = this;
        const nL = (n - 1 + N) % N;
        const nR = (n + 1) % N;

        const dR = q[nR] - q[n];
        const dL = q[n] - q[nL];

        // 1. Linear spring (Laplacian)
        const fLinear = dR - dL;

        // 2. Odd harmonics: hardening spring residual d − tanh(Gd)/G.
        //    At Gd ≈ 1 (unit-amplitude sine), the residual is ~24% of d.
        //    CFL-safe for α ≤ 5 with dt = 0.4.
        const fOdd = alpha * ((dR - Math.tanh(G * dR) * invG)
            - (dL - Math.tanh(G * dL) * invG));

        // 3. Reservoir bridge — strength proportional to local |q| amplitude:
        //    nodes near zero cross weakly; crests/troughs couple strongly.
        //    The w-side force (see _forceW) uses the same |q[n]| factor so
        //    the action-reaction pair stays symmetric (Newton's 3rd law).
        const shift = this._morphShift;
        const wIdx = (n + shift) % N;
        const fBridge = (coupling / N) * this._pitchScale * Math.abs(q[n]) * (w[wIdx] - q[n]);

        return fLinear + fOdd + fBridge;
    }

    // Force on reservoir node n
    _forceW(n) {
        const { _q: q, w, N, coupling, resTension, asymmetry, _G: G } = this;
        const nL = (n - 1 + N) % N;
        const nR = (n + 1) % N;
        const dR = w[nR] - w[n];
        const dL = w[n] - w[nL];

        // Linear Laplacian
        let fInternal = resTension * (dR - dL);

        // Asymmetric (quadratic) spring → even harmonics grow in the reservoir
        // and couple gradually into q via the bridge.
        // G-scaling (= N/2π) keeps the term comparable to the linear spring
        // at unit amplitude, matching the convention used for alpha/cubic.
        if (asymmetry > 0) {
            // Saturated asymmetric spring: matches G·d² at small amplitude
            // but is bounded above so the cubic potential never goes runaway
            // (unsaturated cubic → Burgers self-steepening → NaN in <1 cycle).
            const sR = dR / (1 + G * Math.abs(dR));
            const sL = dL / (1 + G * Math.abs(dL));
            fInternal += asymmetry * G * (sR * sR - sL * sL);
        }

        // Symmetric reverse bridge — w[n] is coupled to q[n - shift].
        // Uses |q[qIdx]| (the amplitude of the paired q node) so this is the
        // exact reaction force to the q-side bridge above.
        const shift = this._morphShift;
        const qIdx = (n - shift + N) % N;
        const fBridge = -(coupling / N) * this._pitchScale * Math.abs(q[qIdx]) * (w[n] - q[qIdx]);

        return fInternal + fBridge;
    }

    // 3-point LP smoothing (unconditionally stable for c ≤ 0.5)
    _smooth(arr, c) {
        if (c <= 0) return;
        const { N, _tmp: tmp } = this;
        const wt = 1 - 2 * c;
        for (let n = 0; n < N; n++) {
            tmp[n] = wt * arr[n] + c * (arr[(n - 1 + N) % N] + arr[(n + 1) % N]);
        }
        arr.set(tmp);
    }

    // Bidirectional single-pole IIR lowpass on a circular buffer.
    // Two laps forward then two laps backward; only the second lap
    // of each direction writes to the array.  This gives zero-phase
    // smoothing equivalent to many three-point LP passes but at 4N
    // cost instead of 20N.  a (0–1) controls cutoff: higher = smoother.
    _biDirSmooth(arr, a) {
        const N = this.N;
        const b = 1 - a;
        // Forward: warmup lap then write lap
        let s = arr[N - 1];
        for (let n = 0; n < N; n++) s = b * arr[n] + a * s;
        for (let n = 0; n < N; n++) { s = b * arr[n] + a * s; arr[n] = s; }
        // Backward: warmup lap then write lap
        s = arr[0];
        for (let n = N - 1; n >= 0; n--) s = b * arr[n] + a * s;
        for (let n = N - 1; n >= 0; n--) { s = b * arr[n] + a * s; arr[n] = s; }
    }

    advanceCycle() {
        const { _q: q, v, w, z, N, sustain,
            resDamping, stepsPerCycle } = this;
        const dt = 0.4;
        const resSustain = 1.0 - resDamping;

        // Self-driven morph: reservoir angular momentum Ω drives
        // the coupling offset.  Ω = Σ w[n]·z[n+1] − w[n+1]·z[n]
        // is the discrete analogue of L_z for the reservoir ring.
        // When the reservoir has net rotational motion the coupling
        // map advances, which feeds back into the reservoir dynamics
        // and creates organic, data-driven harmonic morphing.
        {
            let omega = 0;
            for (let n = 0; n < N; n++) {
                const nR = (n + 1) % N;
                omega += w[n] * z[nR] - w[nR] * z[n];
            }
            this._morphPhase += this.morphRate * this._pitchScale * omega;
            // Map continuous phase to integer shift in [0, N)
            this._morphShift = ((Math.round(this._morphPhase) % N) + N) % N;
        }

        for (let s = 0; s < stepsPerCycle; s++) {
            // --- KICK (half-step): update momenta ---
            for (let n = 0; n < N; n++) {
                v[n] += 0.5 * dt * this._forceQ(n);
                z[n] += 0.5 * dt * this._forceW(n);
            }

            // --- DRIFT (full-step): update positions ---
            for (let n = 0; n < N; n++) {
                q[n] += dt * v[n];
                w[n] += dt * z[n];
            }

            // --- KICK (half-step): update momenta from new positions ---
            for (let n = 0; n < N; n++) {
                v[n] += 0.5 * dt * this._forceQ(n);
                z[n] += 0.5 * dt * this._forceW(n);
            }

            // Reservoir-only damping
            if (resDamping > 0) {
                for (let n = 0; n < N; n++) z[n] *= resSustain;
            }

            // Remove q DC offset (bridge coupling provides natural restoring
            // force on w mean, so w does not need separate DC stripping).
            let dc = 0;
            for (let n = 0; n < N; n++) dc += q[n];
            dc /= N;
            for (let n = 0; n < N; n++) q[n] -= dc;

            // Amplitude decay — scales positions so the waveform fades.
            if (sustain < 1.0) {
                for (let n = 0; n < N; n++) q[n] *= sustain;
            }
        }


        // Curvature cost: blend q toward a heavily LP-smoothed version
        // of itself.  20 LP passes retain the first ~30 harmonics and
        // kill everything above that.  costWeight controls how strongly
        // each cycle pulls toward the smooth shape:
        //   0   = no effect
        //   0.5 = 50% blend toward fundamental per cycle
        // This is CFL-safe (applied outside Verlet) and gives a clear
        // “fundamental purity” control distinct from damping/drag.
        if (this.costWeight > 0) {
            const qSm = this._qSmooth;
            qSm.set(q);
            this._biDirSmooth(qSm, 0.75);
            const cw = this.costWeight;
            for (let n = 0; n < N; n++) {
                q[n] += cw * (qSm[n] - q[n]);
            }
        }

        // Non-destructive output LP: copy physics state → output buffer,
        // then apply bidirectional IIR.  The physics state (_q) is untouched.
        // Asymmetry drives even harmonics in the reservoir (see _forceW), so
        // no output waveshaping is needed here.
        this.buffer.set(q);

        if (this.damping > 0) {
            this._biDirSmooth(this.buffer, this.damping / (this.damping + 1));
        }
    }

    // Total mechanical energy (kinetic + potential, both layers).
    // Excludes bridge coupling PE (rotating map would create phantom deltas).
    _totalEnergy() {
        const { _q: q, v, w, z, N, resTension } = this;
        let ke = 0, pe = 0;
        for (let n = 0; n < N; n++) {
            ke += v[n] * v[n] + z[n] * z[n];
            const nR = n < N - 1 ? n + 1 : 0;
            const dq = q[nR] - q[n];
            pe += dq * dq;
            const dw = w[nR] - w[n];
            pe += resTension * dw * dw;
        }
        return 0.5 * (ke + pe);
    }

    snapshot() { return new Float32Array(this.buffer); }
}

// ========================================================================
//  ManifoldBloomFast — position-Verlet cached-force variant
//
//  Optimisations over ManifoldBloom:
//    • stepsPerCycle is always 1 (4× less force work)
//    • Padé [3,2] rational approximation replaces Math.tanh
//    • Position Verlet with cached accelerations: only 1 force evaluation
//      per cycle instead of 2 (kick-drift-kick)
//    • Fast biDirSmooth for output damping (no warmup laps, 2N vs 4N)
//    • Fused loops: buffer copy+asymmetry, force+velocity+reservoir damp
//    • Pre-computed constants, branchless circular indexing
//
//  Memory: q, v, w, z, aq, aw, tmp, qSmooth, buffer = 9×N arrays.
// ========================================================================
class ManifoldBloomFast {
    constructor(buffer, {
        alpha, asymmetry, costWeight, coupling, resTension, resDamping,
        damping, sustain, stepsPerCycle, morphRate, step
    }) {
        this._q = new Float32Array(buffer.length);
        this._q.set(buffer);
        this.buffer = buffer;
        this.N = buffer.length;
        this.v = new Float32Array(this.N);
        this.w = new Float32Array(this.N);
        this.z = new Float32Array(this.N);

        // Position Verlet: cached accelerations from previous cycle
        this._aq = new Float32Array(this.N);
        this._aw = new Float32Array(this.N);

        // Seed reservoir as 90°-shifted, half-amplitude copy
        const phaseShift = Math.round(this.N / 4);
        for (let i = 0; i < this.N; i++)
            this.w[i] = this._q[(i + phaseShift) % this.N] * 0.5;

        this.alpha = alpha ?? 0.1;
        this.asymmetry = asymmetry ?? 0.1;
        this.costWeight = costWeight ?? 0.02;
        this.coupling = coupling ?? 0.005;
        this.resTension = resTension ?? 0.1;
        this.resDamping = resDamping ?? 0.001;
        this.damping = damping ?? 1.0;
        this.sustain = sustain ?? 0.9999;
        this.morphRate = morphRate ?? 0.5;
        this._morphPhase = 0;
        this._morphShift = 0;

        this._pitchScale = 1 / Math.max(step || 1, 0.01);

        this._tmp = new Float32Array(this.N);
        this._qSmooth = new Float32Array(this.N);

        this._G = this.N / (2 * Math.PI);
        this._invG = 1 / this._G;
        const G2 = this._G * this._G;
        this._G2x5 = 5 * G2;
        this._G2x6 = 6 * G2;

        this._coupScaled = this.coupling * this._pitchScale / this.N;
        this._morphScaled = this.morphRate * this._pitchScale;
        this._resSustain = 1.0 - this.resDamping;

        // Pre-smooth position buffer
        for (let pass = 0; pass < 8; pass++) this._smooth(this._q, 0.45);

        // Init velocity (travelling wave)
        for (let n = 0; n < this.N; n++) {
            const nL = (n - 1 + this.N) % this.N;
            const nR = (n + 1) % this.N;
            this.v[n] = -0.5 * (this._q[nR] - this._q[nL]);
        }

        // Init reservoir velocity
        {
            const cRes = Math.sqrt(this.resTension);
            for (let n = 0; n < this.N; n++) {
                const nL = (n - 1 + this.N) % this.N;
                const nR = (n + 1) % this.N;
                this.z[n] = -0.5 * cRes * (this.w[nR] - this.w[nL]);
            }
        }

        // Compute initial forces for position Verlet cache
        this._computeInitialForces();
    }

    // Compute forces on the current state into the _aq/_aw cache.
    // Called once in constructor; afterwards advanceCycle updates inline.
    _computeInitialForces() {
        const { _q: q, w, N, _aq: aq, _aw: aw } = this;
        const Nm1 = N - 1;
        const alpha = this.alpha;
        const asymmetry = this.asymmetry;
        const G = this._G;
        const G2x5 = this._G2x5;
        const G2x6 = this._G2x6;
        const coup = this._coupScaled;
        const resTen = this.resTension;
        const shift = this._morphShift;

        for (let n = 0; n < N; n++) {
            const nL = n > 0 ? n - 1 : Nm1;
            const nR = n < Nm1 ? n + 1 : 0;
            const qN = q[n], qR = q[nR], qL = q[nL];
            const dR = qR - qN, dL = qN - qL;
            let fq = dR - dL;
            if (alpha !== 0) {
                const dR2 = dR * dR, dL2 = dL * dL;
                fq += alpha * (
                    G2x5 * dR2 * dR / (15 + G2x6 * dR2)
                    - G2x5 * dL2 * dL / (15 + G2x6 * dL2)
                );
            }
            let wIdx = n + shift; if (wIdx >= N) wIdx -= N;
            fq += coup * Math.abs(qN) * (w[wIdx] - qN);
            aq[n] = fq;

            const wN = w[n], wR = w[nR], wL = w[nL];
            const dRw = wR - wN, dLw = wN - wL;
            let fw = resTen * (dRw - dLw);
            if (asymmetry !== 0) {
                const sRw = dRw / (1 + G * Math.abs(dRw));
                const sLw = dLw / (1 + G * Math.abs(dLw));
                fw += asymmetry * G * (sRw * sRw - sLw * sLw);
            }
            let qIdx = n - shift; if (qIdx < 0) qIdx += N;
            fw -= coup * Math.abs(q[qIdx]) * (wN - q[qIdx]);
            aw[n] = fw;
        }
    }

    _smooth(arr, c) {
        if (c <= 0) return;
        const { N, _tmp: tmp } = this;
        const wt = 1 - 2 * c;
        for (let n = 0; n < N; n++) {
            tmp[n] = wt * arr[n] + c * (arr[(n - 1 + N) % N] + arr[(n + 1) % N]);
        }
        arr.set(tmp);
    }

    // Full biDirSmooth with warmup (for curvature cost — affects physics)
    _biDirSmooth(arr, a) {
        const N = this.N;
        const b = 1 - a;
        let s = arr[N - 1];
        for (let n = 0; n < N; n++) s = b * arr[n] + a * s;
        for (let n = 0; n < N; n++) { s = b * arr[n] + a * s; arr[n] = s; }
        s = arr[0];
        for (let n = N - 1; n >= 0; n--) s = b * arr[n] + a * s;
        for (let n = N - 1; n >= 0; n--) { s = b * arr[n] + a * s; arr[n] = s; }
    }

    // Fast biDirSmooth: no warmup laps (2N instead of 4N).
    // For output damping only (cosmetic — doesn't feed back into physics).
    _biDirSmoothFast(arr, a) {
        const N = this.N;
        const b = 1 - a;
        let s = arr[N - 1];
        for (let n = 0; n < N; n++) { s = b * arr[n] + a * s; arr[n] = s; }
        s = arr[0];
        for (let n = N - 1; n >= 0; n--) { s = b * arr[n] + a * s; arr[n] = s; }
    }

    _totalEnergy() {
        const { _q: q, v, w, z, N, resTension } = this;
        let ke = 0, pe = 0;
        for (let n = 0; n < N; n++) {
            ke += v[n] * v[n] + z[n] * z[n];
            const nR = n < N - 1 ? n + 1 : 0;
            const dq = q[nR] - q[n];
            pe += dq * dq;
            const dw = w[nR] - w[n];
            pe += resTension * dw * dw;
        }
        return 0.5 * (ke + pe);
    }

    advanceCycle() {
        const { _q: q, v, w, z, N, _aq: aq, _aw: aw } = this;
        const Nm1 = N - 1;
        const dt = 0.4;
        const halfDt = 0.2;
        const halfDt2 = 0.08;   // 0.5 * dt * dt

        const alpha = this.alpha;
        const asymmetry = this.asymmetry;
        const G = this._G;
        const G2x5 = this._G2x5;
        const G2x6 = this._G2x6;
        const coup = this._coupScaled;
        const resTen = this.resTension;
        const sustain = this.sustain;
        const resSustain = this._resSustain;

        // ── 1. Morph phase update ──
        let omega = 0;
        for (let n = 0; n < Nm1; n++) {
            omega += w[n] * z[n + 1] - w[n + 1] * z[n];
        }
        omega += w[Nm1] * z[0] - w[0] * z[Nm1];
        this._morphPhase += this._morphScaled * omega;
        this._morphShift = ((Math.round(this._morphPhase) % N) + N) % N;
        const shift = this._morphShift;

        // ── 2. Position Verlet drift ──
        // q += dt·v + ½dt²·a_cached,  w += dt·z + ½dt²·aw_cached
        // Fused with DC accumulation.
        let dc = 0;
        for (let n = 0; n < N; n++) {
            q[n] += dt * v[n] + halfDt2 * aq[n];
            w[n] += dt * z[n] + halfDt2 * aw[n];
            dc += q[n];
        }

        // ── 3. Fused DC removal + sustain ──
        dc /= N;
        for (let n = 0; n < N; n++) {
            q[n] = (q[n] - dc) * sustain;
        }
        // ── 4. Curvature cost (full warmup biDirSmooth — affects physics) ──
        if (this.costWeight > 0) {
            const qSm = this._qSmooth;
            qSm.set(q);
            this._biDirSmooth(qSm, 0.75);
            const cw = this.costWeight;
            for (let n = 0; n < N; n++) {
                q[n] += cw * (qSm[n] - q[n]);
            }
        }

        // ── 5. Fused: force computation + velocity update + z damping ──
        // Position Verlet: v += ½dt·(a_old + a_new)
        // Reads old cached force, computes new force on processed q/w,
        // updates velocity, damps reservoir velocity, caches new force.
        // Single pass — eliminates the entire second force evaluation.
        for (let n = 0; n < N; n++) {
            const nL = n > 0 ? n - 1 : Nm1;
            const nR = n < Nm1 ? n + 1 : 0;

            // Read old cached forces before overwriting
            const aq_old = aq[n];
            const aw_old = aw[n];

            // ── Main-wave force (Padé [3,2] hardening spring) ──
            const qN = q[n], qR = q[nR], qL = q[nL];
            const dR = qR - qN;
            const dL = qN - qL;
            let fq = dR - dL;
            if (alpha !== 0) {
                const dR2 = dR * dR, dL2 = dL * dL;
                fq += alpha * (
                    G2x5 * dR2 * dR / (15 + G2x6 * dR2)
                    - G2x5 * dL2 * dL / (15 + G2x6 * dL2)
                );
            }
            let wIdx = n + shift; if (wIdx >= N) wIdx -= N;
            fq += coup * Math.abs(qN) * (w[wIdx] - qN);

            // ── Reservoir force ──
            // Asymmetric (quadratic) spring generates even harmonics in w,
            // which couple gradually into q via the bridge.
            const wN = w[n], wR = w[nR], wL = w[nL];
            const dRw = wR - wN, dLw = wN - wL;
            let fw = resTen * (dRw - dLw);
            if (asymmetry !== 0) {
                const sRw = dRw / (1 + G * Math.abs(dRw));
                const sLw = dLw / (1 + G * Math.abs(dLw));
                fw += asymmetry * G * (sRw * sRw - sLw * sLw);
            }
            let qIdx = n - shift; if (qIdx < 0) qIdx += N;
            fw -= coup * Math.abs(q[qIdx]) * (wN - q[qIdx]);

            // Velocity update (trapezoid rule) + reservoir damping
            v[n] += halfDt * (aq_old + fq);
            z[n] = (z[n] + halfDt * (aw_old + fw)) * resSustain;

            // Cache new forces for next cycle
            aq[n] = fq;
            aw[n] = fw;
        }

        // ── 6. Output: copy q → buffer + output LP ──
        // Asymmetry drives even harmonics in the reservoir spring (step 5),
        // so no output waveshaping is needed here.
        const buf = this.buffer;
        buf.set(q);

        // ── 7. Output damping: fast biDirSmooth (no warmup) ──
        if (this.damping > 0) {
            this._biDirSmoothFast(buf, this.damping / (this.damping + 1));
        }
    }

    snapshot() { return new Float32Array(this.buffer); }
}
// ========================================================================
//  ManifoldBloomV2 — sinusoidal parametric harmonic coupling
//
//  Core idea: instead of a reservoir, modulate the string's effective
//  tension with a SINGLE cosine at a controlled spatial frequency m:
//
//    T(n) = 1 + ε · cos(2π·m·n / N)
//
//  By Fourier's product rule, multiplying mode k's restoring force by
//  cos(2πm·n/N) creates forces at exactly modes k±m:
//
//    cos(k·θ) × cos(m·θ) = ½[cos((k+m)θ) + cos((k−m)θ)]
//
//  Starting from a sine (mode 1):
//    m=1 → octave (mode 2) appears
//    m=2 → 3rd harmonic (mode 3)
//    m=3 → 4th harmonic (mode 4)
//    m=4 → 5th harmonic (mode 5)
//  Between integer m: spectral leakage spreads across 3–4 harmonics.
//  As m drifts, the harmonic spotlight smoothly sweeps.
//
//  The cascading effect: once mode 3 exists, it ALSO couples with the
//  tension modulation (mode 3 × cos(m) → modes 3±m), building a web
//  of selective mode interactions.
//
//  Drift mechanism: m follows a quasiperiodic oscillator (two
//  incommensurate frequencies, golden ratio) → never repeats, always
//  bounded, deterministic, organic.
//
//  Physical analogy: fingers pressing on the string at a specific
//  spacing pattern.  The spacing (m) determines which harmonics feel
//  the pressure.  The pattern slowly, nonperiodically shifts.
//
//  Advantages over reservoir approach:
//    • Selective (only modes k±m, not all sidebands)
//    • Inherently stable (conservative, tension always > 0 for ε<1)
//    • Cheaper (no reservoir arrays, inline cos via Goertzel recurrence)
//    • Simpler (6×N arrays vs 7–9×N)
//    • Creates harmonics from sine input (parametric excitation)
//
//  Slider mapping:
//    coupling   → ε (modulation depth, 0–0.5)
//    resTension → harmonic range (which harmonics the spotlight reaches)
//    morphRate  → drift speed of the spotlight
//
//  Memory: q, v, aq, tmp, qSmooth, buffer = 6×N
// ========================================================================
class ManifoldBloomV2 {
    constructor(buffer, {
        alpha, asymmetry, costWeight, coupling, resTension, resDamping,
        damping, sustain, stepsPerCycle, morphRate, selectivity, step
    }) {
        this._q = new Float32Array(buffer.length);
        this._q.set(buffer);
        this.buffer = buffer;
        this.N = buffer.length;
        this.v = new Float32Array(this.N);

        // Position Verlet: cached acceleration
        this._aq = new Float32Array(this.N);

        this.alpha = alpha ?? 0.1;
        this.asymmetry = asymmetry ?? 0.1;
        this.costWeight = costWeight ?? 0.02;
        this.damping = damping ?? 1.0;
        this.sustain = sustain ?? 0.9999;

        this._pitchScale = 1 / Math.max(step || 1, 0.01);

        this._tmp = new Float32Array(this.N);
        this._qSmooth = new Float32Array(this.N);

        // Gradient gain G = N/(2π) and Padé [3,2] constants
        this._G = this.N / (2 * Math.PI);
        this._invG = 1 / this._G;
        const G2 = this._G * this._G;
        this._G2x5 = 5 * G2;
        this._G2x6 = 6 * G2;

        // ── Parametric modulation ──
        // coupling → modulation depth ε (0–0.5).
        // At default 0.746 → ε ≈ 0.37 → strong harmonic coupling.
        // Tension range [1−ε, 1+ε] stays positive for ε < 1.
        this._modEpsilon = clamp((coupling ?? 0.005) * 0.5, 0, 0.5);

        // resTension → harmonic range of the drifting spotlight.
        //   resTension=0.1 → m drifts ~1–3 (octave to 4th harmonic)
        //   resTension=1   → m drifts ~1–5 (up to 6th)
        //   resTension=5   → m drifts ~1–13 (wide range)
        const rt = resTension ?? 0.1;
        this._modCenter = 2 + rt;
        this._modRange = 1 + rt;

        // ── Quasiperiodic drift ──
        // Two incommensurate frequencies (golden ratio) → nonperiodic,
        // deterministic, always bounded, never exactly repeats.
        //   morphRate=0.5 → full drift cycle ~5 seconds
        //   morphRate=2   → full drift cycle ~1 second
        const mr = (morphRate ?? 0.5) * this._pitchScale;
        this._driftRate1 = mr * 0.015;
        this._driftRate2 = this._driftRate1 * 1.6180339887;  // golden ratio
        this._phase1 = 0;
        this._phase2 = 0;

        // ── Integer selectivity (sinusoidal pinch) ──
        // Warps the raw drift value so it dwells near integer m values.
        // selectivity k=0: no effect (linear pass-through).
        // selectivity k=0.9: strong stickiness at integers, fast transitions.
        // selectivity k>1: non-monotone — creates hysteresis / hard snaps.
        // Formula: m' = m_raw - (k / 2π) · sin(2π · m_raw)
        // Derivative at integers: g'(n) = 1 - k  → slow when k→1.
        // Derivative at half-integers: g'(n+½) = 1 + k → fast transitions.
        this._selectivity = clamp(selectivity ?? 0.7, 0, 1.5);

        // Initial modulation frequency
        this._modFreq = this._modCenter;

        // Pre-smooth position buffer
        for (let pass = 0; pass < 8; pass++) this._smooth(this._q, 0.45);

        // Init velocity (travelling wave)
        for (let n = 0; n < this.N; n++) {
            const nL = (n - 1 + this.N) % this.N;
            const nR = (n + 1) % this.N;
            this.v[n] = -0.5 * (this._q[nR] - this._q[nL]);
        }

        // Compute initial forces with parametric tension
        this._computeInitialForces();

        // Track initial energy for safety clamp
        this._E0 = this._totalEnergy();
    }

    _computeInitialForces() {
        const { _q: q, N, _aq: aq } = this;
        const Nm1 = N - 1;
        const alpha = this.alpha;
        const G2x5 = this._G2x5;
        const G2x6 = this._G2x6;
        const epsilon = this._modEpsilon;

        // Goertzel-style cosine recurrence: cos(2π·m·n/N)
        const modOmega = 2 * Math.PI * this._modFreq / N;
        const twoCosW = 2 * Math.cos(modOmega);
        let cosM = 1;                   // cos(0)
        let cosPrev = Math.cos(modOmega);  // cos(−ω) = cos(ω)

        for (let n = 0; n < N; n++) {
            const nL = n > 0 ? n - 1 : Nm1;
            const nR = n < Nm1 ? n + 1 : 0;
            const qN = q[n], qR = q[nR], qL = q[nL];
            const dR = qR - qN, dL = qN - qL;

            // Parametric tension from cosine modulation
            const tension = 1 + epsilon * cosM;
            let fq = tension * (dR - dL);

            // Padé [3,2] nonlinearity (odd harmonics)
            if (alpha !== 0) {
                const dR2 = dR * dR, dL2 = dL * dL;
                fq += alpha * (
                    G2x5 * dR2 * dR / (15 + G2x6 * dR2)
                    - G2x5 * dL2 * dL / (15 + G2x6 * dL2)
                );
            }
            aq[n] = fq;

            // Advance cosine recurrence
            const cosNext = twoCosW * cosM - cosPrev;
            cosPrev = cosM;
            cosM = cosNext;
        }
    }

    _smooth(arr, c) {
        if (c <= 0) return;
        const { N, _tmp: tmp } = this;
        const wt = 1 - 2 * c;
        for (let n = 0; n < N; n++) {
            tmp[n] = wt * arr[n] + c * (arr[(n - 1 + N) % N] + arr[(n + 1) % N]);
        }
        arr.set(tmp);
    }

    // Full biDirSmooth with warmup (for curvature cost)
    _biDirSmooth(arr, a) {
        const N = this.N;
        const b = 1 - a;
        let s = arr[N - 1];
        for (let n = 0; n < N; n++) s = b * arr[n] + a * s;
        for (let n = 0; n < N; n++) { s = b * arr[n] + a * s; arr[n] = s; }
        s = arr[0];
        for (let n = N - 1; n >= 0; n--) s = b * arr[n] + a * s;
        for (let n = N - 1; n >= 0; n--) { s = b * arr[n] + a * s; arr[n] = s; }
    }

    // Fast biDirSmooth (no warmup, no decay)
    _biDirSmoothFast(arr, a) {
        const N = this.N;
        const b = 1 - a;
        let s = arr[N - 1];
        for (let n = 0; n < N; n++) { s = b * arr[n] + a * s; arr[n] = s; }
        s = arr[0];
        for (let n = N - 1; n >= 0; n--) { s = b * arr[n] + a * s; arr[n] = s; }
    }

    _totalEnergy() {
        const { _q: q, v, N } = this;
        let ke = 0, pe = 0;
        for (let n = 0; n < N; n++) {
            ke += v[n] * v[n];
            const nR = n < N - 1 ? n + 1 : 0;
            const dq = q[nR] - q[n];
            pe += dq * dq;
        }
        return 0.5 * (ke + pe);
    }

    advanceCycle() {
        const { _q: q, v, N, _aq: aq } = this;
        const Nm1 = N - 1;
        const dt = 0.4;
        const halfDt = 0.2;
        const halfDt2 = 0.08;   // 0.5 * dt * dt

        const alpha = this.alpha;
        const G2x5 = this._G2x5;
        const G2x6 = this._G2x6;
        const epsilon = this._modEpsilon;
        const sustain = this.sustain;

        // ── 1. Update modulation frequency m (quasiperiodic drift + integer pinch) ──
        // Two incommensurate sinusoids → nonperiodic orbit.
        // m determines which harmonic family the spotlight hits.
        // The sinusoidal pinch warps m_raw so it dwells near integer values:
        //   m' = m_raw - (k/2π)·sin(2π·m_raw)
        // At integers: derivative = 1−k → slow (long dwell).
        // At half-integers: derivative = 1+k → fast (quick snap).
        this._phase1 += this._driftRate1;
        this._phase2 += this._driftRate2;
        const mRaw = this._modCenter
            + this._modRange * (0.7 * Math.sin(this._phase1)
                + 0.3 * Math.sin(this._phase2));
        const TWO_PI = 6.283185307179586;
        const mPinched = mRaw - (this._selectivity / TWO_PI) * Math.sin(TWO_PI * mRaw);
        this._modFreq = Math.max(0.5, mPinched);

        // ── 2. Position Verlet drift ──
        let dc = 0;
        for (let n = 0; n < N; n++) {
            q[n] += dt * v[n] + halfDt2 * aq[n];
            dc += q[n];
        }

        // ── 3. DC removal + sustain (both q AND v) ──
        // Applying sustain to both prevents KE/PE imbalance.
        dc /= N;
        for (let n = 0; n < N; n++) {
            q[n] = (q[n] - dc) * sustain;
            v[n] *= sustain;
        }

        // ── 4. Curvature cost (every cycle, matching Fast) ──
        // Blends q toward a smooth version to suppress the harshest
        // harmonics.  Runs every cycle at base weight (0.02 default)
        // for smooth, gradual damping.  No amortization — avoids the
        // large discrete PE drops that caused audible energy jumps.
        if (this.costWeight > 0) {
            const qSm = this._qSmooth;
            qSm.set(q);
            this._biDirSmooth(qSm, 0.75);
            const cw = this.costWeight;
            for (let n = 0; n < N; n++) {
                q[n] += cw * (qSm[n] - q[n]);
            }
        }

        // ── 5. Parametric force + velocity update ──
        // Goertzel cosine recurrence generates cos(2π·m·n/N) inline.
        // T(n) = 1 + ε·cos(...) modulates the Laplacian, creating
        // forces at modes k±m for each existing mode k.
        // Cost: 1 extra multiply + 2 recurrence ops per sample.
        const modOmega = 2 * Math.PI * this._modFreq / N;
        const twoCosW = 2 * Math.cos(modOmega);
        let cosM = 1;
        let cosPrev = Math.cos(modOmega);

        for (let n = 0; n < N; n++) {
            const nL = n > 0 ? n - 1 : Nm1;
            const nR = n < Nm1 ? n + 1 : 0;

            const aq_old = aq[n];

            const qN = q[n], qR = q[nR], qL = q[nL];
            const dR = qR - qN;
            const dL = qN - qL;

            // Parametric tension from drifting cosine
            const tension = 1 + epsilon * cosM;
            let fq = tension * (dR - dL);

            // Padé [3,2] nonlinearity (odd harmonics)
            if (alpha !== 0) {
                const dR2 = dR * dR, dL2 = dL * dL;
                fq += alpha * (
                    G2x5 * dR2 * dR / (15 + G2x6 * dR2)
                    - G2x5 * dL2 * dL / (15 + G2x6 * dL2)
                );
            }

            // Position Verlet velocity update (trapezoid rule)
            v[n] += halfDt * (aq_old + fq);
            aq[n] = fq;

            // Advance cosine recurrence
            const cosNext = twoCosW * cosM - cosPrev;
            cosPrev = cosM;
            cosM = cosNext;
        }

        // ── 6. Energy safety clamp ──
        // Time-varying m creates a small Verlet mismatch (cached force
        // was at old m, new force at new m).  Clamp to 2× initial energy.
        if (this._E0 > 1e-20) {
            const E = this._totalEnergy();
            if (E > 2 * this._E0) {
                const scale = Math.sqrt(this._E0 / E);
                for (let n = 0; n < N; n++) { q[n] *= scale; v[n] *= scale; }
            }
        }

        // ── 7. Output: fused copy + asymmetry ──
        const buf = this.buffer;
        if (this.asymmetry > 0) {
            const beta = this.asymmetry;
            let sumQ2 = 0;
            for (let n = 0; n < N; n++) {
                const val = q[n];
                buf[n] = val;
                sumQ2 += val * val;
            }
            const rmsBefore = Math.sqrt(sumQ2 / N);
            if (rmsBefore > 1e-12) {
                const dcQ2 = sumQ2 / N;
                let sumAfter = 0;
                for (let n = 0; n < N; n++) {
                    const bn = buf[n];
                    const shaped = bn + beta * (bn * bn - dcQ2);
                    buf[n] = shaped;
                    sumAfter += shaped * shaped;
                }
                const rmsAfter = Math.sqrt(sumAfter / N);
                if (rmsAfter > 1e-12) {
                    const scale = rmsBefore / rmsAfter;
                    for (let n = 0; n < N; n++) buf[n] *= scale;
                }
            }
        } else {
            buf.set(q);
        }

        // ── 8. Output damping: fast biDirSmooth (no warmup) ──
        if (this.damping > 0) {
            this._biDirSmoothFast(buf, this.damping / (this.damping + 1));
        }
    }

    snapshot() { return new Float32Array(this.buffer); }
}
// ========================================================================
//  ManifoldBloomV3 — Self-Driven Peak Cascade
//
//  The dominant mode's spatial frequency drives a parametric tension
//  modulation that selectively creates its first overtone.  As the
//  overtone grows, the dominant mode estimate shifts, and the cascade
//  targets the next overtone — producing isolated harmonics that
//  appear one at a time and morph smoothly.
//
//  Mode estimation:
//    k_rms = (N/2π) √(Σ(Δq)² / Σq²)
//    This is the amplitude²-weighted spatial frequency.
//    For a pure mode k it gives exactly k.  LP-smoothed so m drifts
//    slowly → smooth morphing rather than jumpy transitions.
//
//  Tension modulation:
//    T(n) = 1 + ε · cos(2π · m · n / N)
//
//  By Fourier's product rule, this creates forces at modes k±m ONLY:
//
//    Starting from mode 1 with m≈1:
//      mode 1 × cos(θ) → modes 2 and 0 (DC removed)
//      → mode 2 appears (isolated frequency doubling)
//    As mode 2 grows, m drifts toward ~1.5:
//      mode 2 × cos(1.5θ) → modes 3.5, 0.5 (weak spectral spread)
//      → modes 3-4 begin appearing (smooth morphing)
//    Then m→2: mode 2 × cos(2θ) → mode 4; mode 1 × cos(2θ) → mode 3
//      → cascade continues: 1 → 2 → 3,4 → 5-8 → ...
//
//  Each new harmonic appears in isolation, grows, then triggers the
//  next.  No broadband q² products — targeted, selective coupling.
//
//  Key difference from V2: m is self-driven (tracks the actual
//  spectrum) instead of externally oscillated.  The cascade is
//  self-organizing and follows the spectral content.
//
//  Integration: kick-drift-kick Störmer-Verlet with m frozen per
//  cycle → symplectic within each cycle.  m evolves slowly between
//  cycles → near-perfect energy conservation.  Safety clamp at 2×.
//
//  Energy: the parametric tension is a conservative Hamiltonian force.
//  H = ½Σv² + ½ΣT(n)(Δq)² is preserved.  Σ(Δq)² ≈ Σ k²|Aₖ|² is
//  approximately preserved, close to the perceptual Σ k|Aₖ|².
//  The curvature cost is the only dissipation: cascade pushes energy
//  UP, cost pushes DOWN → their ratio sets the steady-state bandwidth.
//
//  Memory: q, v, qSmooth, tmp, buffer = 5×N
// ========================================================================
class ManifoldBloomV3 {
    constructor(buffer, { crushRate, costWeight, damping, sustain, step }) {
        this.N = buffer.length;
        this._q = new Float32Array(this.N);
        this._q.set(buffer);
        this.buffer = buffer;
        this.v = new Float32Array(this.N);
        this._qSmooth = new Float32Array(this.N);
        this._tmp = new Float32Array(this.N);

        // crushRate → modulation depth ε (0–0.499)
        // Tension range [1−ε, 1+ε] stays positive for ε < 1.
        this._epsilon = clamp(crushRate ?? 0.1, 0, 0.499);
        this.costWeight = costWeight ?? 0.02;
        this.damping = damping ?? 3.2;
        this.sustain = sustain ?? 0.9999;

        // Pre-smooth position buffer
        for (let pass = 0; pass < 8; pass++) this._smooth(this._q, 0.45);

        // Init velocity (travelling wave)
        for (let n = 0; n < this.N; n++) {
            const nL = (n - 1 + this.N) % this.N;
            const nR = (n + 1) % this.N;
            this.v[n] = -0.5 * (this._q[nR] - this._q[nL]);
        }

        // Initialise kSmooth from the initial waveform's spectral content
        {
            let sumQ2 = 0, sumDq2 = 0;
            for (let n = 0; n < this.N; n++) {
                sumQ2 += this._q[n] * this._q[n];
                const dq = this._q[(n + 1) % this.N] - this._q[n];
                sumDq2 += dq * dq;
            }
            this._kSmooth = sumQ2 > 1e-20
                ? (this.N / (2 * Math.PI)) * Math.sqrt(sumDq2 / sumQ2)
                : 1;
        }

        // Track initial energy for safety clamp
        this._E0 = this._totalEnergy();
    }

    _totalEnergy() {
        const { _q: q, v, N } = this;
        let ke = 0, pe = 0;
        for (let n = 0; n < N; n++) {
            ke += v[n] * v[n];
            const nR = n < N - 1 ? n + 1 : 0;
            const dq = q[nR] - q[n];
            pe += dq * dq;
        }
        return 0.5 * (ke + pe);
    }

    _smooth(arr, c) {
        const { N, _tmp: tmp } = this;
        const wt = 1 - 2 * c;
        for (let n = 0; n < N; n++) {
            tmp[n] = wt * arr[n] + c * (arr[(n - 1 + N) % N] + arr[(n + 1) % N]);
        }
        arr.set(tmp);
    }

    _biDirSmooth(arr, a) {
        const N = this.N;
        const b = 1 - a;
        let s = arr[N - 1];
        for (let n = 0; n < N; n++) s = b * arr[n] + a * s;
        for (let n = 0; n < N; n++) { s = b * arr[n] + a * s; arr[n] = s; }
        s = arr[0];
        for (let n = N - 1; n >= 0; n--) s = b * arr[n] + a * s;
        for (let n = N - 1; n >= 0; n--) { s = b * arr[n] + a * s; arr[n] = s; }
    }

    _biDirSmoothFast(arr, a) {
        const N = this.N;
        const b = 1 - a;
        let s = arr[N - 1];
        for (let n = 0; n < N; n++) { s = b * arr[n] + a * s; arr[n] = s; }
        s = arr[0];
        for (let n = N - 1; n >= 0; n--) { s = b * arr[n] + a * s; arr[n] = s; }
    }

    advanceCycle() {
        const { _q: q, v, N } = this;
        const Nm1 = N - 1;
        const dt = 0.4;
        const halfDt = 0.2;
        const sustain = this.sustain;
        const epsilon = this._epsilon;

        // ── 1. DC removal + sustain (both q AND v) ──
        let dc = 0;
        for (let n = 0; n < N; n++) dc += q[n];
        dc /= N;
        for (let n = 0; n < N; n++) {
            q[n] = (q[n] - dc) * sustain;
            v[n] *= sustain;
        }

        // ── 2. Curvature cost — tames high harmonics, balances cascade ──
        if (this.costWeight > 0) {
            const qSm = this._qSmooth;
            qSm.set(q);
            this._biDirSmooth(qSm, 0.75);
            const cw = this.costWeight;
            for (let n = 0; n < N; n++) {
                q[n] += cw * (qSm[n] - q[n]);
            }
        }

        // ── 3. Estimate dominant mode from gradient/displacement ratio ──
        //
        // k_rms = (N/2π) √(Σ(Δq)²/Σq²) gives the amplitude²-weighted
        // spatial frequency.  LP-smoothed so the modulation frequency
        // drifts slowly → smooth morphing, not jittery jumps.
        //
        // Time constant ≈ 1/0.02 = 50 cycles.  At typical 1kHz pitch
        // (1000 cycles/sec) this is 0.05 seconds — slow enough for
        // individual harmonics to establish before the next appears.
        let sumQ2 = 0, sumDq2 = 0;
        for (let n = 0; n < N; n++) {
            sumQ2 += q[n] * q[n];
            const dq = q[n < Nm1 ? n + 1 : 0] - q[n];
            sumDq2 += dq * dq;
        }
        const kEst = sumQ2 > 1e-20
            ? (N / (2 * Math.PI)) * Math.sqrt(sumDq2 / sumQ2)
            : 1;
        this._kSmooth += 0.02 * (kEst - this._kSmooth);
        const m = Math.max(0.5, Math.min(this._kSmooth, N / 4));

        // ── 4. KDK Verlet with parametric tension ──
        //
        // T(n) = 1 + ε·cos(2π·m·n/N)
        // Goertzel recurrence generates the cosine inline.
        // m is frozen for both kicks → symplectic within this cycle.
        const modOmega = 2 * Math.PI * m / N;
        const twoCosW = 2 * Math.cos(modOmega);
        const cosOmega = Math.cos(modOmega);

        // First half-kick: v += ½dt·T(n)·Laplacian(q)
        {
            let cosM = 1, cosPrev = cosOmega;
            for (let n = 0; n < N; n++) {
                const nL = n > 0 ? n - 1 : Nm1;
                const nR = n < Nm1 ? n + 1 : 0;
                const T = 1 + epsilon * cosM;
                v[n] += halfDt * T * (q[nR] + q[nL] - 2 * q[n]);
                const cosNext = twoCosW * cosM - cosPrev;
                cosPrev = cosM;
                cosM = cosNext;
            }
        }

        // Drift: q += dt·v
        for (let n = 0; n < N; n++) {
            q[n] += dt * v[n];
        }

        // Second half-kick: v += ½dt·T(n)·Laplacian(q_new)
        {
            let cosM = 1, cosPrev = cosOmega;
            for (let n = 0; n < N; n++) {
                const nL = n > 0 ? n - 1 : Nm1;
                const nR = n < Nm1 ? n + 1 : 0;
                const T = 1 + epsilon * cosM;
                v[n] += halfDt * T * (q[nR] + q[nL] - 2 * q[n]);
                const cosNext = twoCosW * cosM - cosPrev;
                cosPrev = cosM;
                cosM = cosNext;
            }
        }

        // ── 5. Energy safety clamp ──
        if (this._E0 > 1e-20) {
            const E = this._totalEnergy();
            if (E > 2 * this._E0) {
                const s = Math.sqrt(this._E0 / E);
                for (let n = 0; n < N; n++) { q[n] *= s; v[n] *= s; }
            }
        }

        // ── 6. Output ──
        this.buffer.set(q);

        // ── 7. Output LP filter ──
        if (this.damping > 0) {
            this._biDirSmoothFast(this.buffer, this.damping / (this.damping + 1));
        }
    }

    snapshot() { return new Float32Array(this.buffer); }
}

// ========================================================================
//  ManifoldBloomV4 — Goertzel spotlight with exact RMS-lock sustain
//
//  Solves three structural problems with V2/V3:
//
//  1. HAMILTONIAN ENERGY CONSERVATION.
//     After every advanceCycle(), H = ½Σv² + ½Σ(Δq)² is measured and
//     q,v are rescaled together by sqrt(H_target/H_new).  RMS(q) is
//     FREE TO VARY as energy sloshes between KE and PE — giving organic
//     amplitude modulation — while the total energy budget is bounded.
//     No decay, no injection, no balance to tune.
//
//  2. ASYMMETRIC ENERGY CONCENTRATION via Goertzel projection.
//     The cos/sin recurrences already run in the force loop; two extra
//     accumulators (dotC, dotS) measure the spotlight mode for free
//     (2 extra muls/sample, zero extra trig calls beyond recurrence init).
//     After the force loop, one coherent pass writes:
//       q_new[n] = spotScale · s[n]  +  residScale · (q[n] − s[n])
//     where s[n] is the spotlight component and the scales route a
//     chosen fraction of total energy toward mode m.  Because spotlight
//     and residual are orthogonal, total energy is analytically conserved
//     before the normalisation clamp — no approximation.
//
//  3. NO costWeight LOW-PASS on the physics state.
//     The biDirSmooth in V2 was a low-pass that preferentially preserved
//     mode 1 (passes intact) while damping high modes.  Removed entirely.
//     Output damping is still available but applied to the output copy
//     only, never feeding back into the physics.
//
//  Per-cycle cost:
//    Force loop:   O(N)  + 4 muls/sample extra (sin recurrence + Goertzel)
//    Pass A:       O(N)  residual store + E_total accumulation
//    Pass B:       O(N)  concentrated + normalised write (fused)
//    Total:        ~3 × O(N), no FFT, 2 trig calls at start of cycle
//
//  Memory: q, v, aq, tmp  =  4 × N
//
//  Parameters:
//    coupling      → parametric modulation depth ε  (0–1 → ε 0–0.5)
//    concentration → fraction of energy routed to spotlight per cycle
//    alpha         → nonlinear spring (odd harmonics)
//    resTension    → harmonic range of the drifting spotlight
//    morphRate     → drift speed
//    selectivity   → integer-pinch strength (0 = linear, 1 = strong dwell)
//    asymmetry     → even-harmonic waveshaping on output copy only
//    damping       → output LP filter (cosmetic, does not affect physics)
// ========================================================================
class ManifoldBloomV4 {
    constructor(buffer, {
        alpha, asymmetry, coupling, resTension,
        damping, morphRate, selectivity, concentration, step
    }) {
        this._q = new Float32Array(buffer.length);
        this._q.set(buffer);
        this.buffer = buffer;
        this.N = buffer.length;
        this.v = new Float32Array(this.N);
        this._aq = new Float32Array(this.N);
        this._tmp = new Float32Array(this.N);

        this.alpha = alpha ?? 0.1;
        this.asymmetry = asymmetry ?? 0.0;
        this.damping = damping ?? 1.0;
        this._pitchScale = 1 / Math.max(step || 1, 0.01);

        // Padé [3,2] constants
        const G2 = (this.N / (2 * Math.PI)) ** 2;
        this._G2x5 = 5 * G2;
        this._G2x6 = 6 * G2;

        // Parametric modulation depth  (coupling 0→1 maps to ε 0→0.5)
        this._modEpsilon = clamp((coupling ?? 0.5) * 0.5, 0, 0.49);

        // Spotlight drift range
        const rt = resTension ?? 1.0;
        const modCenter = 2 + rt;
        const modRange = 1 + rt;
        this._mMin = Math.max(1, Math.round(modCenter - modRange));
        this._mMax = Math.round(modCenter + modRange);

        // ── Integer state machine spotlight ──
        //
        // _modFreq is ALWAYS an integer.  This guarantees:
        //   • T(n) = 1 + ε·cos(2π·m·n/N) is periodic over N — no buffer-wrap
        //     discontinuity, no buzzing.
        //   • Spotlight concentration operates on exact DFT bins.
        //
        // Progression: each dwell lasts 1/_driftRate cycles (morphRate).
        // On each jump the logistic chaotic map (r=3.99, fully chaotic)
        // drives both the step size and direction — non-consecutive.
        //
        // selectivity 0→1.5 maps to maxStep 1→floor(range×0.75)+1
        //   0   → step always 1 (consecutive, useful for debugging)
        //   0.7 → step up to ~half the range (good default)
        //   1.5 → step up to ~all of the range (very jumpy)
        const mr = (morphRate ?? 0.5) * this._pitchScale;
        this._driftRate = mr * 0.015;
        this._dwellT = 0;
        this._logistic = 0.62;    // chaotic state for jump destination
        this._logistic2 = 0.37;    // independent chaotic state for dwell duration
        this._mCurrent = Math.round(modCenter);
        const selRange = this._mMax - this._mMin;
        this._maxStep = Math.max(1, Math.round((selectivity ?? 0.7) * 0.5 * selRange) + 1);
        // Initial dwell target: chaotic value in [0.3, 2.0] × base period.
        // Redrawn after every jump so interval between jumps is irregular.
        this._dwellTarget = 0.3 + 1.7 * this._logistic2;

        // Crossfade state.
        // _mFrom/_mTo are the two integer modes being crossfaded between.
        // _morphBlend 0→1 weights concentration from _mFrom toward _mTo.
        // _morphSpeed controls how many cycles the crossfade takes:
        //   speed = _driftRate * 3  → crossfade ≈ 33% of avg dwell time.
        this._mFrom = this._mCurrent;
        this._mTo = this._mCurrent;
        this._morphBlend = 1.0;   // start fully settled
        this._morphSpeed = this._driftRate * 3;

        // Energy concentration per cycle.
        // 0    = no concentration (symmetric wave physics only)
        // 0.15 = gentle — recommended starting point
        // 0.8  = aggressive (very sparse, one dominant mode at a time)
        this._concentration = clamp(concentration ?? 0.15, 0, 0.95);

        this._modFreq = this._mCurrent;   // always integer

        // Initialise velocity as a travelling wave
        for (let n = 0; n < this.N; n++) {
            const nL = (n - 1 + this.N) % this.N;
            const nR = (n + 1) % this.N;
            this.v[n] = -0.5 * (this._q[nR] - this._q[nL]);
        }

        this._initForces();   // uses _modFreq = _mCurrent (integer) set above

        // Hamiltonian conservation target: H = ½Σv² + ½Σ(Δq)²
        let KE = 0, PE = 0;
        for (let n = 0; n < this.N; n++) {
            KE += this.v[n] * this.v[n];
            const nR = (n + 1) % this.N;
            const d = this._q[nR] - this._q[n];
            PE += d * d;
        }
        this._targetH = 0.5 * (KE + PE);
    }

    _initForces() {
        const { _q: q, N, _aq: aq } = this;
        const Nm1 = N - 1;
        const TWO_PI = 6.283185307179586;
        const modOmega = TWO_PI * this._modFreq / N;
        const twoCosW = 2 * Math.cos(modOmega);
        let cosM = 1, cosPrev = Math.cos(modOmega);
        const alpha = this.alpha, G2x5 = this._G2x5, G2x6 = this._G2x6;
        const epsilon = this._modEpsilon;
        for (let n = 0; n < N; n++) {
            const nL = n > 0 ? n - 1 : Nm1;
            const nR = n < Nm1 ? n + 1 : 0;
            const dR = q[nR] - q[n], dL = q[n] - q[nL];
            const tension = 1 + epsilon * cosM;
            let fq = tension * (dR - dL);
            if (alpha !== 0) {
                const dR2 = dR * dR, dL2 = dL * dL;
                fq += alpha * (G2x5 * dR2 * dR / (15 + G2x6 * dR2)
                    - G2x5 * dL2 * dL / (15 + G2x6 * dL2));
            }
            aq[n] = fq;
            const cosNext = twoCosW * cosM - cosPrev;
            cosPrev = cosM; cosM = cosNext;
        }
    }

    _biDirSmoothFast(arr, a) {
        const N = this.N, b = 1 - a;
        let s = arr[N - 1];
        for (let n = 0; n < N; n++) { s = b * arr[n] + a * s; arr[n] = s; }
        s = arr[0];
        for (let n = N - 1; n >= 0; n--) { s = b * arr[n] + a * s; arr[n] = s; }
    }

    advanceCycle() {
        const { _q: q, v, N, _aq: aq, _tmp: tmp } = this;
        const Nm1 = N - 1;
        const TWO_PI = 6.283185307179586;
        const dt = 0.4, halfDt = 0.2, halfDt2 = 0.08;
        const alpha = this.alpha, G2x5 = this._G2x5, G2x6 = this._G2x6;
        const epsilon = this._modEpsilon;

        // ── 1. State machine + crossfade blend ──
        //
        // On each jump: _mFrom = old, _mTo = new, _morphBlend resets to 0.
        // _morphBlend advances each cycle toward 1 at _morphSpeed.
        // Concentration blends kFrom=k*(1-blend) toward mFrom and
        // kTo=k*blend toward mTo, giving a smooth spectral fade.
        this._dwellT += this._driftRate;
        if (this._dwellT >= this._dwellTarget) {
            this._dwellT = 0;
            this._logistic = 3.99 * this._logistic * (1 - this._logistic);
            const logVal = this._logistic;
            const step = 1 + Math.floor(logVal * this._maxStep);
            const dir = logVal < 0.5 ? 1 : -1;
            const mNew = this._mCurrent + dir * step;
            const mTarget = (mNew < this._mMin || mNew > this._mMax)
                ? clamp(this._mCurrent - dir * step, this._mMin, this._mMax)
                : mNew;
            if (mTarget !== this._mCurrent) {
                this._mFrom = this._mCurrent;
                this._mTo = mTarget;
                this._mCurrent = mTarget;
                this._morphBlend = 0;
            }
            this._logistic2 = 3.99 * this._logistic2 * (1 - this._logistic2);
            this._dwellTarget = 0.3 + 1.7 * this._logistic2;
        }
        this._morphBlend = Math.min(1, this._morphBlend + this._morphSpeed);
        const blend = this._morphBlend;
        const mFrom = this._mFrom;
        const mTo = this._mTo;
        this._modFreq = mTo;   // for force recompute + _initForces consistency

        // ── 2. Position Verlet drift + DC removal ──
        let dc = 0;
        for (let n = 0; n < N; n++) { q[n] += dt * v[n] + halfDt2 * aq[n]; dc += q[n]; }
        dc /= N;
        for (let n = 0; n < N; n++) q[n] -= dc;

        // ── 3. Force loop — dual Goertzel + blended parametric tension ──
        //
        // Two independent cosine/sine recurrences run simultaneously:
        //   F-channel: mFrom (fading out)
        //   T-channel: mTo   (fading in)
        // Blended tension:  T(n) = 1 + ε·[(1−blend)·cosF_n + blend·cosT_n]
        // This keeps the parametric force law smooth throughout the crossfade.
        //
        // Dual Goertzel accumulators (dotCF,dotSF / dotCT,dotST) measure
        // each spotlight's current amplitude for the concentration step.
        // E_total is also accumulated here, eliminating a separate pass.
        const omegaF = TWO_PI * mFrom / N;
        const omegaT = TWO_PI * mTo / N;
        const tCWF = 2 * Math.cos(omegaF);
        const tCWT = 2 * Math.cos(omegaT);
        let cosF = 1, cosFPrev = Math.cos(omegaF);
        let sinF = 0, sinFPrev = -Math.sin(omegaF);
        let cosT = 1, cosTprev = Math.cos(omegaT);
        let sinT = 0, sinTPrev = -Math.sin(omegaT);
        let dotCF = 0, dotSF = 0, dotCT = 0, dotST = 0;
        let E_total = 0;

        for (let n = 0; n < N; n++) {
            const nL = n > 0 ? n - 1 : Nm1;
            const nR = n < Nm1 ? n + 1 : 0;
            const aq_old = aq[n];
            const qN = q[n], qR = q[nR], qL = q[nL];
            const dR = qR - qN, dL = qN - qL;

            const tension = 1 + epsilon * ((1 - blend) * cosF + blend * cosT);
            let fq = tension * (dR - dL);
            if (alpha !== 0) {
                const dR2 = dR * dR, dL2 = dL * dL;
                fq += alpha * (G2x5 * dR2 * dR / (15 + G2x6 * dR2)
                    - G2x5 * dL2 * dL / (15 + G2x6 * dL2));
            }
            v[n] += halfDt * (aq_old + fq);
            aq[n] = fq;

            dotCF += qN * cosF; dotSF += qN * sinF;
            dotCT += qN * cosT; dotST += qN * sinT;
            E_total += qN * qN;

            const cosFNext = tCWF * cosF - cosFPrev; cosFPrev = cosF; cosF = cosFNext;
            const sinFNext = tCWF * sinF - sinFPrev; sinFPrev = sinF; sinF = sinFNext;
            const cosTNext = tCWT * cosT - cosTprev; cosTprev = cosT; cosT = cosTNext;
            const sinTNext = tCWT * sinT - sinTPrev; sinTPrev = sinT; sinT = sinTNext;
        }

        // ── 4. Dual-spotlight energy-conserving concentration ──
        //
        // Integer modes are orthogonal over exactly N samples:
        //   Σ cos(2π·k1·n/N)·cos(2π·k2·n/N) = 0  for k1 ≠ k2
        // So E_from, E_to, E_resid = E_total − E_from − E_to  (exact).
        //
        // kFrom = k·(1−blend) routes concentration toward mFrom (fading out).
        // kTo   = k·blend     routes concentration toward mTo   (fading in).
        // Delta approach — no tmp[] needed:
        //   q[n] += (scaleFrom−1)·sFrom[n] + (scaleTo−1)·sTo[n]
        // where sX[n] = (2/N)·(dotCX·cosX_n + dotSX·sinX_n).
        // After this, Σq_new² = E_total exactly (orthogonality guarantees it).
        const k = this._concentration;
        const inv2N = 2 / N;
        const E_from = inv2N * (dotCF * dotCF + dotSF * dotSF);
        const E_to_e = inv2N * (dotCT * dotCT + dotST * dotST);
        const E_resid = Math.max(0, E_total - E_from - E_to_e);
        const kFrom = k * (1 - blend);
        const kTo = k * blend;
        const sfM1 = E_from > 1e-20
            ? Math.sqrt((E_from + kFrom * E_resid) / E_from) - 1 : 0;
        const stM1 = E_to_e > 1e-20
            ? Math.sqrt((E_to_e + kTo * E_resid) / E_to_e) - 1 : 0;

        cosF = 1; cosFPrev = Math.cos(omegaF);
        sinF = 0; sinFPrev = -Math.sin(omegaF);
        cosT = 1; cosTprev = Math.cos(omegaT);
        sinT = 0; sinTPrev = -Math.sin(omegaT);
        for (let n = 0; n < N; n++) {
            const sF = inv2N * (dotCF * cosF + dotSF * sinF);
            const sT = inv2N * (dotCT * cosT + dotST * sinT);
            q[n] += sfM1 * sF + stM1 * sT;
            const cosFNext = tCWF * cosF - cosFPrev; cosFPrev = cosF; cosF = cosFNext;
            const sinFNext = tCWF * sinF - sinFPrev; sinFPrev = sinF; sinF = sinFNext;
            const cosTNext = tCWT * cosT - cosTprev; cosTprev = cosT; cosT = cosTNext;
            const sinTNext = tCWT * sinT - sinTPrev; sinTPrev = sinT; sinT = sinTNext;
        }

        // ── 5. Hamiltonian conservation clamp ──
        //
        // H = ½Σv² + ½Σ(Δq)²
        // Concentration changed the spectral shape of q, changing PE.
        // Rescale q and v (and aq, since it has units of acceleration ∝ q)
        // by s = sqrt(H_target / H_new) so total energy is restored.
        {
            let sumKE = 0, sumPE = 0;
            for (let n = 0; n < N; n++) {
                sumKE += v[n] * v[n];
                const nR = n < Nm1 ? n + 1 : 0;
                const d = q[nR] - q[n];
                sumPE += d * d;
            }
            const H_new = 0.5 * (sumKE + sumPE);
            if (H_new > 1e-20) {
                const s = Math.sqrt(this._targetH / H_new);
                for (let n = 0; n < N; n++) { q[n] *= s; v[n] *= s; }
            }
        }

        // ── 6. Recompute cached forces from final q ──
        //
        // Uses blended tension (same as the force loop above) so aq[] is
        // consistent with the q that will be read at the start of the next cycle.
        {
            let cF = 1, cFp = Math.cos(omegaF);
            let cT = 1, cTp = Math.cos(omegaT);
            for (let n = 0; n < N; n++) {
                const nL = n > 0 ? n - 1 : Nm1;
                const nR = n < Nm1 ? n + 1 : 0;
                const dR = q[nR] - q[n], dL = q[n] - q[nL];
                const ten = 1 + epsilon * ((1 - blend) * cF + blend * cT);
                let fq = ten * (dR - dL);
                if (alpha !== 0) {
                    const dR2 = dR * dR, dL2 = dL * dL;
                    fq += alpha * (G2x5 * dR2 * dR / (15 + G2x6 * dR2)
                        - G2x5 * dL2 * dL / (15 + G2x6 * dL2));
                }
                aq[n] = fq;
                const cFn = tCWF * cF - cFp; cFp = cF; cF = cFn;
                const cTn = tCWT * cT - cTp; cTp = cT; cT = cTn;
            }
        }

        // ── 6. Output: copy + optional even-harmonic waveshaping ──
        const buf = this.buffer;
        if (this.asymmetry > 0) {
            const beta = this.asymmetry;
            let sumQ2 = 0;
            for (let n = 0; n < N; n++) { buf[n] = q[n]; sumQ2 += q[n] * q[n]; }
            const rmsBefore = Math.sqrt(sumQ2 / N);
            if (rmsBefore > 1e-12) {
                const dcQ2 = sumQ2 / N;
                let sumAfter = 0;
                for (let n = 0; n < N; n++) {
                    const bn = buf[n];
                    const shaped = bn + beta * (bn * bn - dcQ2);
                    buf[n] = shaped;
                    sumAfter += shaped * shaped;
                }
                const rmsAfter = Math.sqrt(sumAfter / N);
                if (rmsAfter > 1e-12) {
                    const sc = rmsBefore / rmsAfter;
                    for (let n = 0; n < N; n++) buf[n] *= sc;
                }
            }
        } else {
            buf.set(q);
        }

        // ── 6. Output LP (cosmetic — does not affect physics state) ──
        if (this.damping > 0) {
            this._biDirSmoothFast(buf, this.damping / (this.damping + 1));
        }
    }

    snapshot() { return new Float32Array(this.buffer); }
}

// ========================================================================
//  RingCoupler — two counter-propagating rings with exact energy conservation
//
//  Update rule (two passes each cycle):
//
//  Pass 1 — 3-tap FIR on each ring independently:
//    tA[n] = a1·A[n-1] + a2·A[n+1]          (forward,  a1=a2=0.5)
//    tB[n] = a1·B[n+1] + a2·B[n-1]          (reversed, counter-propagating)
//
//    The FIR has frequency response cos(ω) ≤ 1, so it can only dissipate
//    energy (gentle LP rolloff), never inject it.
//
//  Pass 2 — pairwise rotation between rings:
//    For each n, let bIdx = (n+φ)%N.
//    Rotation angle: θ[n] = cp + as · tA[n] · tB[bIdx]
//      • cp: base coupling (constant rotation)
//      • as · tA·tB: nonlinear term — the bilinear product modulates the
//        mixing angle, exactly as in the learnable kernel spec (a4·B[n]·A[n]).
//
//    A_new[n]     =  cos(θ) · tA[n]    + sin(θ) · tB[bIdx]
//    B_new[bIdx]  = −sin(θ) · tA[n]    + cos(θ) · tB[bIdx]
//
//    Each pair (tA[n], tB[bIdx]) is an orthogonal 2×2 Givens rotation →
//    A[n]² + B[bIdx]² is preserved exactly.  Since all pairs are disjoint
//    (φ is a uniform shift), the total energy ΣA² + ΣB² is conserved exactly.
//    Combined with the FIR's non-increasing property, total system energy
//    is non-increasing every cycle — no clamping needed.
//
//  φ drifts quasiperiodically (two incommensurate golden-ratio frequencies)
//  → non-repeating, deterministic, organic spectral modulation.
//
//  Parameters:
//    coupling   → cp: base rotation angle in radians (0 = no coupling, π/4 = full mix)
//    asymmetry  → as: nonlinear angle modulation depth (bilinear product)
//    morphRate  → φ drift speed
//    damping    → output LP (cosmetic, does not affect ring physics)
// ========================================================================
class RingCoupler {
    constructor(buffer, { coupling, asymmetry, strength, morphRate, damping, step }) {
        this.N = buffer.length;
        this.buffer = buffer;

        // Ring A: audio output, initialised from wavetable
        this._A = new Float32Array(this.N);
        this._A.set(buffer);

        // Ring B: hidden state, 90°-shifted half-amplitude copy.
        // 90° phase ensures tA[n]·tB[bIdx] ≠ 0 at t=0 so the nonlinear
        // modulation fires immediately.
        this._B = new Float32Array(this.N);
        const shift = Math.round(this.N / 4);
        for (let i = 0; i < this.N; i++)
            this._B[i] = this._A[(i + shift) % this.N] * 0.5;

        this._tA = new Float32Array(this.N);
        this._tB = new Float32Array(this.N);

        // Base rotation angle, nonlinear depth and overall strength.
        // cp: base rotation angle character (0 = flat, π/4 = full blend).
        // as: nonlinear depth — bilinear product tA·tB modulates θ.
        //   tA·tB ≲ rms² ≈ 0.25 for unit signals, so as=1 → ≲0.25 rad perturbation.
        // st: overall strength scalar.  θ_eff = st · (cp + as·tA·tB).
        //   st=0 → rings fully decoupled.  st=1 → full coupling as set by cp/as.
        this._cp = clamp(coupling ?? 0.1, 0, Math.PI / 4);
        this._as = clamp(asymmetry ?? 0.1, -1, 1);
        this._st = clamp(strength ?? 1.0, 0, 1);

        // φ drift
        this._phiBase = Math.round(this.N / 4);
        this._phiRange = Math.round(this.N / 8);
        this._phi = this._phiBase;

        const ps = 1 / Math.max(step || 1, 0.01);
        const mr = (morphRate ?? 0.5) * ps;
        this._dr1 = mr * 0.15;
        this._dr2 = this._dr1 * 1.6180339887;
        this._ph1 = 0;
        this._ph2 = 0;

        this.damping = damping ?? 0.0;
    }

    advanceCycle() {
        const { _A: A, _B: B, _tA: tA, _tB: tB, N } = this;
        const Nm1 = N - 1;

        // ── 1. Drift φ quasiperiodically ──
        this._ph1 += this._dr1;
        this._ph2 += this._dr2;
        const phiRaw = this._phiRange * (
            0.7 * Math.sin(this._ph1) + 0.3 * Math.sin(this._ph2));
        this._phi = ((Math.round(this._phiBase + phiRaw) % N) + N) % N;
        const phi = this._phi;
        console.log(`φ: ${phi} / ${N}`);

        // ── 2. Pass 1: 3-tap FIR (energy non-increasing) ──
        // Hann-weighted: H(ω) = 0.5 + 0.5·cos(ω) ∈ [0, 1] for all ω.
        // The center tap (weight 0.5) ensures H is never negative, preventing
        // the frequency inversion that occurs with a pure neighbour average.
        // A propagates forward, B propagates backward (nL/nR swapped).
        for (let n = 0; n < N; n++) {
            const nL = n > 0 ? n - 1 : Nm1;
            const nR = n < Nm1 ? n + 1 : 0;
            tA[n] = 0.5 * A[n] + 0.25 * A[nL] + 0.25 * A[nR];
            tB[n] = 0.5 * B[n] + 0.25 * B[nR] + 0.25 * B[nL];   // reversed
        }

        const cp = this._cp;
        const as = this._as;
        const st = this._st;

        // ── 3. Pass 2: pairwise Givens rotation between tA[n] and tB[(n+φ)%N] ──
        // Each rotation preserves tA[n]² + tB[bIdx]² exactly.
        // Nonlinear angle: θ = st · (cp + as·tA[n]·tB[bIdx])
        // cos/sin computed per sample — trig is unavoidable for exactness but
        // N=1024 rotations/cycle is cheap compared to the allpass warmup loops.
        for (let n = 0; n < N; n++) {
            const bIdx = (n + phi) % N;
            const an = tA[n];
            const bn = tB[bIdx];
            const amt = (an + 1) / 2
            const theta = st * amt * (cp + as * an * bn);
            const cosT = Math.cos(theta);
            const sinT = Math.sin(theta);
            A[n] = cosT * an + sinT * bn;
            tB[bIdx] = -sinT * an + cosT * bn;  // stage into tB to avoid overwriting live B
        }
        // Commit rotated B values
        B.set(tB);

        // ── 4. DC removal on A ──
        let dc = 0;
        for (let n = 0; n < N; n++) dc += A[n];
        dc /= N;
        for (let n = 0; n < N; n++) A[n] -= dc;

        // ── 5. Ring damping — bidirectional LP on A and B (shapes ring state,
        //       not just the output, so HF energy can't accumulate in hidden buffer) ──
        if (this.damping > 0) {
            const a_lp = this.damping / (this.damping + 1);
            const b_lp = 1 - a_lp;
            const B2 = this._B;
            let s = A[N - 1];
            for (let n = 0; n < N; n++) { s = b_lp * A[n] + a_lp * s; A[n] = s; }
            s = A[0];
            for (let n = N - 1; n >= 0; n--) { s = b_lp * A[n] + a_lp * s; A[n] = s; }
            s = B2[N - 1];
            for (let n = 0; n < N; n++) { s = b_lp * B2[n] + a_lp * s; B2[n] = s; }
            s = B2[0];
            for (let n = N - 1; n >= 0; n--) { s = b_lp * B2[n] + a_lp * s; B2[n] = s; }
        }

        // ── 6. Copy A → output buffer ──
        this.buffer.set(A);
    }

    snapshot() { return new Float32Array(this.buffer); }
}

// ── RingCouplerX ─────────────────────────────────────────────────────────────
// R hidden rings coupled to audio ring A via Givens rotations.
// Designed as an over-parameterised ~24-scalar optimisation target.
//
// Parameterisation for R rings:
//   theta[r]     – base rotation angle between A and B[r]    (master: coupling)
//   eta[r]       – nonlinear modulation depth (bilinear term) (master: nonlin)
//   phiFrac[r]   – ring offset as fraction of N
//   driftBase[r] – quasiperiodic drift speed                  (master: morphRate)
//   tilt[r]      – per-ring one-pole LP on B only             (master: damping)
//   + shared 5-tap FIR kernel (non-negative, sum=1 → |H(ω)| ≤ 1 for all ω)
//                              4 free params after normalization
//   Total: 5R + 4  ≈ 24 learnable scalars (R=4)
//
// Energy structure:
//   FIR pass    : |H(ω)| ≤ 1  — non-negative taps summing to 1
//   Givens rot  : A[n]² + B[r][bIdx]² conserved exactly per pair
//   Tilt LP     : applied to B only — affects A indirectly via coupling
//
// Master sliders multiply per-ring defaults proportionally.
// Call synth.getParams() from the browser console to serialise the full
// 24-scalar set for use as an optimisation starting point / target.

class RingCouplerX {
    // R = 4 rings, fixed.  All 33 scalars are directly learnable.
    //
    // Nonlinearity is decomposed into three independent per-ring terms:
    //   η_a  · aₙ           — linear-in-A, odd:       amplitude-FM / saturation
    //   η_ab · aₙ·bₙ        — bilinear, odd×odd:      intermodulation
    //   η_sq · aₙ²          — quadratic-in-A, EVEN:   generates even harmonics
    //
    // Full angle: θ = base_θ + η_a·aₙ + η_ab·aₙ·bₙ + η_sq·aₙ²
    // Mixed even+odd terms give asymmetric waveform evolution without adding energy.
    //
    // Energy structure:
    //   FIR pass    : |H(ω)| ≤ 1  (non-negative taps, sum=1)
    //   Givens rot  : ‖A[n]‖² + ‖B[r][bIdx]‖² conserved exactly per pair
    //   Spectral tilt: applied to B only — A affected indirectly via Givens coupling

    static defaultParams() {
        const gr = 1.6180339887;
        const R = 4;
        return {
            // Per-ring half-spec FIR: firTaps[r][0] = centre weight, firTaps[r][k] = weight for ±k.
            // Normalised so DC gain = 1, enforcing |H(ω)| ≤ 1 at all frequencies.
            // Each ring has its own filter so it can independently shape which harmonics
            // dominate B—and therefore which harmonics couple most strongly into A.
            firTaps: Array.from({ length: R }, () => [0.5, 0.25, 0.125, 0.0625, 0.0625]),
            // Per-ring base rotation angles.
            thetas: Array.from({ length: R }, (_, r) => 0.04 / (r * gr + 1)),
            // Per-ring overall coupling strength (0 = decoupled, 1 = full).
            strengths: Array(R).fill(1.0),
            // Per-ring nonlinear coefficients.
            // etasA:   linear in aₙ (odd — saturation).  Keep small: max useful ≈ ±0.05.
            // etasAB:  bilinear aₙ·bₙ (odd — intermodulation).
            // etasSq:  quadratic aₙ² (even DC bias on θ, but CANNOT break anti-symmetry alone).
            // etasCub: cubic aₙ²·bₙ — the lowest-order term that actually breaks
            //          half-wave anti-symmetry: at the paired sample (-aₙ)²(-bₙ) = -aₙ²bₙ.
            //          This is the proper mechanism for generating even harmonics.
            etasA: Array(R).fill(0.0),
            etasAB: Array.from({ length: R }, (_, r) => 0.04 / (r * gr + 1)),
            etasSq: Array(R).fill(0.0),
            etasCub: Array(R).fill(0.0),
            // Per-ring offset as fraction of N (golden-ratio spaced).
            phiFracs: Array.from({ length: R }, (_, r) => (r + 1) / (R + 1)),
            // Per-ring drift speed.  Incommensurate by construction.
            driftBases: Array.from({ length: R }, (_, r) => 0.18 * Math.pow(gr, r)),
            // Per-ring drift depth: phi wanders ±(driftDepth × N) samples around phi0.
            // 0.125 = N/8 (original hardcoded value). Max useful ≈ 0.5 (half-period).
            driftDepths: Array(R).fill(0.125),
            // Per-ring spectral tilt (one-pole LP coefficient, 0=flat, →1=heavy).
            tilts: Array(R).fill(0.0),
            // Dispersion: all-pass chain applied to A each cycle.
            // dispCoeff: all-pass coefficient (0=off, →1=strong inharmonic stretch).
            // dispStages: number of all-pass stages chained (more = more stretch, more cost).
            dispCoeff: 0.0,
            dispStages: 4,
            // Global one-pole LP applied once to A per step (not per-ring, not cascaded).
            // Provides direct, controllable HF decay on the main buffer independent of R.
            tiltA: 0.1,
            // Chain cross-mod: B[r] ↔ B[(r+1)%R], forming a ring among hidden buffers.
            // xModThetas:    base Givens rotation angle for each B–B link.
            // xModStrengths: overall strength of each link (0 = link disabled, default).
            // Both zero = pure star topology (original behaviour).
            xModThetas: Array(R).fill(0.0),
            xModStrengths: Array(R).fill(0.0),
            // Global rotation clamp: hard-clips the total θ angle before cos/sin.
            // Prevents burst explosions when B regrows after a quiet period.
            // Default 0.08 rad → half-rotation ≥ 89ms at 440Hz regardless of any
            // combination of nonlinear terms.
            thetaClamp: 0.08,
            // Per-cycle noise injected into each B ring (RMS per sample).
            // 1e-4 is inaudible but prevents B from dying completely, keeping
            // evolution continuous rather than quiet-then-burst.
            noiseFloor: 1e-4,
            // Inharmonicity: polynomial phase-shift coefficients [c1,c2,c3,c4].
            // phase(k) = c1·k + c2·k² + c3·k³ + c4·k⁴  per cycle.
            // Shifts the apparent frequency of harmonic k away from exact integer
            // multiples of f0, modelling the behaviour of stiff bars/plates.
            inharmCoeffs: [0, 0, 0, 0],
            // Learnable pitch correction: fractional circular shift (samples)
            // applied to A and all Bs every cycle.  Compensates for f0 detection
            // error — the accumulated shift over many cycles adjusts the effective
            // fundamental frequency.
            pitchShift: 0,
        };
    }

    constructor(buffer, { params = null, step }) {
        const N = buffer.length;
        const R = 4;
        this.N = N; this.R = R; this.buffer = buffer;
        const p = params ?? RingCouplerX.defaultParams();
        const gr = 1.6180339887;

        // ── Per-ring half-spec symmetric FIRs ──
        // firTaps[r][0] = centre coefficient (applied once per ring r).
        // firTaps[r][k] = coefficient for offset ±k (applied to B[n+k] AND B[n-k]).
        // DC gain = taps[0] + 2·(taps[1]+…+taps[4]). Normalised so DC=1 → |H(ω)| ≤ 1 ∀ω.
        // Accepts legacy 1D [K] format (shared across all rings) or new 2D [R][K] format.
        const normTaps = (taps) => {
            const a = taps.slice(0, 5).map(v => Math.abs(v));
            const s = a[0] + 2 * a.slice(1).reduce((x, y) => x + y, 0) || 1;
            return new Float32Array(a.map(v => v / s));
        };
        const rawFirTaps = p.firTaps ?? [0.5, 0.25, 0.125, 0.0625, 0.0625];
        if (Array.isArray(rawFirTaps[0])) {
            // New 2D [R][K] format: one taps array per ring
            this._firs = rawFirTaps.map(normTaps);
        } else {
            // Legacy 1D [K] format: broadcast to all rings
            const shared = normTaps(rawFirTaps);
            this._firs = Array.from({ length: R }, () => new Float32Array(shared));
        }
        // _halfK retained for stride indexing (unchanged meaning)
        this._halfK = Math.floor(this._firs[0].length / 2);   // = 2

        // ── Audio ring A ──
        this._A = new Float32Array(N);
        this._A.set(buffer);

        // ── Hidden rings B[r]: phase-staggered, amplitude matched to coupling strength ──
        // Warm-starting at strengths[r] * A avoids a charge-up transient where an
        // under-filled ring drains energy from A for the first few cycles, which
        // would look like artificial extra damping independent of the tilt parameter.
        // A tiny asymmetric linear tilt is added to seed even-harmonic generation.
        this._Bs = Array.from({ length: R }, (_, r) => {
            const B = new Float32Array(N);
            const phi = Math.round((p.phiFracs?.[r] ?? (r + 1) / (R + 1)) * N);
            const str = Math.min(1.0, Math.max(0.0, p.strengths?.[r] ?? 0.8));
            for (let i = 0; i < N; i++) {
                B[i] = this._A[(i + phi) % N] * str
                    + (i / N - 0.5) * 1e-3;   // asymmetric linear tilt seed
            }
            return B;
        });

        this._tA = new Float32Array(N);
        this._tBr = new Float32Array(N);
        // Pre-allocated snapshot buffers for B↔B cross-mod pass.
        // All B rings are copied here before any link runs so each link reads
        // the pre-cycle state — no cascade compounding between consecutive links.
        this._tBsCross = Array.from({ length: R }, () => new Float32Array(N));

        // ── Per-ring parameters (no master scaling — all directly addressable) ──
        this._thetas = new Float32Array(R);
        this._strengths = new Float32Array(R);
        this._etasA = new Float32Array(R);
        this._etasAB = new Float32Array(R);
        this._etasSq = new Float32Array(R);
        this._etasCub = new Float32Array(R);
        this._phis = new Int32Array(R);
        this._tilts = new Float32Array(R);
        this._xModThetas = new Float32Array(R);
        this._xModStrengths = new Float32Array(R);
        this._xModPhis = new Int32Array(R);   // pre-computed B–B stagger = N·(r+1)/(R+1)
        this._phiRng = new Float32Array(R);
        this._dr = [];   // [dr1, dr2] per ring (incommensurate pair)
        this._ph = [];   // [ph1, ph2] per ring (running phase accumulators)  
        this._driftDepths = new Float32Array(R);

        for (let r = 0; r < R; r++) {
            this._thetas[r] = p.thetas?.[r] ?? 0.02;
            this._strengths[r] = p.strengths?.[r] ?? 1.0;
            this._etasA[r] = p.etasA?.[r] ?? 0.0;
            this._etasAB[r] = p.etasAB?.[r] ?? 0.02;
            this._etasSq[r] = p.etasSq?.[r] ?? 0.0;
            this._etasCub[r] = p.etasCub?.[r] ?? 0.0;
            this._phis[r] = Math.round((p.phiFracs?.[r] ?? (r + 1) / (R + 1)) * N);
            this._tilts[r] = p.tilts?.[r] ?? 0.0;
            this._xModThetas[r] = p.xModThetas?.[r] ?? 0.0;
            this._xModStrengths[r] = p.xModStrengths?.[r] ?? 0.0;
            this._xModPhis[r] = Math.round(N * (r + 1) / (R + 1));
            this._phiRng[r] = Math.round((p.driftDepths?.[r] ?? 0.125) * N);
            this._driftDepths[r] = p.driftDepths?.[r] ?? 0.125;
            const db = p.driftBases?.[r] ?? 0.18;
            this._dr.push([db * 0.15, db * 0.15 * gr]);
            this._ph.push([0, 0]);
        }

        // ── Dispersion ──
        this._dispCoeff = p.dispCoeff ?? 0.0;
        this._dispStages = Math.max(1, Math.min(8, Math.round(p.dispStages ?? 4)));

        // ── Inharmonicity kernel ──
        // Precompute real-valued circular convolution kernel from polynomial
        // phase coefficients.  The kernel is the IFFT of exp(j·phase(k)),
        // which applies a k-dependent phase rotation per cycle.
        this._inharmCoeffs = (p.inharmCoeffs ?? [0, 0, 0, 0]).slice(0, 4);
        this._inharmKernel = new Float32Array(N);
        this._tInharm = new Float32Array(N);  // scratch for convolution output
        this._recomputeInharmKernel();

        // ── Global limits ──
        this._thetaClamp = p.thetaClamp ?? 0.08;
        this._tiltA = Math.min(0.99, Math.max(0.0, p.tiltA ?? 0.1));
        this._noiseFloor = p.noiseFloor ?? 1e-4;
        // Pitch correction: fractional circular shift per cycle
        this._pitchShift = p.pitchShift ?? 0;
        // Fast Xorshift32 RNG for noise injection (no GC, deterministic, audio-safe).
        this._rng = 1234567891;
    }

    // In-place all-pass chain on arr[0..N-1] (circular seeding from arr[N-1]).
    // H(z) = (c + z⁻¹) / (1 + c·z⁻¹) per stage: flat magnitude, frequency-varying
    // phase delay → inharmonic stretch (stiff-string / bell character).
    _applyDispersion(arr) {
        const c = this._dispCoeff;
        const S = this._dispStages;
        const N = this.N;
        for (let s = 0; s < S; s++) {
            let px = arr[N - 1];
            let py = arr[N - 1];
            for (let n = 0; n < N; n++) {
                const x = arr[n];
                const y = c * (x - py) + px;
                arr[n] = y;
                px = x;
                py = y;
            }
        }
    }

    // Precompute the real-valued circular convolution kernel for the
    // inharmonicity phase-shift operator.  Called once at construction
    // and whenever inharmCoeffs change.
    //
    // kernel[n] = (1/N) · Σ_{k=0}^{N/2} w[k] · cos(2πkn/N + φ[k])
    //   where w[0]=w[N/2]=1, w[k]=2 for 0<k<N/2
    //   and φ[k] = c1·k + c2·k² + c3·k³ + c4·k⁴
    _recomputeInharmKernel() {
        const N = this.N;
        const [c1, c2, c3, c4] = this._inharmCoeffs;
        const kernel = this._inharmKernel;
        const halfN = N / 2;
        const twoPiOverN = 2 * Math.PI / N;
        for (let n = 0; n < N; n++) {
            let sum = 1;  // k=0 term: cos(0 + 0) = 1, weight = 1
            for (let k = 1; k < halfN; k++) {
                const phase = c1 * k + c2 * k * k + c3 * k * k * k + c4 * k * k * k * k;
                sum += 2 * Math.cos(twoPiOverN * k * n + phase);
            }
            // k=N/2 term (weight = 1)
            const kh = halfN;
            const phH = c1 * kh + c2 * kh * kh + c3 * kh * kh * kh + c4 * kh * kh * kh * kh;
            sum += Math.cos(Math.PI * n + phH);
            kernel[n] = sum / N;
        }
    }

    // Apply inharmonicity: circular convolution with precomputed kernel.
    // O(N²) but N=128 so ∼16 K ops — negligible at audio rate.
    _applyInharm(arr) {
        const N = this.N;
        const kernel = this._inharmKernel;
        const out = this._tInharm;
        for (let n = 0; n < N; n++) {
            let sum = 0;
            for (let m = 0; m < N; m++) {
                sum += arr[m] * kernel[((n - m) % N + N) % N];
            }
            out[n] = sum;
        }
        arr.set(out);
    }

    // Fractional circular shift (bilinear interpolation).
    // output[n] = arr[(n - shift) % N], interpolated for fractional shift.
    // Matches Python _circ_roll_frac exactly.
    _circShiftFrac(arr, shift) {
        const N = this.N;
        const sMod = ((shift % N) + N) % N;   // positive modulo
        const sFloor = Math.floor(sMod);
        const frac = sMod - sFloor;
        const tmp = this._tA;                  // reuse scratch buffer [N]
        for (let n = 0; n < N; n++) {
            const i0 = ((n - sFloor) % N + N) % N;
            const i1 = ((n - sFloor - 1) % N + N) % N;
            tmp[n] = arr[i0] * (1 - frac) + arr[i1] * frac;
        }
        arr.set(tmp);
    }

    // Returns current snapshots of all 4 hidden rings as an array of Float32Arrays.
    hiddenSnapshot() {
        return this._Bs.map(B => new Float32Array(B));
    }

    advanceCycle() {
        const { _A: A, _Bs: Bs, _tA: tA, _tBr: tBr, N, R } = this;
        const Nm1 = N - 1;
        const K = this._firs[0].length;    // 5
        const hK = this._halfK;   // 2
        const thetaClamp = this._thetaClamp;
        const nfl = this._noiseFloor;

        // ── 1. Compute stride for B FIR ──
        // stride=1: conventional 5-tap FIR on adjacent samples.
        // With stride=1 the response is flat (≈1.0) for all harmonics k < N/8,
        // rolling off gently above that — no harmonic content is selectively
        // destroyed.  (stride=N/8 was a comb filter: k=8,16,24 at ∼100%,
        // k=3 at 7%, k=5 at 7% — that's why B was dominated by high harmonics.)
        //
        // The tap sliders still control the HF character of B:
        // Centre-heavy (→ impulse-like) = flat response, all harmonics couple.
        // Outer-heavy (→ box-like) = stronger HF rolloff above k≈N/8.
        const stride = 1;

        // ── 2. Chain cross-mod: B[r] ↔ B[(r+1)%R] Givens BEFORE A↔B rotations ──
        // Snapshot all B rings first so every link reads pre-cycle values.
        // Without this, B[1] modified by link(0,1) feeds directly into link(1,2)
        // on the same cycle, doubling the effective rotation angle (cascade).
        // Each link is bypassed when xModTheta[r] === 0.
        const tBsCross = this._tBsCross;
        let anyXMod = false;
        for (let r = 0; r < R; r++) { if (this._xModThetas[r] !== 0) { anyXMod = true; break; } }
        if (anyXMod) {
            for (let r = 0; r < R; r++) tBsCross[r].set(Bs[r]);
            for (let r = 0; r < R; r++) {
                const xTheta = this._xModThetas[r];
                if (xTheta === 0) continue;
                const srcR = tBsCross[r];
                const srcR1 = tBsCross[(r + 1) % R];
                const dstR = Bs[r];
                const dstR1 = Bs[(r + 1) % R];
                const cosX = Math.cos(xTheta);
                const sinX = Math.sin(xTheta);
                const xPhi = this._xModPhis[r];
                for (let n = 0; n < N; n++) {
                    const bIdx = (n + xPhi) % N;
                    const a = srcR[n];
                    const b = srcR1[bIdx];
                    dstR[n] += cosX * a + sinX * b - a;  // additive delta: dstR was set(srcR)
                    dstR1[bIdx] += -sinX * a + cosX * b - b;
                }
            }
        }

        // ── 3. Per-ring: reversed FIR on B, drift φ, Givens rotation, tilt ──
        for (let r = 0; r < R; r++) {
            const B = Bs[r];
            const th = this._thetas[r];
            const str = this._strengths[r];
            const etA = this._etasA[r];
            const etAB = this._etasAB[r];
            const etSq = this._etasSq[r];
            const etCb = this._etasCub[r];
            const phi0 = this._phis[r];
            const [dr1, dr2] = this._dr[r];
            const ph = this._ph[r];

            // 2a. Drift φ quasiperiodically (two incommensurate sinusoids per ring)
            ph[0] += dr1; ph[1] += dr2;
            const delta = Math.round(
                this._phiRng[r] * (0.7 * Math.sin(ph[0]) + 0.3 * Math.sin(ph[1])));
            const phi = ((phi0 + delta) % N + N) % N;

            // 2b. Reversed half-spec FIR on B[r] → tBr  (counter-propagation direction)
            const fir = this._firs[r];
            for (let n = 0; n < N; n++) {
                let v = fir[0] * B[n];
                for (let k = 1; k < K; k++) {
                    const off = k * stride;
                    v += fir[k] * (B[((n - off) % N + N) % N] + B[(n + off) % N]);
                }
                tBr[n] = v;
            }

            // 2c. Givens rotation with nonlinear angle modulation.
            //
            //   θ = str · (base_θ  +  η_a · aₙ              (odd  — saturation)
            //                      +  η_ab · aₙ·bₙ           (odd  — intermodulation)
            //                      +  η_sq · aₙ²             (even DC bias on θ, no sym-break alone)
            //                      +  η_cb · aₙ²·bₙ          (EVEN sym-break: (-a)²(-b)≠a²b)
            //                      +  xStr · b_next[n])       cross-ring FM: neighbor B modulates angle
            //
            //   The xStr term injects B[(r+1)%R]'s waveform directly into the rotation
            //   angle at each sample — FM sidebands appear immediately in A output.
            //   Unlike the B↔B Givens (step 2 above), this is a first-order effect.
            const BNext = Bs[(r + 1) % R];
            const xStr = this._xModStrengths[r];
            const xPhi = this._xModPhis[r];
            for (let n = 0; n < N; n++) {
                const bIdx = (n + phi) % N;
                const an = A[n];
                const bn = tBr[bIdx];
                const bNextN = BNext[(n + xPhi) % N];
                // Raw angle, then hard-clamp to [-thetaClamp, +thetaClamp].
                // The clamp prevents burst explosions when B regrows: without it,
                // large B and/or high η terms compound into rotation angles that
                // fully invert A in a handful of cycles.
                const tRaw = str * (th + etA * an + etAB * an * bn + etSq * an * an + etCb * an * an * bn
                    + xStr * bNextN);
                const t = tRaw > thetaClamp ? thetaClamp : tRaw < -thetaClamp ? -thetaClamp : tRaw;
                const cosT = Math.cos(t);
                const sinT = Math.sin(t);
                A[n] = cosT * an + sinT * bn;
                tBr[bIdx] = -sinT * an + cosT * bn;
            }
            B.set(tBr);

            // 2e. Noise injection into B: prevents B from dying to near-zero,
            //     which would cause A to stagnate (nonlinear terms all require B).
            //     Uses an in-class Xorshift32 RNG — zero allocation, deterministic.
            if (nfl > 0) {
                let rng = this._rng;
                for (let n = 0; n < N; n++) {
                    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
                    rng = rng >>> 0;
                    B[n] += nfl * (rng * 2.3283064365e-10 - 0.5);
                }
                this._rng = rng;
            }

            // 2d. Spectral tilt: one-pole LP on B only.
            //     Applied to A and B previously caused R cascaded LP filters on A
            //     per step (one per ring), causing far too much HF loss. A is
            //     affected implicitly through the Givens coupling exchange.
            const tilt = this._tilts[r];
            if (tilt > 0) {
                const b1 = 1 - tilt;
                let s = B[Nm1];
                for (let n = 0; n < N; n++) { s = b1 * B[n] + tilt * s; B[n] = s; }
            }
        }

        // ── 3. Global tilt on A (once per step, not per-ring) ──
        // Applied once after all ring interactions to give A a direct HF rolloff
        // without cascading R times. Decoupled from per-ring tilts on B.
        if (this._tiltA > 0) {
            const b1 = 1 - this._tiltA;
            let s = A[Nm1];
            for (let n = 0; n < N; n++) { s = b1 * A[n] + this._tiltA * s; A[n] = s; }
        }

        // ── 4. DC removal ──
        let dc = 0;
        for (let n = 0; n < N; n++) dc += A[n];
        dc /= N;
        for (let n = 0; n < N; n++) A[n] -= dc;

        // ── 5. Dispersion: all-pass chain stretches harmonics inharmonically ──
        if (this._dispCoeff > 0) this._applyDispersion(A);

        // ── 6. Inharmonicity: polynomial spectral phase shift ──
        if (this._inharmCoeffs.some(c => Math.abs(c) > 1e-8)) this._applyInharm(A);

        // ── 7. Pitch correction: fractional circular shift on A and all Bs ──
        const ps = this._pitchShift;
        if (Math.abs(ps) > 1e-10) {
            this._circShiftFrac(A, ps);
            for (let r = 0; r < R; r++) this._circShiftFrac(Bs[r], ps);
        }

        // ── 8. Output ──
        this.buffer.set(A);
    }

    // Serialise all 33 scalars as a JSON-compatible object.
    // Use from the browser console: copy(JSON.stringify(window._rcx.getParams(), null, 2))
    // Paste back as the `params` constructor argument after an optimisation run.
    getParams() {
        return {
            firTaps: this._firs.map(f => Array.from(f)),
            thetas: Array.from(this._thetas),
            strengths: Array.from(this._strengths),
            etasA: Array.from(this._etasA),
            etasAB: Array.from(this._etasAB),
            etasSq: Array.from(this._etasSq),
            etasCub: Array.from(this._etasCub),
            phiFracs: Array.from(this._phis).map(v => v / this.N),
            driftBases: this._dr.map(([d]) => d / 0.15),
            driftDepths: Array.from(this._driftDepths),
            tilts: Array.from(this._tilts),
            xModThetas: Array.from(this._xModThetas),
            xModStrengths: Array.from(this._xModStrengths),
            dispCoeff: this._dispCoeff,
            tiltA: this._tiltA,
            dispStages: this._dispStages,
            thetaClamp: this._thetaClamp,
            noiseFloor: this._noiseFloor,
            inharmCoeffs: this._inharmCoeffs.slice(),
            pitchShift: this._pitchShift,
        };
    }

    snapshot() { return new Float32Array(this.buffer); }
}

// ========================================================================
//  SELF-DESCRIBING SYNTH REGISTRY
//
//  Each class declares:
//    ClassName.baseParams  — array of param descriptors
//    ClassName.createFromOpts(buffer, opts, context) — static factory
//
//  Descriptor format:
//    { key, label, min, max, default, step, format?, isInt? }
//
//  At the bottom, SYNTH_REGISTRY_ARRAY (ordered) and SYNTH_REGISTRY (keyed)
//  are built automatically.  Adding a new param to baseParams instantly
//  makes it appear in the UI — no HTML or ui.js changes needed.
//
//  Waveguide synths are registered for the dropdown + UI but flagged with
//  isWaveguide=true so driver.js keeps its existing specialised factory.
// ========================================================================

// Shared extra params shown for double-ended (open) algorithm variants.
const _OPEN_END_PARAMS = [
    { key: 'clampL', label: 'Clamp L', min: 0, max: 1, default: 0, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'clampR', label: 'Clamp R', min: 0, max: 1, default: 0, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'taper', label: 'Taper %', min: 1, max: 50, default: 5, step: 1, format: v => v + '%', isInt: true },
];

// ── ManifoldBloom ────────────────────────────────────────────────────────────
ManifoldBloom.group = 'Manifold Bloom';
ManifoldBloom.baseParams = [
    { key: 'alpha', label: 'Alpha', min: 0, max: 5, default: 0.18, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'asymmetry', label: 'Asymmetry', min: 0, max: 1, default: 0.14, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'costWeight', label: 'Cost Weight', min: 0, max: 0.5, default: 0.411, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'coupling', label: 'Coupling', min: 0, max: 1, default: 0.746, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'resTension', label: 'Res. Tension', min: 0.001, max: 10, default: 1.95, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'resDamping', label: 'Res. Damping', min: 0, max: 0.1, default: 0.001, step: 0.0001, format: v => Number(v).toFixed(4) },
    { key: 'damping', label: 'Damping', min: 0, max: 5, default: 3.2, step: 0.1, format: v => Number(v).toFixed(1) },
    { key: 'sustain', label: 'Sustain', min: 0.95, max: 0.99999, default: 0.9999, step: 0.00001, format: v => Number(v).toFixed(5) },
    { key: 'stepsPerCycle', label: 'Steps/Cycle', min: 1, max: 32, default: 8, step: 1, isInt: true },
    { key: 'morphRate', label: 'Morph Rate', min: 0, max: 2, default: 0.5, step: 0.01, format: v => Number(v).toFixed(2) },
];
ManifoldBloom.createFromOpts = function (buffer, opts, { step }) {
    return new ManifoldBloom(buffer, {
        alpha: clamp(opts.alpha ?? 0.18, 0, 5),
        asymmetry: clamp(opts.asymmetry ?? 0.14, 0, 1),
        costWeight: clamp(opts.costWeight ?? 0.411, 0, 0.5),
        coupling: clamp(opts.coupling ?? 0.746, 0, 1),
        resTension: clamp(opts.resTension ?? 1.95, 0.001, 10),
        resDamping: clamp(opts.resDamping ?? 0.001, 0, 0.1),
        damping: clamp(opts.damping ?? 3.2, 0, 5),
        sustain: clamp(opts.sustain ?? 0.9999, 0.95, 0.99999),
        stepsPerCycle: Math.max(1, Math.round(opts.stepsPerCycle ?? 8)),
        morphRate: clamp(opts.morphRate ?? 0.5, 0, 2),
        step,
    });
};

// ── ManifoldBloomFast ────────────────────────────────────────────────────────
ManifoldBloomFast.group = 'Manifold Bloom';
ManifoldBloomFast.baseParams = [
    { key: 'alpha', label: 'Alpha', min: 0, max: 5, default: 0.18, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'asymmetry', label: 'Asymmetry', min: 0, max: 1, default: 0.14, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'costWeight', label: 'Cost Weight', min: 0, max: 0.5, default: 0.411, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'coupling', label: 'Coupling', min: 0, max: 1, default: 0.746, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'resTension', label: 'Res. Tension', min: 0.001, max: 10, default: 1.95, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'resDamping', label: 'Res. Damping', min: 0, max: 0.1, default: 0.001, step: 0.0001, format: v => Number(v).toFixed(4) },
    { key: 'damping', label: 'Damping', min: 0, max: 5, default: 3.2, step: 0.1, format: v => Number(v).toFixed(1) },
    { key: 'sustain', label: 'Sustain', min: 0.95, max: 0.99999, default: 0.9999, step: 0.00001, format: v => Number(v).toFixed(5) },
    { key: 'morphRate', label: 'Morph Rate', min: 0, max: 2, default: 0.5, step: 0.01, format: v => Number(v).toFixed(2) },
];
ManifoldBloomFast.createFromOpts = function (buffer, opts, { step }) {
    return new ManifoldBloomFast(buffer, {
        alpha: clamp(opts.alpha ?? 0.18, 0, 5),
        asymmetry: clamp(opts.asymmetry ?? 0.14, 0, 1),
        costWeight: clamp(opts.costWeight ?? 0.411, 0, 0.5),
        coupling: clamp(opts.coupling ?? 0.746, 0, 1),
        resTension: clamp(opts.resTension ?? 1.95, 0.001, 10),
        resDamping: clamp(opts.resDamping ?? 0.001, 0, 0.1),
        damping: clamp(opts.damping ?? 3.2, 0, 5),
        sustain: clamp(opts.sustain ?? 0.9999, 0.95, 0.99999),
        morphRate: clamp(opts.morphRate ?? 0.5, 0, 2),
        step,
    });
};

// ── ManifoldBloomV2 ──────────────────────────────────────────────────────────
ManifoldBloomV2.group = 'Manifold Bloom';
ManifoldBloomV2.baseParams = [
    { key: 'alpha', label: 'Alpha', min: 0, max: 5, default: 0.18, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'asymmetry', label: 'Asymmetry', min: 0, max: 1, default: 0.14, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'costWeight', label: 'Cost Weight', min: 0, max: 0.5, default: 0.411, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'coupling', label: 'Coupling', min: 0, max: 1, default: 0.746, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'resTension', label: 'Res. Tension', min: 0.001, max: 10, default: 1.95, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'resDamping', label: 'Res. Damping', min: 0, max: 0.1, default: 0.001, step: 0.0001, format: v => Number(v).toFixed(4) },
    { key: 'damping', label: 'Damping', min: 0, max: 5, default: 3.2, step: 0.1, format: v => Number(v).toFixed(1) },
    { key: 'sustain', label: 'Sustain', min: 0.95, max: 0.99999, default: 0.9999, step: 0.00001, format: v => Number(v).toFixed(5) },
    { key: 'morphRate', label: 'Morph Rate', min: 0, max: 2, default: 0.5, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'selectivity', label: 'Selectivity', min: 0, max: 1.5, default: 0.7, step: 0.01, format: v => Number(v).toFixed(2) },
];
ManifoldBloomV2.createFromOpts = function (buffer, opts, { step }) {
    return new ManifoldBloomV2(buffer, {
        alpha: clamp(opts.alpha ?? 0.18, 0, 5),
        asymmetry: clamp(opts.asymmetry ?? 0.14, 0, 1),
        costWeight: clamp(opts.costWeight ?? 0.411, 0, 0.5),
        coupling: clamp(opts.coupling ?? 0.746, 0, 1),
        resTension: clamp(opts.resTension ?? 1.95, 0.001, 10),
        resDamping: clamp(opts.resDamping ?? 0.001, 0, 0.1),
        damping: clamp(opts.damping ?? 3.2, 0, 5),
        sustain: clamp(opts.sustain ?? 0.9999, 0.95, 0.99999),
        morphRate: clamp(opts.morphRate ?? 0.5, 0, 2),
        selectivity: clamp(opts.selectivity ?? 0.7, 0, 1.5),
        step,
    });
};

// ── ManifoldBloomV3 ──────────────────────────────────────────────────────────
ManifoldBloomV3.group = 'Manifold Bloom';
ManifoldBloomV3.baseParams = [
    { key: 'crushRate', label: 'Crush Rate', min: 0, max: 0.499, default: 0.1, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'costWeight', label: 'Cost Weight', min: 0, max: 0.5, default: 0.02, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'damping', label: 'Damping', min: 0, max: 5, default: 3.2, step: 0.1, format: v => Number(v).toFixed(1) },
    { key: 'sustain', label: 'Sustain', min: 0.95, max: 0.99999, default: 0.9999, step: 0.00001, format: v => Number(v).toFixed(5) },
];
ManifoldBloomV3.createFromOpts = function (buffer, opts, { step }) {
    return new ManifoldBloomV3(buffer, {
        crushRate: clamp(opts.crushRate ?? 0.1, 0, 0.499),
        costWeight: clamp(opts.costWeight ?? 0.02, 0, 0.5),
        damping: clamp(opts.damping ?? 3.2, 0, 5),
        sustain: clamp(opts.sustain ?? 0.9999, 0.95, 0.99999),
        step,
    });
};

// ── ManifoldBloomV4 ──────────────────────────────────────────────────────────
ManifoldBloomV4.group = 'Manifold Bloom';
ManifoldBloomV4.baseParams = [
    { key: 'alpha', label: 'Alpha', min: 0, max: 5, default: 0.18, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'coupling', label: 'Coupling', min: 0, max: 1, default: 0.5, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'concentration', label: 'Concentration', min: 0, max: 0.95, default: 0.15, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'resTension', label: 'Res. Tension', min: 0.001, max: 10, default: 1.95, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'morphRate', label: 'Morph Rate', min: 0, max: 2, default: 0.5, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'selectivity', label: 'Selectivity', min: 0, max: 1.5, default: 0.7, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'damping', label: 'Damping', min: 0, max: 5, default: 1.0, step: 0.1, format: v => Number(v).toFixed(1) },
];
ManifoldBloomV4.createFromOpts = function (buffer, opts, { step }) {
    return new ManifoldBloomV4(buffer, {
        alpha: clamp(opts.alpha ?? 0.18, 0, 5),
        asymmetry: clamp(opts.asymmetry ?? 0.0, 0, 1),
        coupling: clamp(opts.coupling ?? 0.5, 0, 1),
        resTension: clamp(opts.resTension ?? 1.95, 0.001, 10),
        damping: clamp(opts.damping ?? 1.0, 0, 5),
        morphRate: clamp(opts.morphRate ?? 0.5, 0, 2),
        selectivity: clamp(opts.selectivity ?? 0.7, 0, 1.5),
        concentration: clamp(opts.concentration ?? 0.15, 0, 0.95),
        step,
    });
};

// ── RingCoupler ──────────────────────────────────────────────────────────────
RingCoupler.group = 'Ring Coupler';
RingCoupler.baseParams = [
    { key: 'coupling', label: 'Coupling', min: 0, max: Math.PI / 4, default: 0.1, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'asymmetry', label: 'Asymmetry', min: -1, max: 1, default: 0.1, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'strength', label: 'Strength', min: 0, max: 0.1, default: 0.05, step: 0.001, format: v => Number(v).toFixed(3) },
    { key: 'morphRate', label: 'Morph Rate', min: 0, max: 2, default: 0.5, step: 0.01, format: v => Number(v).toFixed(2) },
    { key: 'damping', label: 'Damping', min: 0, max: 5, default: 0.0, step: 0.1, format: v => Number(v).toFixed(1) },
];
RingCoupler.createFromOpts = function (buffer, opts, { step }) {
    return new RingCoupler(buffer, {
        coupling: clamp(opts.coupling ?? 0.1, 0, Math.PI / 4),
        asymmetry: clamp(opts.asymmetry ?? 0.1, -1, 1),
        strength: clamp(opts.strength ?? 0.05, 0, 0.1),
        morphRate: clamp(opts.morphRate ?? 0.5, 0, 2),
        damping: clamp(opts.damping ?? 0.0, 0, 5),
        step,
    });
};

// ── RingCouplerX ─────────────────────────────────────────────────────────────
// Complex array-based params: uses static buildUI() instead of baseParams.
RingCouplerX.group = 'Ring Coupler';
RingCouplerX.baseParams = [];  // populated below after buildUI helper is defined
RingCouplerX.hasBuildUI = true;
RingCouplerX.buildUI = function (container, currentOpts, onChangeCb) {
    const dp = RingCouplerX.defaultParams();
    const g = (key, fb) => (currentOpts[key] != null ? currentOpts[key] : fb);

    const sections = [
        {
            title: 'Global', params: [
                { key: 'rcxDispCoeff', label: 'Disp. Coeff', min: 0, max: 0.5, step: 0.01, default: dp.dispCoeff, format: v => Number(v).toFixed(2) },
                { key: 'rcxTiltA', label: 'Tilt A', min: 0, max: 0.9, step: 0.01, default: dp.tiltA, format: v => Number(v).toFixed(2) },
                { key: 'rcxDispStages', label: 'Disp. Stages', min: 1, max: 16, step: 1, default: dp.dispStages, format: v => parseInt(v).toString(), isInt: true },
                { key: 'rcxThetaClamp', label: 'Theta Clamp', min: 0, max: 1, step: 0.001, default: dp.thetaClamp, format: v => Number(v).toFixed(3) },
                { key: 'rcxNoiseFloor', label: 'Noise Floor', min: 0, max: 0.01, step: 0.0001, default: dp.noiseFloor, format: v => Number(v).toFixed(4) },
                { key: 'rcxFir0', label: 'FIR 0', min: 0, max: 1, step: 0.01, default: dp.firTaps[0][0], format: v => Number(v).toFixed(2) },
                { key: 'rcxFir1', label: 'FIR 1', min: 0, max: 1, step: 0.01, default: dp.firTaps[0][1], format: v => Number(v).toFixed(2) },
                { key: 'rcxFir2', label: 'FIR 2', min: 0, max: 1, step: 0.01, default: dp.firTaps[0][2], format: v => Number(v).toFixed(2) },
                { key: 'rcxFir3', label: 'FIR 3', min: 0, max: 1, step: 0.01, default: dp.firTaps[0][3], format: v => Number(v).toFixed(2) },
                { key: 'rcxFir4', label: 'FIR 4', min: 0, max: 1, step: 0.01, default: dp.firTaps[0][4], format: v => Number(v).toFixed(2) },
            ]
        },
    ];
    for (let r = 0; r < 4; r++) {
        sections.push({
            title: `Ring ${r + 1}`, params: [
                { key: `rcxR${r + 1}Theta`, label: 'Theta', min: 0, max: 1, step: 0.001, default: dp.thetas[r], format: v => Number(v).toFixed(3) },
                { key: `rcxR${r + 1}Strength`, label: 'Strength', min: 0, max: 1, step: 0.01, default: dp.strengths[r], format: v => Number(v).toFixed(2) },
                { key: `rcxR${r + 1}EtaA`, label: 'Eta A', min: -0.2, max: 0.2, step: 0.001, default: dp.etasA[r], format: v => Number(v).toFixed(3) },
                { key: `rcxR${r + 1}EtaAB`, label: 'Eta AB', min: -0.2, max: 0.2, step: 0.001, default: dp.etasAB[r], format: v => Number(v).toFixed(2) },
                { key: `rcxR${r + 1}EtaSq`, label: 'Eta Sq', min: -0.2, max: 0.2, step: 0.001, default: dp.etasSq[r], format: v => Number(v).toFixed(2) },
                { key: `rcxR${r + 1}EtaCub`, label: 'Eta Cub', min: -0.2, max: 0.2, step: 0.001, default: dp.etasCub[r], format: v => Number(v).toFixed(2) },
                { key: `rcxR${r + 1}Phi`, label: 'Phi', min: 0, max: 1, step: 0.01, default: dp.phiFracs[r], format: v => Number(v).toFixed(2) },
                { key: `rcxR${r + 1}Drift`, label: 'Drift', min: 0, max: 1, step: 0.01, default: dp.driftBases[r], format: v => Number(v).toFixed(2) },
                { key: `rcxR${r + 1}DriftDepth`, label: 'Drift Depth', min: 0, max: 0.5, step: 0.001, default: dp.driftDepths[r], format: v => Number(v).toFixed(3) },
                { key: `rcxR${r + 1}Tilt`, label: 'Tilt', min: 0, max: 0.99, step: 0.01, default: dp.tilts[r], format: v => Number(v).toFixed(2) },
                { key: `rcxR${r + 1}XModTheta`, label: 'XMod Theta', min: 0, max: 1, step: 0.001, default: dp.xModThetas[r], format: v => Number(v).toFixed(3) },
                { key: `rcxR${r + 1}XModStr`, label: 'XMod Str', min: 0, max: 1, step: 0.01, default: dp.xModStrengths[r], format: v => Number(v).toFixed(2) },
            ]
        });
    }

    for (const sec of sections) {
        const hdr = document.createElement('div');
        hdr.className = 'dynSection';
        hdr.textContent = sec.title;
        container.appendChild(hdr);
        for (const p of sec.params) {
            const cg = document.createElement('div');
            cg.className = 'cg';
            const lbl = document.createElement('label');
            lbl.textContent = p.label;
            const sl = document.createElement('input');
            sl.type = 'range'; sl.id = 'sl_' + p.key;
            sl.min = p.min; sl.max = p.max; sl.step = p.step;
            sl.value = g(p.key, p.default);
            const val = document.createElement('span');
            val.className = 'val'; val.id = 'val_' + p.key;
            val.textContent = p.format ? p.format(sl.value) : sl.value;
            sl.addEventListener('input', () => {
                val.textContent = p.format ? p.format(Number(sl.value)) : sl.value;
                if (onChangeCb) onChangeCb();
            });
            cg.appendChild(lbl); cg.appendChild(sl); cg.appendChild(val);
            container.appendChild(cg);
        }
    }
};
RingCouplerX.createFromOpts = function (buffer, opts, { step }) {
    const dp = RingCouplerX.defaultParams();
    const g = (key, fb) => (opts[key] != null ? parseFloat(opts[key]) : fb);
    const params = {
        firTaps: [0, 1, 2, 3, 4].map(k => g(`rcxFir${k}`, dp.firTaps[0][k])),
        thetas: [1, 2, 3, 4].map(r => g(`rcxR${r}Theta`, dp.thetas[r - 1])),
        strengths: [1, 2, 3, 4].map(r => g(`rcxR${r}Strength`, dp.strengths[r - 1])),
        etasA: [1, 2, 3, 4].map(r => g(`rcxR${r}EtaA`, dp.etasA[r - 1])),
        etasAB: [1, 2, 3, 4].map(r => g(`rcxR${r}EtaAB`, dp.etasAB[r - 1])),
        etasSq: [1, 2, 3, 4].map(r => g(`rcxR${r}EtaSq`, dp.etasSq[r - 1])),
        etasCub: [1, 2, 3, 4].map(r => g(`rcxR${r}EtaCub`, dp.etasCub[r - 1])),
        phiFracs: [1, 2, 3, 4].map(r => g(`rcxR${r}Phi`, dp.phiFracs[r - 1])),
        driftBases: [1, 2, 3, 4].map(r => g(`rcxR${r}Drift`, dp.driftBases[r - 1])),
        driftDepths: [1, 2, 3, 4].map(r => g(`rcxR${r}DriftDepth`, dp.driftDepths[r - 1])),
        tilts: [1, 2, 3, 4].map(r => g(`rcxR${r}Tilt`, dp.tilts[r - 1])),
        xModThetas: [1, 2, 3, 4].map(r => g(`rcxR${r}XModTheta`, dp.xModThetas[r - 1])),
        xModStrengths: [1, 2, 3, 4].map(r => g(`rcxR${r}XModStr`, dp.xModStrengths[r - 1])),
        dispCoeff: g('rcxDispCoeff', dp.dispCoeff),
        tiltA: g('rcxTiltA', dp.tiltA),
        dispStages: g('rcxDispStages', dp.dispStages),
        thetaClamp: g('rcxThetaClamp', dp.thetaClamp),
        noiseFloor: g('rcxNoiseFloor', dp.noiseFloor),
    };
    return new RingCouplerX(buffer, { params, step });
};

// ── SYNTH_REGISTRY ────────────────────────────────────────────────────────────
// SYNTH_REGISTRY_ARRAY: ordered list of entries (preserves dropdown order).
// SYNTH_REGISTRY: keyed by algorithm id for O(1) lookup.
//
// To add a new synth: implement the class, add its static descriptors above,
// then call _reg() here.  Everything else is automatic.

const SYNTH_REGISTRY_ARRAY = [];
const SYNTH_REGISTRY = {};

function _reg(id, label, cls, params, extraFlags) {
    const entry = {
        id,
        label,
        cls,
        params,
        isDoubleEnded: !!(extraFlags && extraFlags.isDoubleEnded),
        group: (extraFlags && extraFlags.group) || cls.group || '',
    };
    SYNTH_REGISTRY_ARRAY.push(entry);
    SYNTH_REGISTRY[id] = entry;
}


// Lagrangian Acoustics
_reg('lagrangian', 'Lagrangian Acoustics – Circular', LagrangianAcoustics, LagrangianAcoustics.baseParams);
_reg('lagrangian_open', 'Lagrangian Acoustics – Open', LagrangianAcoustics,
    [...LagrangianAcoustics.baseParams, ..._OPEN_END_PARAMS], { isDoubleEnded: true });

// Lagrangian Acoustics — SWN port match (pitch-adaptive buffer + stabilizers)
_reg('lagrangian_swn', 'Lagrangian Acoustics – SWN-Match',
    LagrangianAcousticsSWN, LagrangianAcousticsSWN.baseParams);

// Waveguide
_reg('waveguide', 'Waveguide – Circular', WaveguideSynth,
    [...WaveguideSynth._sharedParams, ...WaveguideSynth._circularParams]);
_reg('waveguide_fixed', 'Waveguide – Fixed Ends', WaveguideSynth,
    [...WaveguideSynth._sharedParams, ...WaveguideSynth._fixedParams],
    { isDoubleEnded: true });

// Nonlinear Acoustics (Burgers)
_reg('burgers', 'Nonlinear Acoustics – Circular', AcousticsSynth, AcousticsSynth.baseParams);
_reg('burgers_open', 'Nonlinear Acoustics – Open Ends', AcousticsSynth,
    [...AcousticsSynth.baseParams, ..._OPEN_END_PARAMS], { isDoubleEnded: true });

// Maxwell 1-D FDTD
_reg('maxwell', 'Maxwell 1-D – Circular', MaxwellSynth, MaxwellSynth.baseParams);
_reg('maxwell_open', 'Maxwell 1-D – PEC Ends', MaxwellSynth,
    [...MaxwellSynth.baseParams, ..._OPEN_END_PARAMS], { isDoubleEnded: true });

// Simple Nonlinear
_reg('nonlinear', 'Simple Nonlinear', SimpleNonlinearSynth, SimpleNonlinearSynth.baseParams);

// Custom (user-defined skeleton)
_reg('custom', 'Custom', CustomSynth, CustomSynth.baseParams);

// Gradient-flow candidates
_reg('even_gradient', 'Even Gradient', EvenGradientSynth, EvenGradientSynth.baseParams);



// Manifold Bloom family
_reg('mbloom', 'Manifold Bloom', ManifoldBloom, ManifoldBloom.baseParams);
_reg('mbloom_fast', 'Manifold Bloom (Fast)', ManifoldBloomFast, ManifoldBloomFast.baseParams);
_reg('mbloom_v2', 'Manifold Bloom (V2)', ManifoldBloomV2, ManifoldBloomV2.baseParams);
_reg('mbloom_v3', 'Peak Cascade (V3)', ManifoldBloomV3, ManifoldBloomV3.baseParams);
_reg('mbloom_v4', 'Spotlight (V4)', ManifoldBloomV4, ManifoldBloomV4.baseParams);

// Ring Couplers
_reg('ring_coupler', 'Ring Coupler', RingCoupler, RingCoupler.baseParams);
_reg('ring_coupler_x', 'Ring Coupler X (multi-ring)', RingCouplerX, RingCouplerX.baseParams);
