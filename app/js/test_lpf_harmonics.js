#!/usr/bin/env node
// Test: does the LPF in LagrangianAcoustics introduce harmonics on a pure sine?

const N = 1024;

// --- Butterworth biquad coefficients ---
function biquadCoeffs(cutoff, N) {
    const omega0 = 2 * Math.PI * cutoff / N;
    const Om = Math.tan(omega0 / 2);
    const Q = 1 / Math.SQRT2;
    const d = 1 + Om / Q + Om * Om;
    return {
        b0: (Om * Om) / d,
        b1: 2 * (Om * Om) / d,
        b2: (Om * Om) / d,
        a1: 2 * (Om * Om - 1) / d,
        a2: (1 - Om / Q + Om * Om) / d,
    };
}

// --- Cayley-Hamilton circular steady-state solve + filter ---
function lpfCircular(arr, cutoff, mix) {
    if (mix <= 0) return;
    const n = arr.length;
    const { b0, b1, b2, a1, a2 } = biquadCoeffs(cutoff, n);

    // Cayley-Hamilton solve for circular steady-state
    let fs1 = 0, fs2 = 0;
    let p = 1, q = 0;
    for (let i = 0; i < n; i++) {
        const x = arr[i];
        const y = b0 * x + fs1;
        fs1 = b1 * x - a1 * y + fs2;
        fs2 = b2 * x - a2 * y;
        const pN = -a2 * q;
        const qN = p - a1 * q;
        p = pN;
        q = qN;
    }
    const m00 = 1 - p + a1 * q;
    const m01 = -q;
    const m10 = a2 * q;
    const m11 = 1 - p;
    const det = m00 * m11 - m01 * m10;
    let s1, s2;
    if (Math.abs(det) > 1e-15) {
        s1 = (m11 * fs1 - m01 * fs2) / det;
        s2 = (-m10 * fs1 + m00 * fs2) / det;
    } else {
        s1 = 0; s2 = 0;
    }

    // Filter with wet/dry mix
    const dry = 1 - mix;
    for (let i = 0; i < n; i++) {
        const x = arr[i];
        const y = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
        arr[i] = dry * x + mix * y;
    }
    return { s1, s2 };
}

// --- DFT magnitude spectrum (first 64 bins) ---
function fftMag(buf, numBins) {
    const N = buf.length;
    const mag = new Float64Array(numBins);
    for (let k = 0; k < numBins; k++) {
        let re = 0, im = 0;
        for (let n = 0; n < N; n++) {
            const angle = -2 * Math.PI * k * n / N;
            re += buf[n] * Math.cos(angle);
            im += buf[n] * Math.sin(angle);
        }
        mag[k] = Math.sqrt(re * re + im * im) / N;
    }
    return mag;
}

// --- Create pure sine at harmonic 1 ---
function makeSine(N, harmonic) {
    const buf = new Float64Array(N);
    for (let i = 0; i < N; i++) {
        buf[i] = Math.sin(2 * Math.PI * harmonic * i / N);
    }
    return buf;
}

// ======================================================================
// Test 1: Single application of LPF on pure sine
// ======================================================================
console.log('=== Test 1: Single LPF application on pure sine ===');
{
    const buf = makeSine(N, 1);
    const magBefore = fftMag(buf, 32);
    console.log('Before LPF:');
    for (let k = 0; k < 8; k++) {
        console.log(`  bin ${k}: ${magBefore[k].toExponential(4)}`);
    }

    lpfCircular(buf, 12, 0.14);  // cutoff=12, mix=0.14 (dampingP default)
    const magAfter = fftMag(buf, 32);
    console.log('After 1 LPF call:');
    for (let k = 0; k < 8; k++) {
        const db = magAfter[k] > 1e-15 ? 20 * Math.log10(magAfter[k] / 0.5) : -999;
        console.log(`  bin ${k}: ${magAfter[k].toExponential(4)} (${db.toFixed(1)} dB)`);
    }
    // Check for spurious harmonics
    const fundamental = magAfter[1];
    let maxSpurious = 0;
    for (let k = 2; k < 32; k++) {
        if (magAfter[k] > maxSpurious) maxSpurious = magAfter[k];
    }
    const sfdr = 20 * Math.log10(maxSpurious / fundamental);
    console.log(`SFDR (spurious-free dynamic range): ${sfdr.toFixed(1)} dB`);
}

// ======================================================================
// Test 2: Repeated LPF application (simulating many advanceCycle calls)
//         with persistent state (as currently coded)
// ======================================================================
console.log('\n=== Test 2: 100 repeated LPF calls with persistent state ===');
{
    const buf = makeSine(N, 1);
    // First call: solve exact circular state
    let state = lpfCircular(buf, 12, 0.14);

    // Subsequent calls: reuse persisted state (DON'T re-solve)
    const { b0, b1, b2, a1, a2 } = biquadCoeffs(12, N);
    const mix = 0.14;
    const dry = 1 - mix;

    for (let cycle = 1; cycle < 100; cycle++) {
        let { s1, s2 } = state;
        for (let i = 0; i < N; i++) {
            const x = buf[i];
            const y = b0 * x + s1;
            s1 = b1 * x - a1 * y + s2;
            s2 = b2 * x - a2 * y;
            buf[i] = dry * x + mix * y;
        }
        state = { s1, s2 };
    }

    const mag = fftMag(buf, 32);
    console.log('After 100 cycles with persistent state (no re-solve):');
    for (let k = 0; k < 8; k++) {
        const db = mag[k] > 1e-15 ? 20 * Math.log10(mag[k] / 0.5) : -999;
        console.log(`  bin ${k}: ${mag[k].toExponential(4)} (${db.toFixed(1)} dB)`);
    }
    const fundamental = mag[1];
    let maxSpurious = 0;
    for (let k = 2; k < 32; k++) {
        if (mag[k] > maxSpurious) maxSpurious = mag[k];
    }
    const sfdr = 20 * Math.log10(maxSpurious / fundamental);
    console.log(`SFDR: ${sfdr.toFixed(1)} dB`);
}

// ======================================================================
// Test 3: Repeated LPF with re-solve every call
// ======================================================================
console.log('\n=== Test 3: 100 repeated LPF calls with re-solve every call ===');
{
    const buf = makeSine(N, 1);
    for (let cycle = 0; cycle < 100; cycle++) {
        lpfCircular(buf, 12, 0.14);
    }

    const mag = fftMag(buf, 32);
    console.log('After 100 cycles with re-solve every call:');
    for (let k = 0; k < 8; k++) {
        const db = mag[k] > 1e-15 ? 20 * Math.log10(mag[k] / 0.5) : -999;
        console.log(`  bin ${k}: ${mag[k].toExponential(4)} (${db.toFixed(1)} dB)`);
    }
    const fundamental = mag[1];
    let maxSpurious = 0;
    for (let k = 2; k < 32; k++) {
        if (mag[k] > maxSpurious) maxSpurious = mag[k];
    }
    const sfdr = 20 * Math.log10(maxSpurious / fundamental);
    console.log(`SFDR: ${sfdr.toFixed(1)} dB`);
}

// ======================================================================
// Test 4: Simulate full advanceCycle with alpha=0, nonlinearity=0
//         Using persistent state (no re-solve after first call)
// ======================================================================
console.log('\n=== Test 4: Full advanceCycle simulation (alpha=0, nl=0) ===');
{
    const q = makeSine(N, 1);
    const v = new Float64Array(N);  // all zeros
    const dt = 0.25;
    const lpfCutoff = 12;
    const dampingP = 0.14;
    const dampingV = 0.5;
    const sustain = 0.9999;
    const stepsPerCycle = 1;
    const alpha = 0;
    const nonlinearity = 0;

    const { b0, b1, b2, a1, a2 } = biquadCoeffs(lpfCutoff, N);
    let stateQ = null;  // will be solved on first call
    let stateV = null;

    for (let cycle = 0; cycle < 100; cycle++) {
        const dtt = dt + q[0] * dt / 2;

        for (let step = 0; step < stepsPerCycle; step++) {
            // Springs (alpha=0 → no force)
            // Nonlinearity=0 → no force
            // Position update: q[n] += dtt * v[n] (v=0 on first cycles)
            for (let n = 0; n < N; n++) {
                q[n] += dtt * v[n];
            }

            // LPF on velocity
            if (dampingV > 0) {
                if (!stateV) {
                    stateV = lpfCircularInPlace(v, b0, b1, b2, a1, a2, dampingV, null);
                } else {
                    stateV = lpfCircularInPlace(v, b0, b1, b2, a1, a2, dampingV, stateV);
                }
            }

            // LPF on position
            if (dampingP > 0) {
                if (!stateQ) {
                    stateQ = lpfCircularInPlace(q, b0, b1, b2, a1, a2, dampingP, null);
                } else {
                    stateQ = lpfCircularInPlace(q, b0, b1, b2, a1, a2, dampingP, stateQ);
                }
            }

            // Sustain
            if (sustain < 1.0) {
                for (let n = 0; n < N; n++) v[n] *= sustain;
            }
        }

        if (cycle === 0 || cycle === 9 || cycle === 99) {
            const mag = fftMag(q, 32);
            const fundamental = mag[1];
            let maxSpurious = 0;
            let worstBin = 0;
            for (let k = 2; k < 32; k++) {
                if (mag[k] > maxSpurious) { maxSpurious = mag[k]; worstBin = k; }
            }
            const sfdr = 20 * Math.log10(maxSpurious / fundamental);
            console.log(`Cycle ${cycle}: fund=${fundamental.toExponential(4)}, SFDR=${sfdr.toFixed(1)} dB (worst bin ${worstBin})`);
        }
    }
}

// Helper: run filter on array with optional pre-solved state
function lpfCircularInPlace(arr, b0, b1, b2, a1, a2, mix, prevState) {
    const n = arr.length;
    let s1, s2;

    if (!prevState) {
        // Cayley-Hamilton solve
        let fs1 = 0, fs2 = 0;
        let p = 1, q = 0;
        for (let i = 0; i < n; i++) {
            const x = arr[i];
            const y = b0 * x + fs1;
            fs1 = b1 * x - a1 * y + fs2;
            fs2 = b2 * x - a2 * y;
            const pN = -a2 * q;
            const qN = p - a1 * q;
            p = pN;
            q = qN;
        }
        const m00 = 1 - p + a1 * q;
        const m01 = -q;
        const m10 = a2 * q;
        const m11 = 1 - p;
        const det = m00 * m11 - m01 * m10;
        if (Math.abs(det) > 1e-15) {
            s1 = (m11 * fs1 - m01 * fs2) / det;
            s2 = (-m10 * fs1 + m00 * fs2) / det;
        } else {
            s1 = 0; s2 = 0;
        }
    } else {
        s1 = prevState.s1;
        s2 = prevState.s2;
    }

    const dry = 1 - mix;
    for (let i = 0; i < n; i++) {
        const x = arr[i];
        const y = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
        arr[i] = dry * x + mix * y;
    }
    return { s1, s2 };
}
