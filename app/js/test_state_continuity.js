#!/usr/bin/env node
// Diagnose: where exactly does the seam error originate?
// Compare persisted state vs re-solved state after one cycle.

const N = 1024;

function biquadCoeffs(cutoff) {
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

// Solve exact circular initial state for buffer
function solveCircularState(arr, b0, b1, b2, a1, a2) {
    const n = arr.length;
    let fs1 = 0, fs2 = 0;
    let p = 1, q = 0;
    for (let i = 0; i < n; i++) {
        const x = arr[i];
        const y = b0 * x + fs1;
        fs1 = b1 * x - a1 * y + fs2;
        fs2 = b2 * x - a2 * y;
        const pN = -a2 * q;
        const qN = p - a1 * q;
        p = pN; q = qN;
    }
    const m00 = 1 - p + a1 * q, m01 = -q;
    const m10 = a2 * q, m11 = 1 - p;
    const det = m00 * m11 - m01 * m10;
    return {
        s1: (m11 * fs1 - m01 * fs2) / det,
        s2: (-m10 * fs1 + m00 * fs2) / det,
    };
}

// Run filter from given state, return output WITHOUT modifying buffer
function filterReadOnly(arr, b0, b1, b2, a1, a2, s1, s2) {
    const out = new Float64Array(arr.length);
    for (let i = 0; i < arr.length; i++) {
        const x = arr[i];
        const y = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
        out[i] = y;
    }
    return { out, s1, s2 };
}

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

const cutoff = 12;
const mix = 0.14;
const dry = 1 - mix;
const { b0, b1, b2, a1, a2 } = biquadCoeffs(cutoff);

// Pole magnitude
const poleMag = Math.sqrt(a2);
console.log(`Pole magnitude: ${poleMag.toFixed(6)}`);
console.log(`|pole|^N = ${Math.pow(poleMag, N).toExponential(4)}`);
console.log(`|pole|^100 = ${Math.pow(poleMag, 100).toExponential(4)}`);
console.log();

// Start with pure sine
const buf = new Float64Array(N);
for (let i = 0; i < N; i++) buf[i] = Math.sin(2 * Math.PI * i / N);

// Cycle 0: exact solve + filter + mix
const s0 = solveCircularState(buf, b0, b1, b2, a1, a2);
console.log(`Cycle 0 exact state: s1=${s0.s1.toExponential(8)}, s2=${s0.s2.toExponential(8)}`);

// Apply filter+mix
let s1 = s0.s1, s2 = s0.s2;
for (let i = 0; i < N; i++) {
    const x = buf[i];
    const y = b0 * x + s1;
    s1 = b1 * x - a1 * y + s2;
    s2 = b2 * x - a2 * y;
    buf[i] = dry * x + mix * y;
}
console.log(`State after cycle 0: s1=${s1.toExponential(8)}, s2=${s2.toExponential(8)}`);
console.log(`State return error: |Δs1|=${Math.abs(s1 - s0.s1).toExponential(4)}, |Δs2|=${Math.abs(s2 - s0.s2).toExponential(4)}`);

// What is the correct state for the NEW buffer?
const s0_new = solveCircularState(buf, b0, b1, b2, a1, a2);
console.log(`\nCorrect state for new buffer: s1=${s0_new.s1.toExponential(8)}, s2=${s0_new.s2.toExponential(8)}`);
console.log(`Persisted state:              s1=${s1.toExponential(8)}, s2=${s2.toExponential(8)}`);
console.log(`State error: |Δs1|=${Math.abs(s1 - s0_new.s1).toExponential(4)}, |Δs2|=${Math.abs(s2 - s0_new.s2).toExponential(4)}`);

// Filter the new buffer with both states, compare output
const correct = filterReadOnly(buf, b0, b1, b2, a1, a2, s0_new.s1, s0_new.s2);
const persisted = filterReadOnly(buf, b0, b1, b2, a1, a2, s1, s2);

// Compute difference between the two filter outputs
let maxDiff = 0, maxDiffIdx = 0;
const diff = new Float64Array(N);
for (let i = 0; i < N; i++) {
    diff[i] = persisted.out[i] - correct.out[i];
    if (Math.abs(diff[i]) > maxDiff) { maxDiff = Math.abs(diff[i]); maxDiffIdx = i; }
}
console.log(`\nMax filter output difference: ${maxDiff.toExponential(4)} at sample ${maxDiffIdx}`);
console.log(`After mix, max buffer error: ${(mix * maxDiff).toExponential(4)}`);

// Show how the difference decays along the buffer
console.log('\nFilter output difference decay:');
for (const n of [0, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]) {
    if (n < N) {
        console.log(`  sample ${n}: ${diff[n].toExponential(4)} (expected: ${(maxDiff * Math.pow(poleMag, n)).toExponential(4)})`);
    }
}

// Now the key question: after mixing persisted-state output into buffer,
// does the buffer still wrap seamlessly?
const buf_persisted = new Float64Array(buf);  // copy current buffer
let ps1 = s1, ps2 = s2;
for (let i = 0; i < N; i++) {
    const x = buf_persisted[i];
    const y = b0 * x + ps1;
    ps1 = b1 * x - a1 * y + ps2;
    ps2 = b2 * x - a2 * y;
    buf_persisted[i] = dry * x + mix * y;
}
// Check circularity: DFT should show only bin 1 for a perfect sine
const mag_pers = fftMag(buf_persisted, 32);
console.log('\nSpectrum of buffer after cycle 1 (persisted state):');
for (let k = 0; k < 8; k++) {
    const db = mag_pers[k] > 1e-15 ? 20 * Math.log10(mag_pers[k] / mag_pers[1]) : -999;
    console.log(`  bin ${k}: ${mag_pers[k].toExponential(4)} (${db.toFixed(1)} dB rel fundamental)`);
}

// Same but with re-solved state
const buf_resolved = new Float64Array(buf);
const sr = solveCircularState(buf_resolved, b0, b1, b2, a1, a2);
let rs1 = sr.s1, rs2 = sr.s2;
for (let i = 0; i < N; i++) {
    const x = buf_resolved[i];
    const y = b0 * x + rs1;
    rs1 = b1 * x - a1 * y + rs2;
    rs2 = b2 * x - a2 * y;
    buf_resolved[i] = dry * x + mix * y;
}
const mag_res = fftMag(buf_resolved, 32);
console.log('\nSpectrum of buffer after cycle 1 (re-solved state):');
for (let k = 0; k < 8; k++) {
    const db = mag_res[k] > 1e-15 ? 20 * Math.log10(mag_res[k] / mag_res[1]) : -999;
    console.log(`  bin ${k}: ${mag_res[k].toExponential(4)} (${db.toFixed(1)} dB rel fundamental)`);
}
