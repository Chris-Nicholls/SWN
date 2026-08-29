#!/usr/bin/env node
// Test the user's claim: a plain persistent-state IIR should see no
// discontinuity between end-of-buffer and start-of-buffer.
// Where do the harmonics actually come from?

const N = 1024;
const cutoff = 12;
const mix = 0.14;
const dry = 1 - mix;

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
const { b0, b1, b2, a1, a2 } = biquadCoeffs(cutoff);

function fftMag(buf, numBins) {
    const n = buf.length;
    const mag = new Float64Array(numBins);
    for (let k = 0; k < numBins; k++) {
        let re = 0, im = 0;
        for (let j = 0; j < n; j++) {
            const angle = -2 * Math.PI * k * j / n;
            re += buf[j] * Math.cos(angle);
            im += buf[j] * Math.sin(angle);
        }
        mag[k] = Math.sqrt(re * re + im * im) / n;
    }
    return mag;
}

function sfdr(mag) {
    const fund = mag[1];
    let worst = 0;
    for (let k = 2; k < mag.length; k++) if (mag[k] > worst) worst = mag[k];
    return 20 * Math.log10(worst / fund);
}

// =================================================================
// Test A: Plain persistent IIR, starting from [0,0], with wet/dry
// =================================================================
console.log('=== Test A: Persistent IIR + wet/dry mix ===');
{
    const buf = new Float64Array(N);
    for (let i = 0; i < N; i++) buf[i] = Math.sin(2 * Math.PI * i / N);
    let s1 = 0, s2 = 0;  // start cold
    for (let cycle = 0; cycle < 100; cycle++) {
        for (let n = 0; n < N; n++) {
            const x = buf[n];
            const y = b0 * x + s1;
            s1 = b1 * x - a1 * y + s2;
            s2 = b2 * x - a2 * y;
            buf[n] = dry * x + mix * y;
        }
        if (cycle === 0 || cycle === 99) {
            const mag = fftMag(buf, 32);
            // Check circularity: how well does buf[N-1] match buf[0]?
            const wrap_err = Math.abs(buf[0] - buf[N - 1] + (buf[1] - buf[0]));
            console.log(`  Cycle ${cycle}: SFDR=${sfdr(mag).toFixed(1)} dB, wrap_err=${wrap_err.toExponential(4)}`);
        }
    }
}

// =================================================================
// Test B: Plain persistent IIR, starting from [0,0], NO wet/dry (full filter)
// =================================================================
console.log('\n=== Test B: Persistent IIR, NO wet/dry (buf = filtered) ===');
{
    const buf = new Float64Array(N);
    for (let i = 0; i < N; i++) buf[i] = Math.sin(2 * Math.PI * i / N);
    let s1 = 0, s2 = 0;
    for (let cycle = 0; cycle < 100; cycle++) {
        for (let n = 0; n < N; n++) {
            const x = buf[n];
            const y = b0 * x + s1;
            s1 = b1 * x - a1 * y + s2;
            s2 = b2 * x - a2 * y;
            buf[n] = y;  // full filter, no mix
        }
        if (cycle === 0 || cycle === 99) {
            const mag = fftMag(buf, 32);
            const wrap_err = Math.abs(buf[0] - buf[N - 1] + (buf[1] - buf[0]));
            console.log(`  Cycle ${cycle}: SFDR=${sfdr(mag).toFixed(1)} dB, wrap_err=${wrap_err.toExponential(4)}`);
        }
    }
}

// =================================================================
// Test C: Show what the filter's input stream looks like at boundaries
// =================================================================
console.log('\n=== Test C: Signal continuity at cycle boundary ===');
{
    const buf = new Float64Array(N);
    for (let i = 0; i < N; i++) buf[i] = Math.sin(2 * Math.PI * i / N);
    let s1 = 0, s2 = 0;

    // Run cycle 0
    let lastInput = 0;
    for (let n = 0; n < N; n++) {
        const x = buf[n];
        const y = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
        buf[n] = dry * x + mix * y;
        lastInput = x;
    }
    const firstInputCycle1 = buf[0];
    console.log(`  Last input of cycle 0 (orig buf[N-1]):  ${lastInput.toExponential(8)}`);
    console.log(`  First input of cycle 1 (mod buf[0]):    ${firstInputCycle1.toExponential(8)}`);
    console.log(`  Jump:                                    ${(firstInputCycle1 - lastInput).toExponential(4)}`);
    console.log(`  Original buf[N-1]-buf[0] (circular):     ${(Math.sin(2 * Math.PI * (N - 1) / N) - Math.sin(0)).toExponential(4)}`);

    // What does the BUFFER look like — is it still circular?
    console.log(`\n  Buffer circularity after cycle 0:`);
    console.log(`    buf[0]   = ${buf[0].toExponential(8)}`);
    console.log(`    buf[1]   = ${buf[1].toExponential(8)}`);
    console.log(`    buf[N-2] = ${buf[N - 2].toExponential(8)}`);
    console.log(`    buf[N-1] = ${buf[N - 1].toExponential(8)}`);

    // Compare: first derivative at wrap point
    const dStart = buf[1] - buf[0];
    const dEnd = buf[0] - buf[N - 1];  // wrapping derivative
    const dMid = buf[N / 2 + 1] - buf[N / 2];
    console.log(`    d at start:  ${dStart.toExponential(8)}`);
    console.log(`    d at wrap:   ${dEnd.toExponential(8)}`);
    console.log(`    d at mid:    ${dMid.toExponential(8)}`);
    console.log(`    Wrap derivative mismatch: ${((dEnd - dStart) / dMid * 100).toFixed(4)}% of mid`);
}

// =================================================================
// Test D: Is the OUTPUT of the filter circular?
// Show that causal IIR makes the buffer non-circular
// =================================================================
console.log('\n=== Test D: Filter output circularity ===');
{
    const buf = new Float64Array(N);
    for (let i = 0; i < N; i++) buf[i] = Math.sin(2 * Math.PI * i / N);

    // Apply causal IIR from [0,0] — just the filter output, don't write back
    const y_causal = new Float64Array(N);
    let s1 = 0, s2 = 0;
    for (let n = 0; n < N; n++) {
        const x = buf[n];
        y_causal[n] = b0 * x + s1;
        s1 = b1 * x - a1 * y_causal[n] + s2;
        s2 = b2 * x - a2 * y_causal[n];
    }

    // Now the mixed buffer
    const mixed = new Float64Array(N);
    for (let n = 0; n < N; n++) mixed[n] = dry * buf[n] + mix * y_causal[n];

    console.log('  Original sine circularity:');
    console.log(`    buf[0]=${buf[0].toExponential(6)}, buf[N-1]=${buf[N - 1].toExponential(6)}, gap=${(buf[0] - buf[N - 1]).toExponential(4)}`);

    console.log('  Causal filter output circularity:');
    console.log(`    y[0]=${y_causal[0].toExponential(6)}, y[N-1]=${y_causal[N - 1].toExponential(6)}, gap=${(y_causal[0] - y_causal[N - 1]).toExponential(4)}`);

    console.log('  Mixed buffer circularity:');
    console.log(`    m[0]=${mixed[0].toExponential(6)}, m[N-1]=${mixed[N - 1].toExponential(6)}, gap=${(mixed[0] - mixed[N - 1]).toExponential(4)}`);

    const mag_mixed = fftMag(mixed, 32);
    console.log(`  Mixed buffer SFDR: ${sfdr(mag_mixed).toFixed(1)} dB`);

    // DFT of just the filter output (not mixed)
    const mag_y = fftMag(y_causal, 32);
    console.log(`  Causal filter output SFDR: ${sfdr(mag_y).toFixed(1)} dB`);
}
