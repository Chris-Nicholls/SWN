#!/usr/bin/env node
// Test: warm up (prime) on first buffer, then persistent state forever.
// Does it stay clean?

const N = 1024;
const cutoff = 12;

function biquadCoeffs(cutoff) {
    const omega0 = 2 * Math.PI * cutoff / N;
    const Om = Math.tan(omega0 / 2);
    const Q = 1 / Math.SQRT2;
    const d = 1 + Om / Q + Om * Om;
    return {
        b0: (Om * Om) / d, b1: 2 * (Om * Om) / d, b2: (Om * Om) / d,
        a1: 2 * (Om * Om - 1) / d, a2: (1 - Om / Q + Om * Om) / d,
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

// ================================================================
// Test: Prime once, then persistent state, full filter (no mix)
// ================================================================
console.log('=== Prime once, persistent state, FULL FILTER (no mix) ===');
{
    const buf = new Float64Array(N);
    for (let i = 0; i < N; i++) buf[i] = Math.sin(2 * Math.PI * i / N);

    // Warmup: run filter through buffer WITHOUT writing back
    let s1 = 0, s2 = 0;
    for (let n = 0; n < N; n++) {
        const x = buf[n];
        const y = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
    }
    console.log(`After warmup: s1=${s1.toExponential(6)}, s2=${s2.toExponential(6)}`);

    // Now run 100 cycles with persistent state, writing back
    for (let cycle = 0; cycle < 100; cycle++) {
        for (let n = 0; n < N; n++) {
            const x = buf[n];
            const y = b0 * x + s1;
            s1 = b1 * x - a1 * y + s2;
            s2 = b2 * x - a2 * y;
            buf[n] = y;
        }
        if (cycle === 0 || cycle === 9 || cycle === 99) {
            const mag = fftMag(buf, 32);
            console.log(`  Cycle ${cycle}: SFDR=${sfdr(mag).toFixed(1)} dB, fund=${mag[1].toExponential(4)}`);
        }
    }
}

// ================================================================
// Test: Prime once, persistent state, with WET/DRY mix
// ================================================================
console.log('\n=== Prime once, persistent state, WITH mix=0.14 ===');
{
    const mix = 0.14, dry = 1 - mix;
    const buf = new Float64Array(N);
    for (let i = 0; i < N; i++) buf[i] = Math.sin(2 * Math.PI * i / N);

    let s1 = 0, s2 = 0;
    for (let n = 0; n < N; n++) {
        const x = buf[n];
        const y = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
    }

    for (let cycle = 0; cycle < 100; cycle++) {
        for (let n = 0; n < N; n++) {
            const x = buf[n];
            const y = b0 * x + s1;
            s1 = b1 * x - a1 * y + s2;
            s2 = b2 * x - a2 * y;
            buf[n] = dry * x + mix * y;
        }
        if (cycle === 0 || cycle === 9 || cycle === 99) {
            const mag = fftMag(buf, 32);
            console.log(`  Cycle ${cycle}: SFDR=${sfdr(mag).toFixed(1)} dB, fund=${mag[1].toExponential(4)}`);
        }
    }
}

// ================================================================
// Diagnostic: after warmup + one filter pass, what does the
// filter's input stream look like at the cycle boundary?
// ================================================================
console.log('\n=== Diagnostic: input stream at boundary ===');
{
    const buf = new Float64Array(N);
    for (let i = 0; i < N; i++) buf[i] = Math.sin(2 * Math.PI * i / N);

    // Warmup
    let s1 = 0, s2 = 0;
    for (let n = 0; n < N; n++) {
        const x = buf[n];
        const y = b0 * x + s1;
        s1 = b1 * x - a1 * y + s2;
        s2 = b2 * x - a2 * y;
    }

    // Cycle 0: filter + write back
    let lastInput = 0;
    for (let n = 0; n < N; n++) {
        lastInput = buf[n];  // what the filter reads
        const y = b0 * lastInput + s1;
        s1 = b1 * lastInput - a1 * y + s2;
        s2 = b2 * lastInput - a2 * y;
        buf[n] = y;
    }
    // Now buf = filtered output. Next cycle starts with buf[0].
    console.log(`  Last input to filter (cycle 0):  ${lastInput.toExponential(8)}`);
    console.log(`  Next input (cycle 1 buf[0]):     ${buf[0].toExponential(8)}`);
    console.log(`  Original buf[0] was:             ${Math.sin(0).toExponential(8)}`);
    console.log(`  Jump in filter input:            ${(buf[0] - lastInput).toExponential(4)}`);
    console.log(`  Original wrap (sin[N-1]-sin[0]): ${(Math.sin(2 * Math.PI * (N - 1) / N) - Math.sin(0)).toExponential(4)}`);
}
