const N = 128;
const cutoff = 12;
const mix = 0.5;

const omega0 = 2 * Math.PI * cutoff / N;
const a = Math.exp(-omega0);
const b = 1 - a;
console.log('alpha =', a, 'b =', b);

let aN = 1;
for (let i = 0; i < N; i++) aN *= a;
console.log('alpha^N =', aN);

const invDenom = Math.abs(1 - aN) > 1e-15 ? 1 / (1 - aN) : 0;
console.log('invDenom =', invDenom);

// Sine at fundamental (harmonic 1)
const arr = new Float32Array(N);
for (let i = 0; i < N; i++) arr[i] = Math.sin(2 * Math.PI * i / N);
const origRMS = Math.sqrt(arr.reduce((a, x) => a + x * x, 0) / N);
console.log('input RMS =', origRMS);

const orig = new Float32Array(N);

// Save + pole 1 forced
let s = 0;
for (let n = 0; n < N; n++) { orig[n] = arr[n]; s = a * s + b * arr[n]; }
console.log('pole1 forced state =', s);
s *= invDenom;
console.log('pole1 circular s0 =', s);

// Pole 1 filter
for (let n = 0; n < N; n++) { s = a * s + b * arr[n]; arr[n] = s; }
const p1RMS = Math.sqrt(arr.reduce((a, x) => a + x * x, 0) / N);
console.log('after pole1 RMS =', p1RMS);

// Pole 2 forced
s = 0;
for (let n = 0; n < N; n++) s = a * s + b * arr[n];
console.log('pole2 forced state =', s);
s *= invDenom;

// Pole 2 filter + mix
const dry = 1 - mix;
for (let n = 0; n < N; n++) { s = a * s + b * arr[n]; arr[n] = dry * orig[n] + mix * s; }
const finalRMS = Math.sqrt(arr.reduce((a, x) => a + x * x, 0) / N);
console.log('final RMS =', finalRMS);
console.log('gain ratio =', finalRMS / origRMS);

// Also test with cutoff=1 (very low)
console.log('\n--- cutoff=1 ---');
const arr2 = new Float32Array(N);
for (let i = 0; i < N; i++) arr2[i] = Math.sin(2 * Math.PI * i / N);
const omega1 = 2 * Math.PI * 1 / N;
const a1 = Math.exp(-omega1);
const b1 = 1 - a1;
let aN1 = 1;
for (let i = 0; i < N; i++) aN1 *= a1;
const invD1 = Math.abs(1 - aN1) > 1e-15 ? 1 / (1 - aN1) : 0;
console.log('alpha =', a1, 'alpha^N =', aN1, 'invDenom =', invD1);

const orig2 = new Float32Array(N);
s = 0;
for (let n = 0; n < N; n++) { orig2[n] = arr2[n]; s = a1 * s + b1 * arr2[n]; }
s *= invD1;
for (let n = 0; n < N; n++) { s = a1 * s + b1 * arr2[n]; arr2[n] = s; }

s = 0;
for (let n = 0; n < N; n++) s = a1 * s + b1 * arr2[n];
s *= invD1;
for (let n = 0; n < N; n++) { s = a1 * s + b1 * arr2[n]; arr2[n] = 0.5 * orig2[n] + 0.5 * s; }
const f2RMS = Math.sqrt(arr2.reduce((a, x) => a + x * x, 0) / N);
console.log('final RMS =', f2RMS, ' gain =', f2RMS / origRMS);

// Test: what if input is all 1 (DC) — should pass through unchanged
console.log('\n--- DC test (cutoff=12) ---');
const dc = new Float32Array(N).fill(1.0);
const dcOrig = new Float32Array(N).fill(1.0);
s = 0;
for (let n = 0; n < N; n++) { s = a * s + b * dc[n]; }
s *= invDenom;
for (let n = 0; n < N; n++) { s = a * s + b * dc[n]; dc[n] = s; }
s = 0;
for (let n = 0; n < N; n++) s = a * s + b * dc[n];
s *= invDenom;
for (let n = 0; n < N; n++) { s = a * s + b * dc[n]; dc[n] = 0.5 * dcOrig[n] + 0.5 * s; }
console.log('DC output[0..3] =', dc[0], dc[1], dc[2], dc[3]);
