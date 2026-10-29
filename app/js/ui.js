// ========================================================================
//  UI / RENDERING (View)
//
//  Harmonic & kernel editors, canvas drawing, audio playback, App class.
//  Depends on synths.js (clamp). Audio runs in AudioWorklet (audio-processor.js).
// ========================================================================

const NUM_HARMONICS = 32;
const WAVETABLE_LEN = 2048;
const NOTES = [];
const NOTE_NAMES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B'];
for (let oct = 1; oct <= 7; oct++) {
    for (const n of NOTE_NAMES) NOTES.push(n + oct);
}

function noteToFreq(name) {
    const m = NOTE_NAMES.indexOf(name.replace(/\d/, ''));
    const o = parseInt(name.slice(-1));
    const midi = 12 * (o + 1) + m;
    return 440 * Math.pow(2, (midi - 69) / 12);
}

// VCV Rack-style keyboard → semitone offset mapping
// Lower octave: bottom two rows (ZXCVBNM + SDFGHJ as sharps)
// Upper octave: top two rows  (QWERTYU + 23567  as sharps)
const KEYBOARD_NOTE_MAP = {
    'KeyZ': 0, 'KeyS': 1, 'KeyX': 2, 'KeyD': 3,
    'KeyC': 4, 'KeyV': 5, 'KeyG': 6, 'KeyB': 7,
    'KeyH': 8, 'KeyN': 9, 'KeyJ': 10, 'KeyM': 11,
    'KeyQ': 12, 'Digit2': 13, 'KeyW': 14, 'Digit3': 15,
    'KeyE': 16, 'KeyR': 17, 'Digit5': 18, 'KeyT': 19,
    'Digit6': 20, 'KeyY': 21, 'Digit7': 22, 'KeyU': 23,
};
const DEFAULT_KEYBOARD_OCTAVE = 3;   // base octave (Z = C3)

// ========================================================================
//  CANVAS HELPERS
// ========================================================================
function setupCanvas(canvas) {
    if (!canvas.dataset.logicalH) canvas.dataset.logicalH = canvas.height;
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    const w = rect.width;
    const h = Number(canvas.dataset.logicalH);
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    canvas.style.height = h + 'px';
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    canvas._cachedDraw = { ctx, w, h };
    return { ctx, w, h };
}

function getCanvasInfo(canvas) {
    if (canvas._cachedDraw) return canvas._cachedDraw;
    return setupCanvas(canvas);
}

// ========================================================================
//  KERNEL EDITOR
// ========================================================================
const MAX_KERNEL_WIDTH = 9;

class KernelEditor {
    constructor(canvas) {
        this.canvas = canvas;
        this.weights = new Float64Array(MAX_KERNEL_WIDTH);
        this.width = 3;
        this.weights[0] = 0.25;
        this.weights[1] = 0.50;
        this.weights[2] = 0.25;
        this._logicalH = 100;
        canvas.height = this._logicalH;
        const { ctx, w, h } = setupCanvas(canvas);
        this.ctx = ctx; this.w = w; this.h = h;
        this._drawing = false;
        canvas.addEventListener('mousedown', e => { this._drawing = true; this._setBar(e); });
        canvas.addEventListener('mousemove', e => { if (this._drawing) this._setBar(e); });
        canvas.addEventListener('mouseup', () => this._drawing = false);
        canvas.addEventListener('mouseleave', () => this._drawing = false);
        canvas.addEventListener('touchstart', e => { e.preventDefault(); this._drawing = true; this._setBar(e.touches[0]); }, { passive: false });
        canvas.addEventListener('touchmove', e => { e.preventDefault(); if (this._drawing) this._setBar(e.touches[0]); }, { passive: false });
        canvas.addEventListener('touchend', () => this._drawing = false);
    }

    _setBar(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        const barW = this.w / this.width;
        const idx = Math.floor(x / barW);
        if (idx < 0 || idx >= this.width) return;
        this.weights[idx] = clamp(1 - y / this.h, 0, 1);
        this.draw();
        this._updateInfo();
        if (app && document.getElementById('chkAutoPluck').checked) {
            app._autoPluckTimer && clearTimeout(app._autoPluckTimer);
            app._autoPluckTimer = setTimeout(() => app.pluck(undefined, true), 80);
        }
    }

    setWidth(w) {
        const newW = Math.max(1, Math.min(MAX_KERNEL_WIDTH, w | 0));
        if (newW === this.width) return;
        const oldCenter = Math.floor(this.width / 2);
        const newCenter = Math.floor(newW / 2);
        const old = new Float64Array(this.weights);
        this.weights.fill(0);
        for (let i = 0; i < newW; i++) {
            const srcIdx = i - newCenter + oldCenter;
            if (srcIdx >= 0 && srcIdx < this.width) this.weights[i] = old[srcIdx];
        }
        this.width = newW;
        this.draw();
        this._updateInfo();
    }

    getKernel() {
        const k = new Float64Array(this.width);
        for (let i = 0; i < this.width; i++) k[i] = this.weights[i];
        if (document.getElementById('chkKernelNorm').checked) {
            let sum = 0;
            for (let i = 0; i < this.width; i++) sum += Math.abs(k[i]);
            if (sum > 1e-12) for (let i = 0; i < this.width; i++) k[i] /= sum;
        }
        return k;
    }

    _updateInfo() {
        const k = this.getKernel();
        const vals = Array.from(k).map(v => v.toFixed(3)).join(', ');
        const el = document.getElementById('kernelInfo');
        if (el) el.textContent = `kernel: [${vals}]`;
    }

    draw() {
        const { ctx, w, h } = this;
        ctx.clearRect(0, 0, w, h);
        const n = this.width;
        const barW = w / n;
        const gap = Math.max(1, barW * 0.1);
        ctx.strokeStyle = '#333'; ctx.lineWidth = 0.5;
        ctx.beginPath(); ctx.moveTo(0, h); ctx.lineTo(w, h); ctx.stroke();
        const center = Math.floor(n / 2);
        for (let i = 0; i < n; i++) {
            const val = this.weights[i];
            const bh = val * h;
            const x = i * barW + gap / 2;
            const bw = barW - gap;
            const dist = Math.abs(i - center) / Math.max(1, center);
            const r = Math.round(0 + dist * 100);
            const g = Math.round(212 - dist * 100);
            const b = Math.round(170 - dist * 60);
            ctx.fillStyle = `rgb(${r},${g},${b})`;
            ctx.fillRect(x, h - bh, bw, bh);
            if (barW > 18) {
                ctx.fillStyle = '#888'; ctx.font = '8px monospace'; ctx.textAlign = 'center';
                ctx.fillText(val.toFixed(2), x + bw / 2, h - bh - 3);
            }
        }
        this._updateInfo();
    }
}

// ========================================================================
//  HARMONIC EDITOR
// ========================================================================
class HarmonicEditor {
    constructor(canvas) {
        this.amplitudes = new Float64Array(NUM_HARMONICS);
        this.phases = new Float64Array(NUM_HARMONICS);
        this.editMode = 'amplitude';
        this.canvas = canvas;
        this._logicalH = 180;
        canvas.height = this._logicalH;
        const { ctx, w, h } = setupCanvas(canvas);
        this.ctx = ctx; this.w = w; this.h = h;
        this._drawing = false;
        canvas.addEventListener('mousedown', e => { this._drawing = true; this._setBar(e); });
        canvas.addEventListener('mousemove', e => { if (this._drawing) this._setBar(e); });
        canvas.addEventListener('mouseup', () => this._drawing = false);
        canvas.addEventListener('mouseleave', () => this._drawing = false);
        canvas.addEventListener('touchstart', e => { e.preventDefault(); this._drawing = true; this._setBar(e.touches[0]); }, { passive: false });
        canvas.addEventListener('touchmove', e => { e.preventDefault(); if (this._drawing) this._setBar(e.touches[0]); }, { passive: false });
        canvas.addEventListener('touchend', () => this._drawing = false);
    }

    _setBar(e) {
        const rect = this.canvas.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        const barW = this.w / NUM_HARMONICS;
        const idx = Math.floor(x / barW);
        if (idx < 0 || idx >= NUM_HARMONICS) return;
        if (this.editMode === 'phase') {
            this.phases[idx] = clamp(y / this.h, 0, 1);
        } else {
            this.amplitudes[idx] = clamp(1 - y / this.h, 0, 1);
        }
        this.draw();
        if (app) app.onHarmonicsChanged();
    }

    draw() {
        const { ctx, w, h } = this;
        ctx.clearRect(0, 0, w, h);
        const barW = w / NUM_HARMONICS;
        const gap = Math.max(1, barW * 0.12);

        if (this.editMode === 'amplitude') {
            for (let i = 0; i < NUM_HARMONICS; i++) {
                const amp = this.amplitudes[i];
                const bh = amp * h;
                const x = i * barW + gap / 2;
                const bw = barW - gap;
                const t = amp;
                const r = Math.round(t < 0.5 ? 0 : (t - 0.5) * 2 * 255);
                const g = Math.round(t < 0.5 ? t * 2 * 212 : (1 - (t - 0.5) * 2) * 212);
                const b = Math.round(t < 0.5 ? (1 - t * 2) * 170 : 0);
                ctx.fillStyle = `rgb(${r},${g},${b})`;
                ctx.fillRect(x, h - bh, bw, bh);
                if (barW > 14) {
                    ctx.fillStyle = '#666'; ctx.font = '9px monospace'; ctx.textAlign = 'center';
                    ctx.fillText(String(i + 1), x + bw / 2, h - 2);
                }
            }
        } else {
            const mid = h / 2;
            ctx.strokeStyle = '#333'; ctx.lineWidth = 0.5;
            ctx.beginPath(); ctx.moveTo(0, mid); ctx.lineTo(w, mid); ctx.stroke();
            ctx.strokeStyle = '#222'; ctx.setLineDash([2, 4]);
            ctx.beginPath(); ctx.moveTo(0, h * 0.25); ctx.lineTo(w, h * 0.25); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(0, h * 0.75); ctx.lineTo(w, h * 0.75); ctx.stroke();
            ctx.setLineDash([]);
            for (let i = 0; i < NUM_HARMONICS; i++) {
                const ph = this.phases[i];
                const x = i * barW + gap / 2;
                const bw = barW - gap;
                const yPos = ph * h;
                const barH = yPos - mid;
                const hue = ph * 360;
                const alpha = this.amplitudes[i] > 1e-4 ? 0.9 : 0.25;
                ctx.fillStyle = `hsla(${hue}, 70%, 55%, ${alpha})`;
                if (barH >= 0) ctx.fillRect(x, mid, bw, barH);
                else ctx.fillRect(x, mid + barH, bw, -barH);
                if (barW > 14) {
                    const deg = Math.round(ph * 360);
                    ctx.fillStyle = '#888'; ctx.font = '8px monospace'; ctx.textAlign = 'center';
                    ctx.fillText(deg + '°', x + bw / 2, h - 2);
                    ctx.fillStyle = '#555';
                    ctx.fillText(String(i + 1), x + bw / 2, 9);
                }
            }
        }
    }

    buildWavetable(len) {
        const buf = new Float32Array(len);
        for (let n = 0; n < len; n++) {
            let s = 0;
            for (let k = 0; k < NUM_HARMONICS; k++) {
                if (this.amplitudes[k] < 1e-6) continue;
                const phaseOffset = this.phases[k] * 2 * Math.PI;
                s += this.amplitudes[k] * Math.sin(2 * Math.PI * (k + 1) * n / len + phaseOffset);
            }
            buf[n] = s;
        }
        let mx = 0;
        for (let n = 0; n < len; n++) mx = Math.max(mx, Math.abs(buf[n]));
        if (mx > 1e-12) for (let n = 0; n < len; n++) buf[n] /= mx;
        return buf;
    }

    /**
     * Build both the normal wavetable (sin harmonics) and its Hilbert
     * transform (-cos harmonics), normalised by the SAME factor so
     * the Re/Im ratio is physically correct for a traveling wave.
     */
    buildWavetablePair(len) {
        const re = new Float32Array(len);
        const im = new Float32Array(len);
        for (let n = 0; n < len; n++) {
            let sr = 0, si = 0;
            for (let k = 0; k < NUM_HARMONICS; k++) {
                if (this.amplitudes[k] < 1e-6) continue;
                const ph = this.phases[k] * 2 * Math.PI;
                const angle = 2 * Math.PI * (k + 1) * n / len + ph;
                sr += this.amplitudes[k] * Math.sin(angle);
                si -= this.amplitudes[k] * Math.cos(angle);
            }
            re[n] = sr;
            im[n] = si;
        }
        // Normalise both by the real part's peak (preserves Re/Im ratio)
        let mx = 0;
        for (let n = 0; n < len; n++) mx = Math.max(mx, Math.abs(re[n]));
        if (mx > 1e-12) {
            for (let n = 0; n < len; n++) { re[n] /= mx; im[n] /= mx; }
        }
        return { real: re, imag: im };
    }
}

// ========================================================================
//  DRAWING FUNCTIONS
// ========================================================================
function drawWaveform(canvas, data, color = '#00d4aa') {
    const { ctx, w, h } = getCanvasInfo(canvas);
    ctx.clearRect(0, 0, w, h);
    if (!data || !data.length) return;
    ctx.strokeStyle = color; ctx.lineWidth = 1.2;
    ctx.beginPath();
    for (let i = 0; i < data.length; i++) {
        const x = (i / (data.length - 1)) * w;
        const y = (1 - data[i]) * 0.5 * h;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.strokeStyle = '#333'; ctx.lineWidth = 0.5;
    ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
}

function drawWaterfall(canvas, snapshots, bufferLen) {
    const { ctx, w, h } = getCanvasInfo(canvas);
    ctx.clearRect(0, 0, w, h);
    if (!snapshots || snapshots.length < 2) return;
    const numRows = snapshots.length;
    const img = ctx.createImageData(w, h);
    const data = img.data; // Uint8ClampedArray RGBA
    for (let row = 0; row < numRows; row++) {
        const snap = snapshots[row];
        const y0 = Math.round(row * h / numRows);
        const y1 = Math.round((row + 1) * h / numRows);
        for (let col = 0; col < w; col++) {
            const bufIdx = (col / w) * snap.length;
            const idx = bufIdx | 0;
            const frac = bufIdx - idx;
            const s0 = snap[Math.min(idx, snap.length - 1)];
            const s1 = snap[Math.min(idx + 1, snap.length - 1)];
            const v = Math.max(-1, Math.min(1, s0 + frac * (s1 - s0)));
            let r, g, b;
            if (v >= 0) { r = 20 + v * 235; g = 20 + v * 120; b = 20 + v * 20; }
            else { const t = -v; r = 20 + t * 20; g = 20 + t * 100; b = 20 + t * 235; }
            // Write to all pixel rows this snapshot occupies
            for (let py = y0; py < y1; py++) {
                const off = (py * w + col) << 2;
                data[off] = r; data[off + 1] = g; data[off + 2] = b; data[off + 3] = 255;
            }
        }
    }
    ctx.putImageData(img, 0, 0);
    ctx.fillStyle = '#888'; ctx.font = '9px monospace';
    ctx.fillText('t=0', 2, 10);
    ctx.fillText('t=end', 2, h - 3);
}

function drawEnvelope(canvas, peaks) {
    const { ctx, w, h } = getCanvasInfo(canvas);
    ctx.clearRect(0, 0, w, h);
    if (!peaks || peaks.length < 2) return;
    const mx = Math.max(...peaks) || 1;
    ctx.strokeStyle = '#00d4aa'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let i = 0; i < peaks.length; i++) {
        const x = (i / (peaks.length - 1)) * w;
        const y = (1 - peaks[i] / mx) * h;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.fillStyle = '#555'; ctx.font = '9px monospace';
    ctx.fillText('RMS per cycle →', w - 90, h - 4);
}

// In-place radix-2 Cooley-Tukey FFT (re[], im[] length must be power of 2)
function _fftInPlace(re, im) {
    const N = re.length;
    // Bit-reversal permutation
    for (let i = 1, j = 0; i < N; i++) {
        let bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            let tmp = re[i]; re[i] = re[j]; re[j] = tmp;
            tmp = im[i]; im[i] = im[j]; im[j] = tmp;
        }
    }
    // Butterfly passes
    for (let len = 2; len <= N; len <<= 1) {
        const halfLen = len >> 1;
        const angle = -2 * Math.PI / len;
        const wRe = Math.cos(angle), wIm = Math.sin(angle);
        for (let i = 0; i < N; i += len) {
            let curRe = 1, curIm = 0;
            for (let j = 0; j < halfLen; j++) {
                const a = i + j, b = a + halfLen;
                const tRe = curRe * re[b] - curIm * im[b];
                const tIm = curRe * im[b] + curIm * re[b];
                re[b] = re[a] - tRe; im[b] = im[a] - tIm;
                re[a] += tRe; im[a] += tIm;
                const nextRe = curRe * wRe - curIm * wIm;
                curIm = curRe * wIm + curIm * wRe;
                curRe = nextRe;
            }
        }
    }
}

function drawFFT(canvas, data) {
    const { ctx, w, h } = getCanvasInfo(canvas);
    ctx.clearRect(0, 0, w, h);
    if (!data || data.length < 2) return;
    // Pad to next power of 2
    const raw = data.length;
    let N = 1; while (N < raw) N <<= 1;
    const re = new Float32Array(N);
    const im = new Float32Array(N);
    for (let i = 0; i < raw; i++) re[i] = data[i];
    _fftInPlace(re, im);
    const halfN = N >> 1;
    const mag = new Float32Array(halfN);
    for (let k = 0; k < halfN; k++) {
        mag[k] = Math.sqrt(re[k] * re[k] + im[k] * im[k]) / N;
    }
    // Convert to dB, floor at -80 dB
    const magDB = new Float32Array(halfN);
    let peak = 0;
    for (let k = 0; k < halfN; k++) { if (mag[k] > peak) peak = mag[k]; }
    if (peak < 1e-12) return;
    for (let k = 0; k < halfN; k++) {
        magDB[k] = 20 * Math.log10(Math.max(mag[k] / peak, 1e-4));
    }
    const dbMin = -80;
    // Logarithmic x-axis: map bin k (1..halfN-1) to x via log spacing
    // Skip bin 0 (DC)
    const logMin = Math.log(1);
    const logMax = Math.log(halfN);
    const logRange = logMax - logMin;
    // Draw filled bars between adjacent log-spaced positions
    for (let k = 1; k < halfN; k++) {
        const x0 = ((Math.log(k) - logMin) / logRange) * w;
        const x1 = ((Math.log(k + 1) - logMin) / logRange) * w;
        const norm = clamp((magDB[k] - dbMin) / -dbMin, 0, 1);
        const barH = norm * h;
        const g = Math.round(180 + norm * 32);
        const b = Math.round(140 + norm * 100);
        ctx.fillStyle = `rgb(0,${g},${b})`;
        ctx.fillRect(x0, h - barH, Math.max(1, Math.ceil(x1 - x0)), barH);
    }
    // Axis labels — show harmonic numbers at key positions
    ctx.fillStyle = '#555'; ctx.font = '9px monospace';
    ctx.fillText('0 dB', 2, 10);
    ctx.fillText(dbMin + ' dB', 2, h - 4);
    const labelBins = [1, 2, 4, 8, 16, 32, 64, 128, 256];
    for (const kb of labelBins) {
        if (kb >= halfN) break;
        const x = ((Math.log(kb) - logMin) / logRange) * w;
        ctx.fillStyle = '#333';
        ctx.fillRect(x, 0, 1, h);
        ctx.fillStyle = '#666'; ctx.font = '9px monospace';
        ctx.fillText(kb.toString(), x + 2, h - 2);
    }
}

// ========================================================================
//  AUDIO PLAYBACK (AudioWorklet — dedicated real-time thread)
// ========================================================================
let audioCtx = null;
let workletNode = null;
let gainNode = null;
let workletReady = false;
let _workletPromise = null;

async function initAudio() {
    if (_workletPromise) return _workletPromise;
    _workletPromise = (async () => {
        audioCtx = new (window.AudioContext || window.webkitAudioContext)();
        // Persistent gain node
        gainNode = audioCtx.createGain();
        gainNode.gain.value = 0.5;
        gainNode.connect(audioCtx.destination);

        // Load synth + driver + processor into a single worklet module via Blob
        const [synthsSrc, driverSrc, processorSrc] = await Promise.all([
            fetch('synths.js').then(r => r.text()),
            fetch('driver.js').then(r => r.text()),
            fetch('audio-processor.js').then(r => r.text()),
        ]);
        const combined = synthsSrc + '\n' + driverSrc + '\n' + processorSrc;
        const blob = new Blob([combined], { type: 'application/javascript' });
        const url = URL.createObjectURL(blob);
        await audioCtx.audioWorklet.addModule(url);
        URL.revokeObjectURL(url);

        // Create persistent worklet node
        workletNode = new AudioWorkletNode(audioCtx, 'synth-processor');
        workletNode.connect(gainNode);

        // Receive visualization data from worklet
        workletNode.port.onmessage = (e) => {
            if (e.data.type === 'viz' && app) {
                const d = e.data;
                for (const s of d.snapshots) app._vizSnapshots.push(s);
                for (const p of d.peaks) app._vizPeaks.push(p);
                if (d.velSnapshots) {
                    for (const vs of d.velSnapshots) app._vizVelSnapshots.push(vs);
                }
                app._vizBufLen = d.bufferLen;
                if (d.hiddenRings) app._vizHiddenRings = d.hiddenRings;
                if (d.done) app._engineDone = true;
            }
        };

        workletReady = true;
    })();
    return _workletPromise;
}

// Warm up audio on first user gesture
document.addEventListener('click', () => initAudio(), { once: true });
document.addEventListener('keydown', () => initAudio(), { once: true });

function startStreaming(algorithm, opts, volume, voice) {
    if (!workletReady) return;
    if (audioCtx.state === 'suspended') audioCtx.resume();
    gainNode.gain.value = clamp(volume, 0, 1);
    workletNode.port.postMessage({ type: 'start', algorithm, opts, voice: voice ?? 0 });
}

function setStreamVolume(vol) {
    if (gainNode) gainNode.gain.value = clamp(vol, 0, 1);
}

function stopStreaming(voice) {
    if (workletNode) workletNode.port.postMessage({ type: 'stop', voice });
}

// ========================================================================
//  APP
// ========================================================================
class App {
    constructor() {
        this.editor = new HarmonicEditor(document.getElementById('cvHarmonics'));
        this.kernelEditor = new KernelEditor(document.getElementById('cvKernel'));
        this._animFrame = null;
        this._engine = null;
        this._keyboardOctave = DEFAULT_KEYBOARD_OCTAVE;
        this._keysHeld = new Set();
        // 3-voice round-robin allocation
        this._nextVoice = 0;
        this._numVoices = 3;
        this._keyToVoice = new Map();  // e.code → voice index

        // Visualization state (populated by worklet messages)
        this._vizSnapshots = [];
        this._vizVelSnapshots = [];
        this._vizPeaks = [];
        this._vizBufLen = 0;
        this._engineDone = false;

        this.preset('saw');

        this._wireSlider('slFineTune', 'valFineTune', v => v + ' ct');
        this._wireSlider('slNoise', 'valNoise', v => Number(v).toFixed(2));
        this._wireSlider('slNoiseColor', 'valNoiseColor', v => {
            const n = Number(v);
            if (n < 0.01) return 'white';
            // Exponential mapping: alpha = 1 - 10^(-4n)
            const alpha = 1 - Math.pow(10, -4 * n);
            const bw = (1 - alpha) * 100;
            return bw < 0.1 ? bw.toFixed(3) + '% BW' : bw < 1 ? bw.toFixed(2) + '% BW' : bw.toFixed(1) + '% BW';
        });
        this._wireSlider('slVolume', 'valVolume', v => Number(v).toFixed(2));
        this._wireSlider('slCycleLP', 'valCycleLP', v => Number(v).toFixed(2));
        this._wireSlider('slCycleHP', 'valCycleHP', v => Number(v).toFixed(4));
        document.getElementById('slVolume').addEventListener('input', () => {
            setStreamVolume(parseFloat(document.getElementById('slVolume').value));
        });

        const slKW = document.getElementById('slKernelWidth');
        const valKW = document.getElementById('valKernelWidth');
        slKW.addEventListener('input', () => {
            valKW.textContent = slKW.value;
            this.kernelEditor.setWidth(parseInt(slKW.value));
        });

        // ── Algorithm selector: auto-populated from SYNTH_REGISTRY_ARRAY ──────
        const algoSel = document.getElementById('selAlgo');
        algoSel.innerHTML = '';
        if (typeof SYNTH_REGISTRY_ARRAY !== 'undefined') {
            let lastGroup = null;
            let groupEl = null;
            for (const entry of SYNTH_REGISTRY_ARRAY) {
                if (entry.group !== lastGroup) {
                    groupEl = document.createElement('optgroup');
                    groupEl.label = entry.group || 'Other';
                    algoSel.appendChild(groupEl);
                    lastGroup = entry.group;
                }
                const opt = document.createElement('option');
                opt.value = entry.id;
                opt.textContent = entry.label;
                (groupEl || algoSel).appendChild(opt);
            }
        }

        // All algo-specific params are generated dynamically in _showAlgoParams.

        const sel = document.getElementById('selNote');
        for (const n of NOTES) {
            const opt = document.createElement('option');
            opt.value = n; opt.textContent = n;
            if (n === 'C3') opt.selected = true;
            sel.appendChild(opt);
        }
        sel.addEventListener('change', () => this._updateFreqDisplay());

        document.getElementById('selAlgo').addEventListener('change', () => this._showAlgoParams());
        this._showAlgoParams();
        this._updateFreqDisplay();

        document.addEventListener('keydown', e => {
            // Ignore OS key-repeat entirely — all musical keys act on first press only
            if (e.repeat) return;

            // Is this a musical key (note, octave, space, escape)?
            const isNoteKey = KEYBOARD_NOTE_MAP[e.code] !== undefined;
            const isMusicalKey = isNoteKey
                || e.code === 'Space' || e.code === 'Escape'
                || e.code === 'Minus' || e.code === 'NumpadSubtract'
                || e.code === 'Equal' || e.code === 'NumpadAdd';

            // Block non-musical keys when a form element is focused (typing text, etc.)
            if (!isMusicalKey && e.target.matches('input,select,textarea')) return;

            // Blur focused UI element so keyboard keeps working after slider interaction
            if (isMusicalKey && e.target.matches('input,select,textarea')) {
                e.target.blur();
            }

            if (e.code === 'Space') {
                e.preventDefault();
                if (!this._keysHeld.has(e.code)) {
                    this._keysHeld.add(e.code);
                    const voice = this._nextVoice;
                    this._nextVoice = (this._nextVoice + 1) % this._numVoices;
                    this._keyToVoice.set(e.code, voice);
                    this.pluck(voice);
                }
                return;
            }
            if (e.code === 'Escape') { this.stop(); return; }

            // Octave shift (VCV Rack: - / = keys)
            if (e.code === 'Minus' || e.code === 'NumpadSubtract') {
                e.preventDefault();
                this._keyboardOctave = Math.max(1, this._keyboardOctave - 1);
                this._updateOctaveDisplay(); return;
            }
            if (e.code === 'Equal' || e.code === 'NumpadAdd') {
                e.preventDefault();
                this._keyboardOctave = Math.min(6, this._keyboardOctave + 1);
                this._updateOctaveDisplay(); return;
            }

            // Keyboard note input
            const semi = KEYBOARD_NOTE_MAP[e.code];
            if (semi !== undefined && !this._keysHeld.has(e.code)) {
                e.preventDefault();
                this._keysHeld.add(e.code);
                const noteIdx = semi + this._keyboardOctave * 12;  // semitones from C0
                const octave = Math.floor(noteIdx / 12);
                const name = NOTE_NAMES[noteIdx % 12] + octave;
                if (octave >= 1 && octave <= 7) {
                    const voice = this._nextVoice;
                    this._nextVoice = (this._nextVoice + 1) % this._numVoices;
                    this._keyToVoice.set(e.code, voice);
                    this.pluckNote(name, voice);
                }
            }
        });
        document.addEventListener('keyup', e => {
            this._keysHeld.delete(e.code);
            // Send gate-off to the voice so ADSR enters release phase
            const voice = this._keyToVoice.get(e.code);
            if (voice !== undefined && workletNode) {
                workletNode.port.postMessage({ type: 'gate', voice, on: false });
            }
            this._keyToVoice.delete(e.code);
        });
        this._updateOctaveDisplay();
    }

    _wireSlider(sliderId, valId, fmt) {
        const sl = document.getElementById(sliderId);
        const vl = document.getElementById(valId);
        const update = () => { vl.textContent = fmt(sl.value); this._updateFreqDisplay(); };
        sl.addEventListener('input', update);
        update();
    }

    _showAlgoParams() {
        const algo = document.getElementById('selAlgo').value;

        // Hide all legacy hardcoded panels (still in DOM but empty)
        for (const id of ['paramsLP', 'paramsHeat', 'paramsBurgers', 'paramsNL', 'paramsLag',
            'paramsBloom', 'paramsMB', 'paramsMBOpt', 'paramsMBV3', 'paramsMBV4',
            'paramsRC', 'paramsRCX', 'paramsMf', 'paramsSW', 'paramsMaxwell', 'paramsWG']) {
            const el = document.getElementById(id);
            if (el) el.classList.add('hidden');
        }

        // Buffer-mode selector: show for double-ended algorithms
        const regEntry = (typeof SYNTH_REGISTRY !== 'undefined') ? SYNTH_REGISTRY[algo] : null;
        const cgSym = document.getElementById('cgSymmetric');
        if (cgSym) cgSym.style.display = (regEntry && regEntry.isDoubleEnded) ? '' : 'none';

        // Build dynamic params panel
        const dynPanel = document.getElementById('paramsDynamic');
        const dynCtrl = document.getElementById('dynamicControls');
        if (!dynPanel || !regEntry) {
            if (dynPanel) dynPanel.classList.add('hidden');
            return;
        }

        const autoPluck = () => {
            if (document.getElementById('chkAutoPluck').checked) {
                this._autoPluckTimer && clearTimeout(this._autoPluckTimer);
                this._autoPluckTimer = setTimeout(() => this.pluck(undefined, true), 80);
            }
        };

        // Preserve existing slider values when rebuilding so a note keeps playing
        const savedVals = {};
        dynCtrl.querySelectorAll('input[type=range]').forEach(el => {
            savedVals[el.id] = el.value;
        });
        dynCtrl.innerHTML = '';

        if (regEntry.cls.hasBuildUI) {
            // RingCouplerX and similar: bespoke UI builder
            const currentOpts = {};
            for (const [id, v] of Object.entries(savedVals))
                currentOpts[id.replace(/^sl_/, '')] = parseFloat(v);
            regEntry.cls.buildUI(dynCtrl, currentOpts, autoPluck);
        } else {
            for (const p of regEntry.params) {
                const cg = document.createElement('div');
                cg.className = 'cg';
                const lbl = document.createElement('label');
                lbl.textContent = p.label;
                const sl = document.createElement('input');
                sl.type = 'range'; sl.id = 'sl_' + p.key;
                sl.min = p.min; sl.max = p.max; sl.step = p.step;
                // Restore saved value if same algo was already shown, else use default
                sl.value = savedVals['sl_' + p.key] ?? p.default;
                const val = document.createElement('span');
                val.className = 'val'; val.id = 'val_' + p.key;
                val.textContent = p.format ? p.format(Number(sl.value)) : sl.value;
                sl.addEventListener('input', () => {
                    val.textContent = p.format ? p.format(Number(sl.value)) : sl.value;
                    autoPluck();
                });
                cg.appendChild(lbl); cg.appendChild(sl); cg.appendChild(val);
                dynCtrl.appendChild(cg);
            }
        }
        dynPanel.classList.remove('hidden');
    }

    _updateOctaveDisplay() {
        const el = document.getElementById('kbdOctave');
        if (el) el.textContent = 'Oct ' + this._keyboardOctave;
    }

    pluckNote(noteName, voice) {
        const sel = document.getElementById('selNote');
        // only update dropdown if the note exists in it
        for (let i = 0; i < sel.options.length; i++) {
            if (sel.options[i].value === noteName) { sel.selectedIndex = i; break; }
        }
        // Suppress auto-pluck — we trigger pluck() directly below
        this._suppressAutoPluck = true;
        this._updateFreqDisplay();
        this._suppressAutoPluck = false;
        this.pluck(voice);
    }

    _updateFreqDisplay() {
        const note = document.getElementById('selNote').value;
        const cents = parseFloat(document.getElementById('slFineTune').value);
        const freq = noteToFreq(note) * Math.pow(2, cents / 1200);
        document.getElementById('valFreq').textContent = freq.toFixed(1) + ' Hz';
        if (!this._suppressAutoPluck && document.getElementById('chkAutoPluck').checked) {
            this._autoPluckTimer && clearTimeout(this._autoPluckTimer);
            this._autoPluckTimer = setTimeout(() => this.pluck(undefined, true), 80);
        }
    }

    toggleEditMode() {
        const ed = this.editor;
        ed.editMode = ed.editMode === 'amplitude' ? 'phase' : 'amplitude';
        const btn = document.getElementById('btnEditMode');
        btn.textContent = ed.editMode === 'amplitude' ? 'Editing: Amplitude' : 'Editing: Phase';
        btn.style.color = ed.editMode === 'phase' ? 'var(--accent2)' : '';
        ed.draw();
    }

    resetPhases() {
        this.editor.phases.fill(0);
        this.editor.draw();
        this.onHarmonicsChanged();
    }

    randomisePhases() {
        for (let i = 0; i < NUM_HARMONICS; i++) this.editor.phases[i] = Math.random();
        this.editor.draw();
        this.onHarmonicsChanged();
    }

    kernelPreset(name) {
        const ke = this.kernelEditor;
        const slW = document.getElementById('slKernelWidth');
        const valW = document.getElementById('valKernelWidth');
        switch (name) {
            case 'box': {
                const w = parseInt(slW.value) || 3;
                ke.width = w;
                for (let i = 0; i < w; i++) ke.weights[i] = 1;
                break;
            }
            case 'triangle': {
                const w = parseInt(slW.value) || 5;
                const nw = w < 3 ? 3 : (w % 2 === 0 ? w + 1 : w);
                ke.width = nw; slW.value = nw; valW.textContent = nw;
                const center = Math.floor(nw / 2);
                for (let i = 0; i < nw; i++) ke.weights[i] = 1 - Math.abs(i - center) / (center + 1);
                break;
            }
            case 'gaussian': {
                const w = parseInt(slW.value) || 5;
                const nw = w < 3 ? 3 : (w % 2 === 0 ? w + 1 : w);
                ke.width = nw; slW.value = nw; valW.textContent = nw;
                const center = Math.floor(nw / 2);
                const sigma = center / 2.5;
                for (let i = 0; i < nw; i++) {
                    const d = i - center;
                    ke.weights[i] = Math.exp(-0.5 * (d * d) / (sigma * sigma));
                }
                break;
            }
            case 'ks': {
                ke.width = 3; slW.value = 3; valW.textContent = 3;
                ke.weights[0] = 0.25; ke.weights[1] = 0.50; ke.weights[2] = 0.25;
                break;
            }
            case 'sharp': {
                ke.width = 3; slW.value = 3; valW.textContent = 3;
                ke.weights[0] = 0.1; ke.weights[1] = 0.8; ke.weights[2] = 0.1;
                break;
            }
            case 'custom': {
                const w = ke.width;
                for (let i = 0; i < w; i++) ke.weights[i] = Math.random();
                break;
            }
        }
        ke.draw();
    }

    onHarmonicsChanged() {
        const wt = this.editor.buildWavetable(WAVETABLE_LEN);
        drawWaveform(document.getElementById('cvWavetable'), wt);
        drawWaveform(document.getElementById('cvBuffer'), wt, '#ff6b6b');
        if (document.getElementById('chkAutoPluck').checked) {
            this._autoPluckTimer && clearTimeout(this._autoPluckTimer);
            this._autoPluckTimer = setTimeout(() => this.pluck(undefined, true), 80);
        }
    }

    preset(name) {
        const a = this.editor.amplitudes;
        const p = this.editor.phases;
        a.fill(0); p.fill(0);
        switch (name) {
            case 'saw':
                for (let i = 0; i < NUM_HARMONICS; i++) a[i] = 1 / (i + 1);
                break;
            case 'square':
                for (let i = 0; i < NUM_HARMONICS; i++) { if ((i + 1) % 2 === 1) a[i] = 1 / (i + 1); }
                break;
            case 'triangle':
                for (let i = 0; i < NUM_HARMONICS; i++) {
                    if ((i + 1) % 2 === 1) {
                        a[i] = 1 / ((i + 1) * (i + 1));
                        if (((i) / 2) % 2 === 1) p[i] = 0.5;
                    }
                }
                break;
            case 'sine': a[0] = 1; break;
            case 'buzz': a.fill(1); break;
            case 'random':
                for (let i = 0; i < NUM_HARMONICS; i++) { a[i] = Math.random(); p[i] = Math.random(); }
                break;
            case 'clear': break;
        }
        let mx = 0;
        for (let i = 0; i < NUM_HARMONICS; i++) mx = Math.max(mx, a[i]);
        if (mx > 1e-12) for (let i = 0; i < NUM_HARMONICS; i++) a[i] /= mx;
        this.editor.draw();
        this.onHarmonicsChanged();
    }

    pluck(voice, ping = false) {
        if (!workletReady) { initAudio(); return; }
        if (audioCtx.state === 'suspended') audioCtx.resume();

        // Voice allocation: use provided voice, or round-robin
        if (voice === undefined) {
            voice = this._nextVoice;
            this._nextVoice = (this._nextVoice + 1) % this._numVoices;
        }

        const note = document.getElementById('selNote').value;
        const cents = parseFloat(document.getElementById('slFineTune').value);
        const freq = noteToFreq(note) * Math.pow(2, cents / 1200);
        const nm = parseFloat(document.getElementById('slNoise').value);
        const vol = parseFloat(document.getElementById('slVolume').value);
        const algo = document.getElementById('selAlgo').value;

        const sr = audioCtx.sampleRate;

        // For Schrödinger, also build the Hilbert wavetable
        // so the wave packet travels around the ring.
        let wavetable, wavetableHilbert = null;
        if (algo === 'schrodinger') {
            const pair = this.editor.buildWavetablePair(WAVETABLE_LEN);
            wavetable = pair.real;
            wavetableHilbert = pair.imag;
        } else {
            wavetable = this.editor.buildWavetable(WAVETABLE_LEN);
        }

        const opts = {
            wavetable, wavetableHilbert,
            frequency: freq, sampleRate: sr, algorithm: algo, noiseMix: nm,
            noiseColor: parseFloat(document.getElementById('slNoiseColor').value),
            lpKernel: this.kernelEditor.getKernel(),
            bufferMode: document.getElementById('selBufferMode').value,
            cycleLPCoeff: parseFloat(document.getElementById('slCycleLP').value),
            cycleHPCoeff: parseFloat(document.getElementById('slCycleHP').value),
        };

        // Gather all algo-specific params from the dynamic registry panel
        const regEntry = (typeof SYNTH_REGISTRY !== 'undefined') && SYNTH_REGISTRY[algo];
        if (regEntry) {
            // Standard params enumerated in entry.params
            for (const p of regEntry.params) {
                const el = document.getElementById('sl_' + p.key);
                if (el) opts[p.key] = p.isInt ? parseInt(el.value) : parseFloat(el.value);
            }
            // For classes with buildUI (RingCouplerX): sweep all dynamic sliders too
            if (regEntry.cls.hasBuildUI) {
                const dynCtrl = document.getElementById('dynamicControls');
                if (dynCtrl) dynCtrl.querySelectorAll('input[type=range]').forEach(el => {
                    const key = el.id.replace(/^sl_/, '');
                    if (opts[key] == null) opts[key] = parseFloat(el.value);
                });
            }
        }

        // Reset visualization state
        this._vizSnapshots = [];
        this._vizPeaks = [];
        this._vizHiddenRings = null;
        this._engineDone = false;
        this._engine = {}; // unique sentinel per pluck for viz loop

        startStreaming(algo, opts, vol, voice);
        // Ping mode: immediately release the gate so ADSR just gets a trigger
        if (ping && workletNode) {
            workletNode.port.postMessage({ type: 'gate', voice, on: false });
        }
        this._startVisualization();

        document.getElementById('statusText').textContent =
            `Playing ${freq.toFixed(1)}Hz, ${algo} (voice ${voice})`;
    }

    _startVisualization() {
        if (this._animFrame) cancelAnimationFrame(this._animFrame);
        if (!this._engine) return;

        const cvBuffer = document.getElementById('cvBuffer');
        const cvVelocity = document.getElementById('cvVelocity');
        const panelVelocity = document.getElementById('panelVelocity');
        const cvFFT = document.getElementById('cvFFT');
        const cvWaterfall = document.getElementById('cvWaterfall');
        const cvEnvelope = document.getElementById('cvEnvelope');
        const cvRCXBs = [0, 1, 2, 3].map(i => document.getElementById(`cvRCXB${i}`));
        const rcxRingColors = ['#5ec4e0', '#e07c5e', '#8ee05e', '#c05ee0'];
        const vizStart = performance.now();
        const VIZ_MAX_MS = 2000;
        let waterfallFrozen = false;
        // Accumulated snapshots for waterfall display
        let allSnapshots = [];
        let allVelSnapshots = [];
        let allPeaks = [];
        const pluckId = this._engine; // unique object ref — identity check detects re-pluck

        const tick = () => {
            if (this._engine !== pluckId) return;

            // Drain new data from worklet messages
            if (this._vizSnapshots.length > 0) {
                for (const s of this._vizSnapshots) allSnapshots.push(s);
                this._vizSnapshots = [];

                // Thin if too many
                if (allSnapshots.length >= 300) {
                    const thinned = [];
                    for (let i = 0; i < allSnapshots.length; i += 2) thinned.push(allSnapshots[i]);
                    allSnapshots = thinned;
                }
            }
            // Drain velocity snapshots
            if (this._vizVelSnapshots.length > 0) {
                for (const vs of this._vizVelSnapshots) allVelSnapshots.push(vs);
                this._vizVelSnapshots = [];
                if (allVelSnapshots.length >= 300) {
                    const thinned = [];
                    for (let i = 0; i < allVelSnapshots.length; i += 2) thinned.push(allVelSnapshots[i]);
                    allVelSnapshots = thinned;
                }
            }
            if (this._vizPeaks.length > 0) {
                for (const p of this._vizPeaks) allPeaks.push(p);
                this._vizPeaks = [];
            }

            // Draw latest snapshot as live buffer + FFT
            if (allSnapshots.length > 0) {
                const latest = allSnapshots[allSnapshots.length - 1];
                drawWaveform(cvBuffer, latest, '#ff6b6b');
                drawFFT(cvFFT, latest);
            }

            // Draw velocity buffer (show/hide panel based on data availability)
            if (allVelSnapshots.length > 0) {
                if (panelVelocity) panelVelocity.style.display = '';
                const latestVel = allVelSnapshots[allVelSnapshots.length - 1];
                drawWaveform(cvVelocity, latestVel, '#6bc5ff');
            }

            // Draw hidden ring waveforms (RingCouplerX only)
            if (this._vizHiddenRings) {
                const rings = this._vizHiddenRings;
                for (let i = 0; i < 4; i++) {
                    if (cvRCXBs[i] && rings[i]) drawWaveform(cvRCXBs[i], rings[i], rcxRingColors[i]);
                }
            }

            if (!waterfallFrozen) {
                if (performance.now() - vizStart > VIZ_MAX_MS) {
                    waterfallFrozen = true;
                }
                drawWaterfall(cvWaterfall, allSnapshots, this._vizBufLen);
                drawEnvelope(cvEnvelope, allPeaks);
            }

            if (!this._engineDone) {
                this._animFrame = requestAnimationFrame(tick);
            } else {
                document.getElementById('statusText').textContent = 'Decayed to silence.';
            }
        };
        this._animFrame = requestAnimationFrame(tick);
    }

    stop() {
        stopStreaming();
        this._engine = null;
        if (this._animFrame) cancelAnimationFrame(this._animFrame);
        document.getElementById('statusText').textContent = 'Stopped.';
    }

}

// ========================================================================
//  INIT
// ========================================================================
let app;
document.addEventListener('DOMContentLoaded', () => {
    ['cvHarmonics', 'cvKernel', 'cvWavetable', 'cvBuffer', 'cvFFT', 'cvWaterfall',
        'cvEnvelope'].forEach(id => {
            const c = document.getElementById(id);
            if (c) setupCanvas(c);
        });

    app = new App();

    window.addEventListener('resize', () => {
        ['cvHarmonics', 'cvKernel', 'cvWavetable', 'cvBuffer', 'cvFFT', 'cvWaterfall',
            'cvEnvelope'].forEach(id => {
                const c = document.getElementById(id);
                if (c) setupCanvas(c);
            });
        app.editor.ctx = app.editor.canvas._cachedDraw.ctx;
        app.editor.w = app.editor.canvas._cachedDraw.w;
        app.editor.h = app.editor.canvas._cachedDraw.h;
        app.editor.draw();
        app.kernelEditor.ctx = app.kernelEditor.canvas._cachedDraw.ctx;
        app.kernelEditor.w = app.kernelEditor.canvas._cachedDraw.w;
        app.kernelEditor.h = app.kernelEditor.canvas._cachedDraw.h;
        app.kernelEditor.draw();
    });
});
