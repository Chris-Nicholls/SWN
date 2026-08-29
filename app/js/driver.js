// ========================================================================
//  SYNTHESIS DRIVER (Controller)
//
//  All non-waveguide algorithms operate on a fixed-size buffer (BUFFER_LEN).
//  The driver reads through the buffer at a rate determined by frequency,
//  using linear interpolation. When the read head wraps past the end,
//  advanceCycle() is called — so higher notes trigger the algorithm
//  more often, producing natural pitch-dependent decay.
// ========================================================================

const BUFFER_LEN = 1024;

// --- Helpers ------------------------------------------------------------

/** Generate colored noise: 0 = white, 0.5 ≈ pink, 1 = brown.
 *  Uses a one-pole LP filter on white noise with coefficient derived from color,
 *  then normalises to ±1 so the noise mix amplitude stays consistent. */
function generateColoredNoise(len, color) {
    const c = clamp(color, 0, 1);
    // Map color 0..1 to filter coefficient 0..0.995
    // Use a curve that gives useful midpoint (pink-ish around 0.5)
    const alpha = c * c * 0.995;
    const noise = new Float32Array(len);
    let state = 0;
    for (let i = 0; i < len; i++) {
        const white = Math.random() * 2 - 1;
        state = alpha * state + (1 - alpha) * white;
        noise[i] = state;
    }
    // Normalise peak to ±1 so volume is consistent across colors
    let peak = 0;
    for (let i = 0; i < len; i++) { const a = Math.abs(noise[i]); if (a > peak) peak = a; }
    if (peak > 1e-12) for (let i = 0; i < len; i++) noise[i] /= peak;
    return noise;
}

/** Resample a wavetable into a buffer of given length, with optional noise mix.
 *  noiseColor: 0 = white, 0.5 ≈ pink, 1 = brown (default 0). */
function sampleWavetable(wavetable, bufferLen, noiseMix, noiseColor) {
    const buf = new Float32Array(bufferLen);
    const nm = clamp(noiseMix, 0, 1);
    const noise = nm > 1e-6 ? generateColoredNoise(bufferLen, noiseColor || 0) : null;
    for (let i = 0; i < bufferLen; i++) {
        const phase = i / bufferLen;
        const wtPos = phase * wavetable.length;
        const idx = Math.floor(wtPos);
        const frac = wtPos - idx;
        const s0 = wavetable[idx % wavetable.length];
        const s1 = wavetable[(idx + 1) % wavetable.length];
        let s = s0 + frac * (s1 - s0);
        if (noise) s = s * (1 - nm) + noise[i] * nm;
        buf[i] = s;
    }
    return buf;
}

/** Read from a buffer with linear interpolation (wrapping). */
function interpRead(buf, N, pos) {
    const idx = Math.floor(pos);
    const frac = pos - idx;
    const i0 = ((idx % N) + N) % N;
    const i1 = (i0 + 1) % N;
    return buf[i0] + frac * (buf[i1] - buf[i0]);
}

/** Read with clamped interpolation (no wrap — for open-ended buffers). */
function interpReadClamped(buf, N, pos) {
    const idx = Math.floor(pos);
    const frac = pos - idx;
    const i0 = idx < 0 ? 0 : (idx >= N ? N - 1 : idx);
    const i1 = i0 + 1 < N ? i0 + 1 : N - 1;
    return buf[i0] + frac * (buf[i1] - buf[i0]);
}

/** Compute RMS of a Float32Array. */
function rmsOfBuffer(buf) {
    let sq = 0;
    for (let i = 0; i < buf.length; i++) sq += buf[i] * buf[i];
    return Math.sqrt(sq / buf.length);
}

// --- Decay / reflection helpers -----------------------------------------

function decayTimeToWgRefl(t60, frequency, balance) {
    const t = Math.max(0.01, t60);
    const combined = Math.pow(0.001, 1.0 / (t * frequency));
    const b = clamp(balance, -1, 1);
    const base = Math.sqrt(combined);
    const reflL = Math.pow(base, 1 + b);
    const reflR = Math.pow(base, 1 - b);
    return [clamp(reflL, 0, 1), clamp(reflR, 0, 1)];
}

// --- Algorithm factory --------------------------------------------------
// Creates the synth instance plus metadata for the synthesis loop.

function createSynth(algorithm, opts) {
    const { wavetable, sampleRate, noiseMix, noiseColor } = opts;
    // Pingpong / flip modes double the effective wavelength, so synthesize
    // at double the frequency (one octave up) to compensate.
    const bufMode = opts.bufferMode || 'normal';
    const octaveUp = (bufMode === 'pingpong' || bufMode === 'flip') ? 2 : 1;
    // Shift mode doesn't double the wavelength — it reads forward like normal.
    const frequency = opts.frequency * octaveUp;

    // Waveguide — specialised dual-buffer setup; handle before registry.
    if (algorithm.startsWith('waveguide')) {
        const bufferLen = BUFFER_LEN;
        const step = bufferLen * frequency / sampleRate;
        const isCircular = algorithm === 'waveguide';
        const isFixed = algorithm === 'waveguide_fixed';
        const wgSteps = Math.max(1, Math.round(opts.wgSteps || 4));

        const wt = sampleWavetable(wavetable, bufferLen, noiseMix, noiseColor);
        const R = new Float32Array(bufferLen);
        const L = new Float32Array(bufferLen);
        for (let i = 0; i < bufferLen; i++) {
            L[i] = wt[i] * 0.5;
            R[i] = wt[(i * 3) % bufferLen];
        }

        const buffer = new Float32Array(bufferLen);
        for (let i = 0; i < bufferLen; i++) buffer[i] = R[i] + L[i];

        const [reflL, reflR] = isFixed
            ? decayTimeToWgRefl(opts.wgDecayTime, frequency * wgSteps / bufferLen, opts.wgBalance)
            : [1, 1];

        const synth = new WaveguideSynth(R, L, buffer, {
            isCircular,
            reflL, reflR,
            lpCoeffL: clamp(opts.wgLPL, 0, 1),
            lpCoeffR: clamp(opts.wgLPR, 0, 1),
            clampStrength: isFixed ? clamp(opts.wgClamp, 0, 1) : clamp(opts.wgAP ?? 0.5, 0, 1),
            clampStrengthR: isFixed ? clamp(opts.wgClamp, 0, 1) : clamp(opts.wgAPR ?? 0.5, 0, 1),
            stepsPerCycle: wgSteps,
        });
        return { synth, bufferLen, isDoubleEnded: isFixed, step };
    }

    // All other algorithms: single pre-sampled buffer, registry-driven dispatch.
    const bufferLen = BUFFER_LEN;
    const step = bufferLen * frequency / sampleRate;
    const buffer = sampleWavetable(wavetable, bufferLen, noiseMix, noiseColor);

    // Look up the algorithm in the self-describing registry.
    // Each registered class provides createFromOpts() which handles all
    // instantiation details (extra buffers, parameter mapping, etc.).
    if (typeof SYNTH_REGISTRY !== 'undefined' && SYNTH_REGISTRY[algorithm]) {
        const entry = SYNTH_REGISTRY[algorithm];
        const cls = entry.cls;

        // Collect opts for every declared param, applying range clamping.
        const paramOpts = {};
        for (const p of entry.params) {
            const raw = opts[p.key] ?? p.default;
            paramOpts[p.key] = p.isInt
                ? Math.max(p.min, Math.min(p.max, Math.round(raw)))
                : clamp(raw, p.min, p.max);
        }
        // For classes with buildUI (RingCouplerX), also pass through raw opts
        // since their keys vary and are not enumerated in entry.params.
        if (cls.hasBuildUI) Object.assign(paramOpts, opts);
        if (opts.wavetableHilbert) paramOpts.wavetableHilbert = opts.wavetableHilbert;

        const synth = cls.createFromOpts(buffer, paramOpts, {
            step,
            sampleRate,
            frequency,
            isDoubleEnded: entry.isDoubleEnded,
        });
        // If the synth replaced its buffer with one of a different length
        // (e.g. pitch-adaptive sizing à la the SWN port), recompute the
        // playback step so audible frequency stays correct: step must equal
        // bufferLen * frequency / sampleRate.
        const actualBufferLen = synth.buffer.length;
        const actualStep = actualBufferLen === bufferLen
            ? step
            : (actualBufferLen * frequency / sampleRate);
        return { synth, bufferLen: actualBufferLen, isDoubleEnded: entry.isDoubleEnded, step: actualStep };
    }

    throw new Error(`Unknown algorithm: ${algorithm}`);
}

// --- Streaming synthesis engine -----------------------------------------
//
// Generates audio on demand, one block at a time.
// Tracks snapshots and envelope peaks for live visualization.

class SynthEngine {
    constructor(algorithm, opts) {
        const { synth, bufferLen, isDoubleEnded, step } = createSynth(algorithm, opts);
        this.synth = synth;
        this.bufferLen = bufferLen;
        this.step = step;

        const bufMode = isDoubleEnded ? (opts.bufferMode || 'normal') : 'normal';
        this.symmetric = bufMode === 'pingpong';
        this.flipInvert = bufMode === 'flip';
        this.shiftMode = bufMode === 'shift';
        this.flipSign = 1;
        this.flipOffset = 0;

        this.pos = 0;
        this.ppDir = 1;
        this.distAccum = 0;
        this.cycleIdx = 0;
        this.done = false;
        this._lastSample = 0;

        // For open-ended modes, use clamped interpolation (no wrap around
        // buf[N-1] → buf[0] which creates spikes on open-ended buffers).
        this.useClampedRead = this.symmetric || this.flipInvert || this.shiftMode;

        // DC blocker (first-order high-pass) for modes that introduce DC offset.
        // H(z) = (1 - z^-1) / (1 - R*z^-1), R close to 1 → very low cutoff.
        this.dcBlock = this.symmetric || this.flipInvert || this.shiftMode;
        this._dcX1 = 0;  // previous input
        this._dcY1 = 0;  // previous output
        this._dcR = 0.999;  // pole at ~0.7 Hz @ 44.1k — blocks only true DC drift

        // Attack ramp (1 ms) to avoid click on note onset
        this._attackLen = Math.max(1, Math.round(opts.sampleRate * 0.001));
        this._attackPos = 0;

        // Per-cycle buffer filters (compound over time)
        // LP: one-pole IIR applied forward across the buffer each cycle.
        //   cycleLPCoeff 0 = no filtering, 1 = maximum smoothing.
        // HP: one-pole IIR HP applied forward across the buffer each cycle.
        //   cycleHPCoeff 0 = no filtering, 1 = maximum high-pass.
        this._cycleLPCoeff = clamp(opts.cycleLPCoeff || 0, 0, 1);
        this._cycleHPCoeff = clamp(opts.cycleHPCoeff || 0, 0, 1);

        // Visualization state
        // Optional output soft-clip / compressor (modelled on the SWN
        // firmware's `compress()`, src/compressor.c, configured by
        // init_compressor(COMPRESS_SIGNED_24BIT, 0.90)).  Rescaled to ±1
        // unit-space the curve is:
        //
        //     y = x                              for |x·drive| ≤ thresh
        //     y = sgn(x)·(1 − K/|x·drive|)       for |x·drive| >  thresh
        //
        //   thresh = 0.9
        //   K      = MAX² · thresh · (1 − thresh) = 0.9 · 0.1 = 0.09
        //
        // Below the threshold the path is bit-transparent.  Above it the
        // signal saturates smoothly toward ±1, adding the characteristic
        // odd-harmonic soft-saturation that the firmware imposes on
        // chord-load peaks via master_gain → 24-bit compress() → DAC.
        // Synths opt in by setting `synth.softClipDrive > 0`; default
        // drive is 0 (compressor bypassed) for every algorithm except the
        // SWN-Match variant.
        this._softClipDrive  = +(synth.softClipDrive) || 0;
        this._softClipThresh = 0.9;
        this._softClipK      = this._softClipThresh * (1 - this._softClipThresh);

        this.snapshots = [synth.snapshot()];
        this.envelopePeaks = [];
        this._silentCycles = 0;

        // Cycle-boundary crossfade: when advanceCycle() modifies the buffer,
        // the jump from old buf[wrap] to new buf[wrap] creates a click.
        // We keep the previous buffer and blend over CYCLE_XFADE_LEN samples.
        this._prevBuf = new Float32Array(bufferLen);
        this._cycleXfade = 0;     // remaining crossfade samples (0 = inactive)
        this._cycleXfadeLen = 64;  // ~1.5 ms at 44.1 kHz

        // Pre-allocated ring buffer for capturing snapshots in the audio thread
        // without triggering GC. Each slot is a Float32Array(bufferLen).
        const RING_SIZE = 64;
        this._ring = new Array(RING_SIZE);
        for (let i = 0; i < RING_SIZE; i++) this._ring[i] = new Float32Array(bufferLen);
        this._ringHead = 0;   // next write index (audio thread)
        this._ringTail = 0;   // next read index  (UI thread)
        this._ringSize = RING_SIZE;

        // Velocity ring buffer (same structure, only populated if synth has .v)
        this._hasVelocity = !!synth.velocitySnapshot;
        this._velRing = new Array(RING_SIZE);
        for (let i = 0; i < RING_SIZE; i++) this._velRing[i] = this._hasVelocity ? new Float32Array(bufferLen) : null;
        this.velSnapshots = this._hasVelocity ? [synth.velocitySnapshot()] : [];

        // Pre-allocated ring for envelope peaks (avoids push() on audio thread)
        const PEAK_RING_SIZE = 1024;
        this._peakRing = new Float64Array(PEAK_RING_SIZE);
        this._peakHead = 0;
        this._peakTail = 0;
        this._peakRingSize = PEAK_RING_SIZE;
    }

    /** Fill output[offset .. offset+count) with audio samples. */
    fillBlock(output, offset, count) {
        if (this.done) {
            for (let i = offset; i < offset + count; i++) output[i] = 0;
            return;
        }

        const { synth, bufferLen, step, symmetric, useClampedRead, dcBlock } = this;
        const buf = synth.buffer;
        const read = useClampedRead ? interpReadClamped : interpRead;
        let { pos, ppDir, distAccum, flipSign, flipOffset, _lastSample,
            _dcX1, _dcY1, _dcR } = this;

        const prevBuf = this._prevBuf;
        let cycleXfade = this._cycleXfade;
        const cycleXfadeLen = this._cycleXfadeLen;

        // Soft-clip / compressor locals (zero drive = bypass).
        const softClipDrive  = this._softClipDrive;
        const softClipThresh = this._softClipThresh;
        const softClipK      = this._softClipK;

        for (let i = 0; i < count; i++) {
            // Read sample (with crossfade if we just updated the buffer)
            let sample;
            if (cycleXfade > 0) {
                const t = 1 - cycleXfade / cycleXfadeLen; // 0→1 over window
                const sNew = flipSign * read(buf, bufferLen, pos) + flipOffset;
                const sOld = flipSign * read(prevBuf, bufferLen, pos) + flipOffset;
                sample = sOld + t * (sNew - sOld);
                cycleXfade--;
            } else {
                sample = flipSign * read(buf, bufferLen, pos) + flipOffset;
            }

            // DC blocker: y[n] = x[n] - x[n-1] + R * y[n-1]
            if (dcBlock) {
                const x = sample;
                sample = x - _dcX1 + _dcR * _dcY1;
                _dcX1 = x;
                _dcY1 = sample;
            }

            // Soft-clip / compressor (mirrors src/compressor.c::compress()).
            // Drive scales the input to the threshold so the user can tune
            // how aggressively the saturation engages — drive≈2 simulates
            // a few-voice chord-mode load on the firmware where the post-
            // master-gain signal sits around 2× the 90% threshold.
            if (softClipDrive > 0) {
                const x = sample * softClipDrive;
                if (x > softClipThresh)         sample =  1 - softClipK / x;
                else if (x < -softClipThresh)   sample = -1 - softClipK / x;
                else                            sample = x;
            }

            // Attack ramp (1 ms fade-in)
            if (this._attackPos < this._attackLen) {
                sample *= this._attackPos / this._attackLen;
                this._attackPos++;
            }

            output[offset + i] = sample;
            _lastSample = sample;

            // Advance read head
            if (symmetric) {
                pos += ppDir * step;
                while (pos >= bufferLen || pos < 0) {
                    if (pos >= bufferLen) { pos = 2 * (bufferLen - 1) - pos; ppDir = -1; }
                    if (pos < 0) { pos = -pos; ppDir = 1; }
                }
            } else {
                pos += step;
                while (pos >= bufferLen) pos -= bufferLen;
            }

            // Cycle detection
            distAccum += step;
            while (distAccum >= bufferLen) {
                distAccum -= bufferLen;
                // Write back locals before _endOfCycle (it may modify flipSign/flipOffset)
                this.pos = pos; this.ppDir = ppDir; this.distAccum = distAccum;
                this.flipSign = flipSign; this.flipOffset = flipOffset;
                this._lastSample = _lastSample;
                this._dcX1 = _dcX1; this._dcY1 = _dcY1;
                this._cycleXfade = cycleXfade;
                this._endOfCycle();
                flipSign = this.flipSign; flipOffset = this.flipOffset;
                cycleXfade = this._cycleXfade;
            }
        }
        this.pos = pos; this.ppDir = ppDir; this.distAccum = distAccum;
        this.flipSign = flipSign; this.flipOffset = flipOffset;
        this._lastSample = _lastSample;
        this._dcX1 = _dcX1; this._dcY1 = _dcY1;
        this._cycleXfade = cycleXfade;
    }

    _endOfCycle() {
        this.cycleIdx++;
        const read = this.useClampedRead ? interpReadClamped : interpRead;

        if (this.flipInvert) {
            // Use the actual last output sample (pre-DC-block) for continuity.
            // Note: _lastSample is post-DC-block, so we recompute the raw value.
            const rawLast = this.flipSign *
                read(this.synth.buffer, this.bufferLen,
                    // pos just wrapped, so the sample we just output was near the boundary
                    this.bufferLen - 1) + this.flipOffset;
            this.flipSign = -this.flipSign;
            this.synth.advanceCycle();
            this.flipOffset = rawLast - this.flipSign *
                read(this.synth.buffer, this.bufferLen, this.pos);
        } else if (this.shiftMode) {
            // Shift mode: read forward, but add a DC offset each cycle so that
            // buf[0] of the new cycle continues from buf[N-1] of the old one.
            const rawLast = this.flipSign *
                read(this.synth.buffer, this.bufferLen, this.bufferLen - 1) + this.flipOffset;
            this.synth.advanceCycle();
            this.flipOffset = rawLast - this.flipSign *
                read(this.synth.buffer, this.bufferLen, this.pos);
        } else {
            // Snapshot buffer before mutation for crossfade
            this._prevBuf.set(this.synth.buffer);
            this.synth.advanceCycle();
            this._cycleXfade = this._cycleXfadeLen;
        }

        // --- Apply per-cycle LP and HP filters to the synth buffer ---
        // this._applyCycleFilters();

        // Copy current buffer into the next ring slot (no allocation)
        const head = this._ringHead;
        const slot = this._ring[head % this._ringSize];
        const buf = this.synth.buffer;
        const N = buf.length;
        for (let j = 0; j < N; j++) slot[j] = buf[j];

        // Copy velocity buffer into velocity ring slot (if present)
        if (this._hasVelocity) {
            const vSlot = this._velRing[head % this._ringSize];
            const vBuf = this.synth.v;
            for (let j = 0; j < N; j++) vSlot[j] = vBuf[j];
        }

        this._ringHead = head + 1;

        // Lightweight RMS — no allocation, just iterate the live buffer
        let sq = 0;
        for (let j = 0; j < N; j++) sq += buf[j] * buf[j];
        const rms = Math.sqrt(sq / N);

        // Write RMS into pre-allocated ring (no push / no GC)
        this._peakRing[this._peakHead % this._peakRingSize] = rms;
        this._peakHead++;

        // Auto-stop on silence
        if (rms < 1e-5) {
            this._silentCycles++;
            if (this._silentCycles > 20) this.done = true;
        } else {
            this._silentCycles = 0;
        }
    }

    /** Apply one-pole LP and/or HP filter across the synth buffer in-place.
     *  Because this runs every cycle, the filtering compounds over time —
     *  higher partials decay faster (LP) or the fundamental decays faster (HP).
     *  Both filters run circularly (seeded with buf[N-1]) so no discontinuity
     *  is introduced at the buffer wrap point. */
    // _applyCyc


    /** Called from UI thread to drain the ring buffer into snapshots */
    collectSnapshot() {
        // Drain snapshot ring
        const tail = this._ringTail;
        const head = this._ringHead;
        if (tail !== head) {
            const ringSize = this._ringSize;
            const start = (head - tail > ringSize) ? head - ringSize : tail;
            for (let i = start; i < head; i++) {
                const src = this._ring[i % ringSize];
                this.snapshots.push(new Float32Array(src));
                // Drain velocity ring in lock-step
                if (this._hasVelocity) {
                    const vSrc = this._velRing[i % ringSize];
                    this.velSnapshots.push(new Float32Array(vSrc));
                }
            }
            this._ringTail = head;

            if (this.snapshots.length >= 300) {
                const thinned = [];
                const vThinned = this._hasVelocity ? [] : null;
                for (let i = 0; i < this.snapshots.length; i += 2) {
                    thinned.push(this.snapshots[i]);
                    if (vThinned) vThinned.push(this.velSnapshots[i]);
                }
                this.snapshots = thinned;
                if (vThinned) this.velSnapshots = vThinned;
            }
        }

        // Drain peak ring
        const pTail = this._peakTail;
        const pHead = this._peakHead;
        if (pTail !== pHead) {
            const pSize = this._peakRingSize;
            const pStart = (pHead - pTail > pSize) ? pHead - pSize : pTail;
            for (let i = pStart; i < pHead; i++) {
                this.envelopePeaks.push(this._peakRing[i % pSize]);
            }
            this._peakTail = pHead;
        }
    }

    snapshot() { return this.synth.snapshot(); }

    /** Returns hidden ring snapshots if the synth supports it (e.g. RingCouplerX) */
    hiddenRingSnap() {
        return this.synth.hiddenSnapshot ? this.synth.hiddenSnapshot() : null;
    }
}
