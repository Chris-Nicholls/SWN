// ========================================================================
//  AudioWorklet Processor
//
//  Loaded as a Blob combined with synths.js + driver.js so that
//  SynthEngine (and all synth classes) are available in the worklet scope.
//  Runs on a dedicated real-time audio thread — no main-thread contention.
// ========================================================================

const XFADE_LEN = 128; // ~3ms crossfade at 44.1kHz to avoid clicks on retrigger
const NUM_VOICES = 3;

class SynthProcessor extends AudioWorkletProcessor {
    constructor() {
        super();
        // 3-voice polyphony: each slot holds an engine or null
        this.voices = new Array(NUM_VOICES).fill(null);
        // Per-voice fade-out engines for crossfade on retrigger
        this.fadingOut = new Array(NUM_VOICES).fill(null);
        this.fadePos = new Array(NUM_VOICES).fill(0);
        this._fadeBuffers = [];
        for (let i = 0; i < NUM_VOICES; i++) this._fadeBuffers.push(new Float32Array(128));
        this._snapshotCounter = 0;
        this._mixBuf = new Float32Array(128);
        this._lastStartedVoice = 0;

        this.port.onmessage = (e) => {
            const { type } = e.data;
            if (type === 'start') {
                const voice = e.data.voice ?? 0;
                const v = Math.max(0, Math.min(NUM_VOICES - 1, voice));

                // Crossfade: move current engine to fadingOut
                if (this.voices[v] && !this.voices[v].done) {
                    this.fadingOut[v] = this.voices[v];
                    this.fadePos[v] = 0;
                }
                // Reconstruct typed arrays from transferred data
                const opts = e.data.opts;
                if (opts.wavetable && !(opts.wavetable instanceof Float32Array)) {
                    opts.wavetable = new Float32Array(opts.wavetable);
                }
                if (opts.lpKernel && !(opts.lpKernel instanceof Float64Array)) {
                    opts.lpKernel = new Float64Array(opts.lpKernel);
                }
                this.voices[v] = new SynthEngine(e.data.algorithm, opts);
                this._lastStartedVoice = v;
            } else if (type === 'stop') {
                const voice = e.data.voice;
                if (voice !== undefined) {
                    // Stop a specific voice
                    const v = Math.max(0, Math.min(NUM_VOICES - 1, voice));
                    if (this.voices[v]) {
                        this.fadingOut[v] = this.voices[v];
                        this.fadePos[v] = 0;
                    }
                    this.voices[v] = null;
                } else {
                    // Stop all voices
                    for (let v = 0; v < NUM_VOICES; v++) {
                        if (this.voices[v]) {
                            this.fadingOut[v] = this.voices[v];
                            this.fadePos[v] = 0;
                        }
                        this.voices[v] = null;
                    }
                }
            } else if (type === 'gate') {
                const voice = e.data.voice;
                const on = e.data.on;
                if (voice !== undefined) {
                    const v = Math.max(0, Math.min(NUM_VOICES - 1, voice));
                    if (!on && this.voices[v] && this.voices[v].synth && this.voices[v].synth.noteOff) {
                        this.voices[v].synth.noteOff();
                    }
                } else if (!on) {
                    for (let v = 0; v < NUM_VOICES; v++) {
                        if (this.voices[v] && this.voices[v].synth && this.voices[v].synth.noteOff) {
                            this.voices[v].synth.noteOff();
                        }
                    }
                }
            } else if (type === 'reseed') {
                for (let v = 0; v < NUM_VOICES; v++) {
                    if (this.voices[v] && this.voices[v].synth && this.voices[v].synth.reseed) {
                        this.voices[v].synth.reseed(e.data.seed);
                    }
                }
            }
        };
    }

    process(inputs, outputs) {
        const out = outputs[0][0];
        if (!out) return true;
        const len = out.length;

        // Clear output
        out.fill(0);

        // Sum all active voices
        for (let v = 0; v < NUM_VOICES; v++) {
            const engine = this.voices[v];
            if (engine && !engine.done) {
                // Reuse _mixBuf to generate this voice's audio
                const mb = this._mixBuf;
                if (mb.length < len) this._mixBuf = new Float32Array(len);
                engine.fillBlock(this._mixBuf, 0, len);
                for (let i = 0; i < len; i++) out[i] += this._mixBuf[i];
            }

            // Crossfade old engine out to avoid clicks on retrigger
            if (this.fadingOut[v]) {
                const fb = this._fadeBuffers[v];
                if (!this.fadingOut[v].done) {
                    this.fadingOut[v].fillBlock(fb, 0, len);
                } else {
                    for (let i = 0; i < len; i++) fb[i] = 0;
                }
                const remaining = XFADE_LEN - this.fadePos[v];
                const crossLen = Math.min(len, remaining);
                for (let i = 0; i < crossLen; i++) {
                    const t = (this.fadePos[v] + i) / XFADE_LEN;
                    // Subtract what we already added for this voice, add crossfaded version
                    out[i] += fb[i] * (1 - t);
                }
                this.fadePos[v] += len;
                if (this.fadePos[v] >= XFADE_LEN) this.fadingOut[v] = null;
            }
        }

        // Scale down to prevent clipping (simple 1/N_VOICES mix)
        const gain = 1 / NUM_VOICES;
        for (let i = 0; i < len; i++) out[i] *= gain;

        // Post visualization data periodically (~20fps)
        this._snapshotCounter++;
        if (this._snapshotCounter >= 17) {
            this._snapshotCounter = 0;
            this._postViz();
        }

        return true;
    }

    _postViz() {
        // Use the most recently started voice for visualization
        let engine = null;
        const lv = this._lastStartedVoice;
        if (this.voices[lv] && !this.voices[lv].done) {
            engine = this.voices[lv];
        } else {
            for (let v = 0; v < NUM_VOICES; v++) {
                if (this.voices[v] && !this.voices[v].done) { engine = this.voices[v]; break; }
            }
        }
        if (!engine) return;

        // Drain ring buffers
        engine.collectSnapshot();

        const nSnaps = engine.snapshots.length;
        const nPeaks = engine.envelopePeaks.length;
        if (nSnaps === 0 && nPeaks === 0 && !engine.done) return;

        const allDone = this.voices.every(v => !v || v.done);
        const hiddenRings = engine.hiddenRingSnap ? engine.hiddenRingSnap() : null;
        const nVelSnaps = engine.velSnapshots ? engine.velSnapshots.length : 0;
        this.port.postMessage({
            type: 'viz',
            snapshots: engine.snapshots,
            velSnapshots: nVelSnaps > 0 ? engine.velSnapshots : null,
            peaks: engine.envelopePeaks,
            done: allDone,
            bufferLen: engine.bufferLen,
            hiddenRings,
        });

        engine.snapshots = nSnaps > 0
            ? [engine.snapshots[nSnaps - 1]]
            : [];
        if (nVelSnaps > 0) {
            engine.velSnapshots = [engine.velSnapshots[nVelSnaps - 1]];
        }
        engine.envelopePeaks = [];
    }
}

registerProcessor('synth-processor', SynthProcessor);
