"""
Independent Python reimplementation of the mpump-style 808/909 drum voice
algorithms (kick/snare/hats/cymbals/clap/cowbell/tom/rimshot), written from
scratch to match the same DSP recipe (sine sweeps, exponential envelopes,
biquad bandpass "wire"/"ring" resonances, inharmonic partial banks).
Not copied from any AGPL source -- this is our own expression of the
technique, for comparison against a locally-run reference render.
"""
import numpy as np
from scipy.io import wavfile

SR = 44100


def tune_ratio(semi):
    return 2 ** (semi / 12.0)


def seeded_noise(n, seed):
    rng = np.random.default_rng(seed)
    return rng.uniform(-1, 1, n)


def biquad_bandpass(x, freq, q, sr=SR):
    w0 = 2 * np.pi * freq / sr
    alpha = np.sin(w0) / (2 * q)
    b0 = alpha
    a0 = 1 + alpha
    a1 = -2 * np.cos(w0)
    a2 = 1 - alpha
    y = np.zeros_like(x)
    x1 = x2 = y1 = y2 = 0.0
    for i in range(len(x)):
        yy = (b0 * x[i] - b0 * x2 - a1 * y1 - a2 * y2) / a0
        x2, x1 = x1, x[i]
        y2, y1 = y1, yy
        y[i] = yy
    return y


def fade_out(x, ms=5, sr=SR):
    n = int(sr * ms / 1000)
    n = min(n, len(x))
    if n <= 0:
        return x
    ramp = np.linspace(1, 0, n)
    x = x.copy()
    x[-n:] *= ramp
    return x


def normalize(x, peak=0.9):
    m = np.max(np.abs(x)) + 1e-12
    return x / m * peak


def write(path, x, sr=SR):
    x = normalize(x)
    wavfile.write(path, sr, (x * 32767).astype(np.int16))
    print("wrote", path)


# ---------------------------------------------------------------------------
def synth_kick(tune=0, decay=1.0, click_amt=0.15, sweep_depth=0.5, sweep_rate=0.5, click_tune=0.0):
    r = tune_ratio(tune)
    n = int(min(0.6 * decay, 2.0) * SR)
    t = np.arange(n) / SR

    base_f = 45 * r
    sweep = (80 + 170 * sweep_depth) * r
    s_rate = (20 + 70 * sweep_rate) / max(decay, 0.5)

    phase = 2 * np.pi * (base_f * t + (sweep / s_rate) * (1 - np.exp(-t * s_rate)))
    body_attack = np.exp(-t * 200)
    body_tail = np.exp(-t * (5 / decay))
    body = np.sin(phase) * (body_attack * 0.55 + body_tail * 0.12) * 0.95
    sub = np.sin(2 * np.pi * 50 * r * t) * np.exp(-t * (5 / decay)) * 0.4

    f1 = 2000 * 2 ** click_tune
    f2 = 5000 * 2 ** click_tune
    click = (np.sin(2 * np.pi * f1 * t) + np.sin(2 * np.pi * f2 * t)) * 0.5 * np.exp(-t * 2000) * click_amt

    return body + sub + click


def synth_snare(tune=0, decay=1.0, noise_mix=0.55):
    r = tune_ratio(tune)
    n = int(min(0.3 * decay, 2.0) * SR)
    t = np.arange(n) / SR
    tone_level = 1.0 * (1 - noise_mix)
    noise_level = 1.0 * noise_mix

    raw_noise = seeded_noise(n, seed=38)
    shaped = biquad_bandpass(raw_noise, 3800 * r, 3)

    pitch_env = 1 + 0.5 * np.exp(-t * 60)
    body = np.sin(2 * np.pi * 185 * r * pitch_env * t) * np.exp(-t * (18 / decay)) * tone_level
    low = np.sin(2 * np.pi * 110 * r * t) * np.exp(-t * (22 / decay)) * (tone_level * 0.2)
    noise_env = np.exp(-t * (14 / decay))
    noise = (raw_noise * 0.45 + shaped * 0.55) * noise_env * noise_level

    return body + low + noise


def _hat_like(tune, decay, color, freqs, amps, transient_rate, transient_amp,
              noise_rate, noise_amp, ring_rate, seed, n_seconds, max_seconds):
    r = tune_ratio(tune)
    shift = 2 ** (color * 0.5)
    n = int(min(n_seconds * decay, max_seconds) * SR)
    t = np.arange(n) / SR
    raw = seeded_noise(n, seed)
    prev = np.concatenate([[0], raw[:-1]])
    transient = np.exp(-t * transient_rate) * transient_amp
    noise = (raw - prev) * np.exp(-t * (noise_rate / decay)) * noise_amp
    ring = np.zeros(n)
    for f, a in zip(freqs, amps):
        ring += np.sin(2 * np.pi * f * shift * r * t) * a
    ring *= np.exp(-t * (ring_rate / decay))
    return transient * raw + noise + ring


def synth_closed_hat(tune=0, decay=1.0, color=0.0):
    freqs = [3500, 5200, 7500, 4100, 6300, 8800]
    amps = [0.06, 0.04, 0.08, 0.04, 0.06, 0.02]
    return _hat_like(tune, decay, color, freqs, amps,
                      transient_rate=1000, transient_amp=0.25,
                      noise_rate=50, noise_amp=0.45,
                      ring_rate=120, seed=42, n_seconds=0.08, max_seconds=1.0)


def synth_open_hat(tune=0, decay=1.0, color=0.0):
    freqs = [3500, 5200, 7500, 4100, 6300, 8800]
    amps = [0.10, 0.07, 0.12, 0.07, 0.09, 0.04]
    return _hat_like(tune, decay, color, freqs, amps,
                      transient_rate=600, transient_amp=0.18,
                      noise_rate=6, noise_amp=0.35,
                      ring_rate=5, seed=46, n_seconds=0.3, max_seconds=2.0)


def synth_crash(tune=0, decay=1.0, color=0.0):
    r = tune_ratio(tune)
    shift = 2 ** (color * 0.5)
    n = int(min(1.0 * decay, 3.0) * SR)
    t = np.arange(n) / SR
    raw = seeded_noise(n, seed=49)
    prev = np.concatenate([[0], raw[:-1]])
    transient = np.exp(-t * 300) * 0.35
    noise = (raw - prev) * np.exp(-t * (3 / decay)) * 0.40
    freqs = [3200, 5000, 6800, 8500, 11000]
    amps = [0.08, 0.10, 0.08, 0.06, 0.04]
    ring = np.zeros(n)
    for p, (f, a) in enumerate(zip(freqs, amps)):
        ring += np.sin(2 * np.pi * f * shift * r * t) * a * np.exp(-t * ((3 + p) / decay))
    return transient * raw + noise + ring


def synth_ride(tune=0, decay=1.0, color=0.0):
    r = tune_ratio(tune)
    shift = 2 ** (color * 0.5)
    n = int(min(0.6 * decay, 3.0) * SR)
    t = np.arange(n) / SR
    raw = seeded_noise(n, seed=51)
    prev = np.concatenate([[0], raw[:-1]])
    stick = np.exp(-t * 400) * 0.18
    noise = (raw - prev) * 0.35 * np.exp(-t * (5 / decay))
    freqs = [392, 1200, 2800, 4600, 6200, 8500]
    amps = [0.04, 0.05, 0.05, 0.04, 0.03, 0.02]
    ring = np.zeros(n)
    for f, a in zip(freqs, amps):
        ring += np.sin(2 * np.pi * f * shift * r * t) * a
    ring *= np.exp(-t * (8 / decay))
    return stick * raw + noise + ring


def synth_rimshot(tune=0, decay=1.0):
    r = tune_ratio(tune)
    n = int(min(0.04 * decay, 1.0) * SR)
    t = np.arange(n) / SR
    noise = seeded_noise(n, seed=37)
    tone1 = np.sin(2 * np.pi * 920 * r * t) * 0.3
    tone2 = np.sin(2 * np.pi * 1600 * r * t) * 0.2 * np.exp(-t * (100 / decay))
    return (tone1 + tone2 + noise * 0.15) * np.exp(-t * (80 / decay))


def synth_clap(tune=0, decay=1.0):
    r = tune_ratio(tune)
    n = int(min(0.25 * decay, 2.0) * SR)
    t = np.arange(n) / SR
    rng = np.random.default_rng(50)
    raw_noise = seeded_noise(n, seed=50)
    offsets = [0, 0.008 + rng.random() * 0.004, 0.018 + rng.random() * 0.006, 0.03 + rng.random() * 0.005]
    bursts = np.zeros(n)
    for off in offsets:
        bt = t - off
        mask = bt >= 0
        bursts[mask] += np.exp(-bt[mask] * (35 / decay)) * 0.5
    raw = raw_noise * bursts
    shaped = biquad_bandpass(raw, 3200 * r, 3)
    return raw * 0.5 + shaped * 0.5


def synth_tom(tune=0, decay=1.0):
    r = tune_ratio(tune)
    n = int(min(0.25 * decay, 2.0) * SR)
    t = np.arange(n) / SR
    base_f, sweep, sweep_rate = 200 * r, 80 * r, 25 / decay
    phase = 2 * np.pi * (base_f * t + (sweep / sweep_rate) * (1 - np.exp(-t * sweep_rate)))
    body = np.sin(phase) * np.exp(-t * (12 / decay)) * 0.7
    click = np.sin(2 * np.pi * 5000 * t) * np.exp(-t * 2500) * 0.08
    return body + click


def synth_cowbell(tune=0, decay=1.0):
    r = tune_ratio(tune)
    n = int(min(0.15 * decay, 1.0) * SR)
    t = np.arange(n) / SR
    env = np.exp(-t * (20 / decay))
    f1, f2 = 545 * r, 815 * r
    raw = (np.sign(np.sin(2 * np.pi * f1 * t)) * 0.22 + np.sign(np.sin(2 * np.pi * f2 * t)) * 0.22) * env
    shaped = biquad_bandpass(raw, 800 * r, 4)
    return raw * 0.6 + shaped * 0.4


VOICES = {
    "kick": synth_kick,
    "snare": synth_snare,
    "closedhat": synth_closed_hat,
    "openhat": synth_open_hat,
    "crash": synth_crash,
    "ride": synth_ride,
    "rimshot": synth_rimshot,
    "clap": synth_clap,
    "tom": synth_tom,
    "cowbell": synth_cowbell,
}

if __name__ == "__main__":
    import os
    outdir = os.path.dirname(os.path.abspath(__file__))
    for name, fn in VOICES.items():
        sig = fade_out(fn())
        write(os.path.join(outdir, f"py_{name}.wav"), sig)
