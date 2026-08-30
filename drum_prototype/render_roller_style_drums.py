"""
Independent Python reimplementation of the ROLLER (Synth_DrumAndBass) drum
voices: kick, snare, closed/open hat, ride, crash, perc/tom, rimshot.
Written from scratch translating the Web Audio automation curves
(setValueAtTime/linearRamp/exponentialRamp + BiquadFilterNode formulas) into
plain numpy, using the exact RBJ/Web-Audio-spec biquad coefficient formulas.
Not copied code -- our own expression of the same DSP recipe, for comparison
against a locally-run reference render of the actual source.
"""
import numpy as np
from scipy.io import wavfile

SR = 44100


# ---- Web Audio-spec biquad filters (RBJ cookbook coefficients) ----------
def _biquad_coeffs(kind, freq, q, sr=SR):
    w0 = 2 * np.pi * freq / sr
    alpha = np.sin(w0) / (2 * q)
    cosw0 = np.cos(w0)
    if kind == "lowpass":
        b0 = (1 - cosw0) / 2
        b1 = 1 - cosw0
        b2 = (1 - cosw0) / 2
    elif kind == "highpass":
        b0 = (1 + cosw0) / 2
        b1 = -(1 + cosw0)
        b2 = (1 + cosw0) / 2
    elif kind == "bandpass":
        b0 = alpha
        b1 = 0.0
        b2 = -alpha
    else:
        raise ValueError(kind)
    a0 = 1 + alpha
    a1 = -2 * cosw0
    a2 = 1 - alpha
    return b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0


def biquad(x, kind, freq, q, sr=SR):
    """Static-coefficient biquad (freq/q constant over the buffer)."""
    b0, b1, b2, a1, a2 = _biquad_coeffs(kind, freq, q, sr)
    y = np.zeros_like(x)
    x1 = x2 = y1 = y2 = 0.0
    for i in range(len(x)):
        yy = b0 * x[i] + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
        x2, x1 = x1, x[i]
        y2, y1 = y1, yy
        y[i] = yy
    return y


def biquad_swept(x, kind, freq_arr, q, sr=SR):
    """Time-varying cutoff: recompute coefficients per sample (freq_arr is an array)."""
    y = np.zeros_like(x)
    x1 = x2 = y1 = y2 = 0.0
    for i in range(len(x)):
        b0, b1, b2, a1, a2 = _biquad_coeffs(kind, freq_arr[i], q, sr)
        yy = b0 * x[i] + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
        x2, x1 = x1, x[i]
        y2, y1 = y1, yy
        y[i] = yy
    return y


# ---- Web Audio AudioParam automation helpers -----------------------------
def exp_ramp(n_total, sr, segments):
    """segments: list of (t_start, t_end, v_start, v_end); exponential ramp per segment,
    holds last value afterward. Returns an array of length n_total (samples)."""
    out = np.zeros(n_total)
    t = np.arange(n_total) / sr
    last_val = segments[0][2]
    for (t0, t1, v0, v1) in segments:
        mask = (t >= t0) & (t <= t1)
        if t1 > t0:
            frac = (t[mask] - t0) / (t1 - t0)
            out[mask] = v0 * (v1 / v0) ** frac
        last_val = v1
    out[t > segments[-1][1]] = last_val
    out[t < segments[0][0]] = segments[0][2]
    return out


def linear_ramp(n_total, sr, t0, t1, v0, v1):
    out = np.zeros(n_total)
    t = np.arange(n_total) / sr
    mask = (t >= t0) & (t <= t1)
    frac = (t[mask] - t0) / (t1 - t0)
    out[mask] = v0 + (v1 - v0) * frac
    return out


def envelope_lin_then_exp(n, sr, t_attack, v_peak, t_decay_end, v_floor=0.0008, v_start=0.0):
    """setValueAtTime(0)->linearRamp(peak, t_attack)->exponentialRamp(floor, t_decay_end)"""
    out = np.zeros(n)
    t = np.arange(n) / sr
    m1 = t <= t_attack
    out[m1] = v_start + (v_peak - v_start) * (t[m1] / t_attack if t_attack > 0 else 1.0)
    m2 = (t > t_attack) & (t <= t_decay_end)
    frac = (t[m2] - t_attack) / (t_decay_end - t_attack)
    out[m2] = v_peak * (v_floor / v_peak) ** frac
    out[t > t_decay_end] = v_floor
    return out


def envelope_instant_exp(n, sr, v_peak, t_decay_end, v_floor=0.0008):
    """setValueAtTime(v_peak)->exponentialRampToValueAtTime(floor, t_decay_end)"""
    out = np.zeros(n)
    t = np.arange(n) / sr
    m = t <= t_decay_end
    frac = t[m] / t_decay_end
    out[m] = v_peak * (v_floor / v_peak) ** frac
    out[t > t_decay_end] = v_floor
    return out


def sine(freq_or_arr, n, sr=SR):
    if np.ndim(freq_or_arr) == 0:
        t = np.arange(n) / sr
        return np.sin(2 * np.pi * freq_or_arr * t)
    phase = np.cumsum(freq_or_arr) / sr
    return np.sin(2 * np.pi * phase)


def triangle(freq, n, sr=SR):
    t = np.arange(n) / sr
    ph = (freq * t) % 1.0
    return 2 * np.abs(2 * ph - 1) - 1


def triangle_from_freq(freq_arr, n, sr=SR):
    ph = np.cumsum(freq_arr) / sr % 1.0
    return 2 * np.abs(2 * ph - 1) - 1


def square(freq, n, sr=SR):
    t = np.arange(n) / sr
    return np.sign(np.sin(2 * np.pi * freq * t + 1e-9))


def noise(n, seed=None):
    rng = np.random.default_rng(seed)
    return rng.uniform(-1, 1, n)


def tanh_shape(x, k=1.6):
    d = np.tanh(k)
    return np.tanh(x * k) / d


def normalize(x, peak=0.9):
    m = np.max(np.abs(x)) + 1e-12
    return x / m * peak


def write(path, x, sr=SR):
    x = normalize(x)
    wavfile.write(path, sr, (x * 32767).astype(np.int16))
    print("wrote", path)


DUR_S = 3.0
N = int(DUR_S * SR)


# ---------------------------------------------------------------------------
def synth_kick(v=1.0, ktune=49, kdec=0.34, k=0.0):
    n = N
    f0 = ktune * (1 + k * 0.022)
    freq = exp_ramp(n, SR, [
        (0, 0.022, f0 * 4.4, f0 * 1.25),
        (0.022, 0.09, f0 * 1.25, f0),
        (0.09, kdec, f0, f0 * 0.93),
    ])
    body_osc = sine(freq, n)
    env = envelope_lin_then_exp(n, SR, t_attack=0.002, v_peak=1.25 * v, t_decay_end=kdec)
    t_full = np.arange(n) / SR
    body = tanh_shape(body_osc * env, 1.6) * (t_full <= kdec + 0.12)

    beater = triangle(980 * (1 + k * 0.07), n)
    beater_env = envelope_instant_exp(n, SR, 0.5 * v, 0.009)
    t = np.arange(n) / SR
    beater = beater * beater_env * (t <= 0.02)

    air = noise(n, seed=1)
    air = biquad(air, "highpass", 2600, 0.7)
    air_env = envelope_instant_exp(n, SR, 0.36 * v, 0.014)
    air = air * air_env * (t <= 0.016)

    return body + beater + air


def synth_snare(v=1.0, sdec=0.17, stune=1.0, snap=0.62, k=0.0):
    n = N
    t = np.arange(n) / SR
    ghost = v < 0.45
    dec = sdec * (0.5 if ghost else 1) * (1 + k * 0.13)
    tune = stune * (1 + k * 0.035)
    shellV = 0.52 * v * (0.34 if ghost else 1)
    shellD = 0.024 if ghost else 0.058

    shell = np.zeros(n)
    for ix, f in enumerate([178 * tune, 268 * tune]):
        freq = exp_ramp(n, SR, [(0, 0.02, f * 1.18, f)])
        osc = triangle_from_freq(freq, n) if ix else sine(freq, n)
        env = envelope_instant_exp(n, SR, shellV * (0.7 if ix else 1.0), shellD)
        env = env * (t <= shellD + 0.02)
        shell += osc * env

    wires_noise = noise(n, seed=2)
    wires = biquad(wires_noise, "bandpass", 1900 * tune * (1 + k * 0.06), 0.62)
    wires = biquad(wires, "highpass", 620, 0.7)
    wires_env = envelope_lin_then_exp(n, SR, t_attack=0.002, v_peak=0.95 * v, t_decay_end=dec)
    wires = wires * wires_env * (t <= dec + 0.05)

    crack = np.zeros(n)
    if snap > 0.01:
        crack_noise = noise(n, seed=3)
        crack = biquad(crack_noise, "highpass", 6200 * (1 + k * 0.05), 0.8)
        crack_dec = 0.018 + 0.014 * snap
        crack_env = envelope_instant_exp(n, SR, 0.62 * v * snap, crack_dec)
        crack = crack * crack_env * (t <= 0.03)

    return shell + wires + crack


def _metal(v, dec, bpF, hpF, ratio, noise_mix, seed):
    n = N
    t = np.arange(n) / SR
    squares = np.zeros(n)
    for f in [263, 400, 421, 474, 587, 845]:
        squares += square(f * ratio, n)
    squares = biquad(squares, "bandpass", bpF, 0.9)
    squares = biquad(squares, "highpass", hpF, 0.7)

    combined = squares
    if noise_mix:
        nz = noise(n, seed=seed)
        nz = biquad(nz, "highpass", hpF * 1.05, 0.7)
        combined = combined + noise_mix * nz * (t <= dec + 0.02)

    env = envelope_lin_then_exp(n, SR, t_attack=0.0015, v_peak=v, t_decay_end=dec, v_floor=0.0006)
    return combined * env * (t <= dec + 0.04)


def synth_closed_hat(v=1.0, k=0.0):
    dec = 0.0345 * (1 + k * 0.22)  # mean of (0.03 + 0.009*rand())
    return _metal(0.7 * v, dec, 9000 * (1 + k * 0.06), 6400, 2.15 * (1 + k * 0.03), 0.6, seed=10)


def synth_open_hat(v=1.0, hdec=0.26, k=0.0):
    dec = hdec * (1 + k * 0.16)
    return _metal(0.6 * v, dec, 8200 * (1 + k * 0.05), 5800, 2.1 * (1 + k * 0.03), 0.5, seed=11)


def synth_ride(v=1.0, k=0.0):
    n = N
    t = np.arange(n) / SR
    body = _metal(0.2 * v, 0.7 * (1 + k * 0.14), 3600 * (1 + k * 0.05), 1400, 1.75 * (1 + k * 0.025), 0.22, seed=12)
    bow_osc = sine(2900 * (1 + k * 0.04), n)
    bow_env = envelope_instant_exp(n, SR, 0.06 * v, 0.14, v_floor=0.0006)
    bow = bow_osc * bow_env * (t <= 0.16)
    return body + bow


def synth_crash(v=1.0):
    n = N
    t = np.arange(n) / SR
    nz = noise(n, seed=13)
    nz = biquad(nz, "highpass", 4600, 0.6)
    cutoff = exp_ramp(n, SR, [(0, 1.7, 15000, 4200)])
    nz = biquad_swept(nz, "lowpass", cutoff, 0.7)
    wash_env = envelope_lin_then_exp(n, SR, t_attack=0.004, v_peak=0.5 * v, t_decay_end=1.8, v_floor=0.0006)
    wash = nz * wash_env * (t <= 1.9)

    metallic = _metal(0.16 * v, 1.3, 7800, 5200, 2.6, 0.0, seed=14)
    return wash + metallic


def synth_perc(v=1.0, k=0.0):
    n = N
    t = np.arange(n) / SR
    f0 = (430 if v < 0.45 else (250 if v > 0.85 else 320)) * (1 + k * 0.05)
    body_lvl = 0.78 * v * v
    dec = 0.05 + 0.07 * v

    freq = exp_ramp(n, SR, [(0, 0.03, f0 * 2.1, f0)])
    body_osc = sine(freq, n)
    body_env = envelope_lin_then_exp(n, SR, t_attack=0.002, v_peak=body_lvl, t_decay_end=dec)
    body = body_osc * body_env * (t <= dec + 0.04)

    nz = noise(n, seed=15)
    nz = biquad(nz, "bandpass", 3200, 1.4)
    nz_env = envelope_instant_exp(n, SR, 0.34 * v, 0.035)
    nz = nz * nz_env * (t <= 0.04)

    return body + nz


def synth_rimshot(v=1.0, k=0.0):
    n = N
    t = np.arange(n) / SR
    nz = noise(n, seed=16)
    nz = biquad(nz, "bandpass", 1750 * (1 + k * 0.08), 7)
    nz_env = envelope_instant_exp(n, SR, 0.8 * v, 0.022)
    nz = nz * nz_env * (t <= 0.03)

    click = square(860 * (1 + k * 0.06), n)
    click_env = envelope_instant_exp(n, SR, 0.3 * v, 0.008)
    click = click * click_env * (t <= 0.02)

    return nz + click


VOICES = {
    "roller_kick": synth_kick,
    "roller_snare": synth_snare,
    "roller_chat": synth_closed_hat,
    "roller_ohat": synth_open_hat,
    "roller_ride": synth_ride,
    "roller_crash": synth_crash,
    "roller_perc": synth_perc,
    "roller_rim": synth_rimshot,
}

if __name__ == "__main__":
    import os
    outdir = os.path.dirname(os.path.abspath(__file__))
    for name, fn in VOICES.items():
        sig = fn()
        write(os.path.join(outdir, f"py_{name}.wav"), sig)
