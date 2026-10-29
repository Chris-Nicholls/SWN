"""
Chiptune-style percussion voices: a faithful reimplementation of the NES
2A03 APU's noise channel (15-bit LFSR, two feedback taps for "long"/"metallic"
modes, the documented fixed noise-period table) and its envelope generator
(a genuine linear 16-step staircase decay clocked by the ~240Hz frame
sequencer, NOT a smooth exponential -- that stepping is a big part of the
"chip" character), plus a duty-cycle pulse oscillator for pitched hits.

This is built directly from the public NESdev wiki technical specification
(the APU's behavior is a documented hardware spec, not someone else's code),
not ported from any existing project -- there is no known open-source
Pocket-Operator-style engine to reference (confirmed by research), so this
voice family is our own construction in that spirit: cheap, deliberately
lo-fi/aliased, distinct from the analog-drum-machine emulations already built.
"""
import numpy as np
from scipy.io import wavfile

SR = 44100
APU_CLOCK = 894886.0     # NTSC 2A03 APU clock (CPU clock / 2)
FRAME_SEQ_HZ = 240.0     # quarter-frame envelope clock rate (NTSC 4-step mode)

# Fixed NES noise "period" table (in APU clock ticks) -- selects noise pitch/color.
NTSC_NOISE_PERIODS = [4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068]


def nes_lfsr_noise(n_samples, period_ticks, mode_metallic=False, seed=1, sr=SR):
    """15-bit Galois LFSR noise exactly per NESdev APU Noise spec.
    feedback = bit0 XOR (bit6 if mode_metallic else bit1); shift right; feedback -> bit14.
    Output is silent (0) whenever bit0 is set after the shift, else full level."""
    shift = seed & 0x7FFF
    samples_per_update = sr * period_ticks / APU_CLOCK
    out = np.zeros(n_samples)
    acc = 0.0
    level = 0
    for i in range(n_samples):
        acc += 1.0
        while acc >= samples_per_update:
            acc -= samples_per_update
            bit0 = shift & 1
            tap = (shift >> 6) & 1 if mode_metallic else (shift >> 1) & 1
            feedback = bit0 ^ tap
            shift = (shift >> 1) | (feedback << 14)
            level = 0 if (shift & 1) else 1
        out[i] = 1.0 if level else -1.0
    return out


def nes_envelope(n_samples, period, loop=False, sr=SR):
    """Genuine NES APU envelope: linear 16-step (15->0) staircase decay,
    one step every (period+1) quarter-frame clocks (~240Hz on NTSC).
    This deliberately does NOT smoothly interpolate -- the stepping IS the sound."""
    t = np.arange(n_samples) / sr
    steps_per_sec = FRAME_SEQ_HZ / (period + 1)
    step_idx = np.floor(t * steps_per_sec).astype(np.int64)
    if loop:
        level = (15 - step_idx) % 16
    else:
        level = np.maximum(0, 15 - step_idx)
    return level / 15.0


def pulse_wave(freq_or_arr, n_samples, duty=0.5, sr=SR):
    """NES-style duty-cycle pulse: duty in {0.125, 0.25, 0.5, 0.75}."""
    if np.ndim(freq_or_arr) == 0:
        t = np.arange(n_samples) / sr
        phase = (freq_or_arr * t) % 1.0
    else:
        phase = (np.cumsum(freq_or_arr) / sr) % 1.0
    return np.where(phase < duty, 1.0, -1.0)


def exp_pitch_sweep(n_samples, f_start, f_end, t_sweep, sr=SR):
    t = np.arange(n_samples) / sr
    ratio = f_end / f_start
    frac = np.clip(t / t_sweep, 0, 1)
    freq = f_start * (ratio ** frac)
    return freq


def bitcrush(x, bits=8):
    levels = 2 ** bits
    return np.round(x * (levels / 2 - 1)) / (levels / 2 - 1)


def normalize(x, peak=0.9):
    m = np.max(np.abs(x)) + 1e-12
    return x / m * peak


def write(path, x, sr=SR):
    x = normalize(x)
    wavfile.write(path, sr, (x * 32767).astype(np.int16))
    print("wrote", path)


DUR = 1.2
N = int(DUR * SR)


# ---------------------------------------------------------------------------
def chip_kick():
    n = N
    t = np.arange(n) / SR
    freq = exp_pitch_sweep(n, 320, 48, t_sweep=0.045)
    body = pulse_wave(freq, n, duty=0.5)
    env = nes_envelope(n, period=2)  # fast-ish linear decay
    dur_gate = t <= 0.22
    return body * env * dur_gate


def chip_snare():
    n = N
    noise = nes_lfsr_noise(n, period_ticks=64, mode_metallic=False, seed=1)
    env = nes_envelope(n, period=1)
    t = np.arange(n) / SR
    dur_gate = t <= 0.12
    # a little pulse "body" underneath, very short, for a snare-ish thump
    body = pulse_wave(180.0, n, duty=0.5)
    body_env = nes_envelope(n, period=0)
    body = body * body_env * (t <= 0.02)
    return noise * env * dur_gate * 0.85 + body * 0.3


def chip_closed_hat():
    n = N
    noise = nes_lfsr_noise(n, period_ticks=NTSC_NOISE_PERIODS[2], mode_metallic=True, seed=1)
    env = nes_envelope(n, period=0)
    t = np.arange(n) / SR
    return noise * env * (t <= 0.05)


def chip_open_hat():
    n = N
    noise = nes_lfsr_noise(n, period_ticks=NTSC_NOISE_PERIODS[2], mode_metallic=True, seed=1)
    env = nes_envelope(n, period=3)
    t = np.arange(n) / SR
    return noise * env * (t <= 0.35)


def chip_perc():
    """staccato multi-burst using the noise channel -- a lo-fi clap/rimshot hybrid."""
    n = N
    t = np.arange(n) / SR
    noise = nes_lfsr_noise(n, period_ticks=NTSC_NOISE_PERIODS[5], mode_metallic=False, seed=1)
    out = np.zeros(n)
    offsets = [0.0, 0.014, 0.03, 0.05]
    for off in offsets:
        env = nes_envelope(n, period=0)
        shifted = np.zeros(n)
        start = int(off * SR)
        seg_len = int(0.03 * SR)
        end = min(n, start + seg_len)
        shifted[start:end] = noise[:end - start] * env[:end - start]
        out += shifted * 0.6
    return out


def chip_cowbell():
    """two detuned pulses, NES-metallic-noise dusting, staircase decay -- deliberately toy-ish."""
    n = N
    t = np.arange(n) / SR
    f1, f2 = 540.0, 810.0
    tone = 0.5 * pulse_wave(f1, n, duty=0.5) + 0.5 * pulse_wave(f2, n, duty=0.5)
    env = nes_envelope(n, period=2)
    noise = nes_lfsr_noise(n, period_ticks=NTSC_NOISE_PERIODS[3], mode_metallic=True, seed=2)
    return (tone * 0.8 + noise * 0.2) * env * (t <= 0.2)


VOICES = {
    "chip_kick": chip_kick,
    "chip_snare": chip_snare,
    "chip_chat": chip_closed_hat,
    "chip_ohat": chip_open_hat,
    "chip_perc": chip_perc,
    "chip_cowbell": chip_cowbell,
}

if __name__ == "__main__":
    import os
    outdir = os.path.dirname(os.path.abspath(__file__))
    for name, fn in VOICES.items():
        sig = fn()
        write(os.path.join(outdir, f"py_{name}.wav"), sig)
