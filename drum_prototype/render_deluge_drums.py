"""
Offline prototype renderer for the Deluge "Synth.XML" kit drum sounds.

Goal: sanity-check the *topology* of each drum voice (ring-mod kick,
subtractive noise+ladder-filter snare/hats/cymbal, chained-FM cowbell)
before porting any of it to the SWN firmware. Envelope/filter curve
mappings are reasonable perceptual approximations of Deluge's real
lookup-table curves, not bit-exact reproductions -- good enough to judge
"does this sound like a kick/snare/hat/cowbell/cymbal", not to match
the hardware sample-for-sample.
"""
import numpy as np
from scipy.io import wavfile

SR = 44100


def h2s(hexstr):
    """hex string -> signed 32-bit int"""
    v = int(hexstr, 16)
    if v >= 0x80000000:
        v -= 0x100000000
    return v


def uni(hexstr):
    """unipolar knob 0..1 (deluge convention: 0x80000000=min, 0x7FFFFFFF=max)"""
    v = h2s(hexstr)
    return (v + 2147483648) / 4294967295.0


def bi(hexstr):
    """bipolar -1..1"""
    v = h2s(hexstr)
    return v / 2147483648.0 if v < 0 else v / 2147483647.0


def semitones_to_freq(semis_from_c3):
    # C3 = 130.81 Hz as an arbitrary reference "note" for these drum hits
    return 130.81 * 2 ** (semis_from_c3 / 12.0)


def env_time(knob01, tmin=0.0015, tmax=4.0):
    """knob 0..1 -> seconds, exponential taper (Deluge attack/decay/release feel)"""
    return tmin * (tmax / tmin) ** knob01


def adsr(n, attack, decay, sustain, release, hold=0.0):
    """simple exponential-ish ADSR envelope, `n` samples total, note-off after hold+attack+decay"""
    a_n = max(1, int(attack * SR))
    d_n = max(1, int(decay * SR))
    r_n = max(1, int(release * SR))
    env = np.zeros(n)
    i = 0
    # attack: 0 -> 1
    seg = min(a_n, n - i)
    if seg > 0:
        env[i:i + seg] = 1 - np.exp(-5 * np.linspace(0, 1, seg))
        env[i:i + seg] /= (1 - np.exp(-5)) if seg else 1
        i += seg
    # decay: 1 -> sustain
    seg = min(d_n, n - i)
    if seg > 0:
        t = np.linspace(0, 1, seg)
        env[i:i + seg] = sustain + (1 - sustain) * np.exp(-5 * t)
        i += seg
    # sustain hold (drums: essentially none/short, we just fall into release)
    level_at_release_start = sustain
    # release: sustain -> 0
    seg = min(r_n, n - i)
    if seg > 0:
        t = np.linspace(0, 1, seg)
        env[i:i + seg] = level_at_release_start * np.exp(-5 * t)
        i += seg
    if i < n:
        env[i:] = 0
    return env


def moog_ladder(x, cutoff_hz, resonance01, drive=1.0):
    """4-pole transistor-ladder-style LPF (simple discrete approximation)."""
    cutoff_hz = np.clip(cutoff_hz, 20, SR * 0.45)
    g = np.tan(np.pi * cutoff_hz / SR)  # per-sample cutoff can vary (array) or scalar
    g = g / (1 + g)
    k = resonance01 * 4.0  # resonance feedback amount, ~0..4 self-osc region near 4
    y = np.zeros_like(x)
    s = [0.0, 0.0, 0.0, 0.0]
    for n in range(len(x)):
        gg = g[n] if np.ndim(g) else g
        u = x[n] - k * s[3]
        u = np.tanh(u * drive) / max(drive, 1e-6)
        for stage in range(4):
            s_prev = s[stage]
            s[stage] = s_prev + gg * (u - s_prev)
            u = s[stage]
        y[n] = s[3]
    return y


def one_pole_hpf(x, cutoff_hz):
    cutoff_hz = np.clip(cutoff_hz, 5, SR * 0.45)
    a = np.exp(-2 * np.pi * cutoff_hz / SR)
    y = np.zeros_like(x)
    prev_x = 0.0
    prev_y = 0.0
    for n in range(len(x)):
        y[n] = a * (prev_y + x[n] - prev_x)
        prev_x = x[n]
        prev_y = y[n]
    return y


def noise(n, seed=0):
    rng = np.random.default_rng(seed)
    return rng.uniform(-1, 1, n)


def sine(freq_hz, n, phase0=0.0):
    t = np.arange(n) / SR
    return np.sin(2 * np.pi * freq_hz * t + phase0)


def square(freq_hz, n):
    t = np.arange(n) / SR
    return np.sign(np.sin(2 * np.pi * freq_hz * t + 1e-9))


def saw(freq_hz, n):
    t = np.arange(n) / SR
    ph = (freq_hz * t) % 1.0
    return 2 * ph - 1


def normalize(x, peak=0.9):
    m = np.max(np.abs(x)) + 1e-12
    return (x / m) * peak


def write(name, x):
    x = normalize(x)
    wavfile.write(name, SR, (x * 32767).astype(np.int16))
    print("wrote", name)


# ---------------------------------------------------------------------------
# U2 -- KICK (ringmod: two synced sines, envelope1 -> pitch drop)
# ---------------------------------------------------------------------------
def render_kick():
    dur = 1.0
    n = int(dur * SR)
    base_freq = semitones_to_freq(-24)   # osc1
    freq2 = semitones_to_freq(-10)       # osc2 (sync master in the patch, but for a
                                          # ring-mod kick topology the pitch-drop on
                                          # oscA dominates the perceived pitch)
    pitch_env = adsr(n, attack=0.0, decay=env_time(0.10, 0.01, 1.0),
                      sustain=uni("0x8F5C28F3"), release=env_time(0.094, 0.01, 1.0))
    # envelope1 -> oscA pitch, patch amount from XML (~0x228F5C1C -> bipolar)
    pitch_amount = bi("0x228F5C1C")  # depth of pitch envelope
    drop_octaves = 2.5 * pitch_amount  # pitch drops from +drop to base over the envelope
    freq_t = base_freq * 2 ** (drop_octaves * pitch_env)

    phase = np.cumsum(freq_t) / SR
    osc1 = np.sin(2 * np.pi * phase)
    osc2 = sine(freq2, n)
    ring = osc1 * osc2

    amp_env = adsr(n, attack=0.0005, decay=env_time(0.10), sustain=uni("0x8F5C28F3"),
                   release=env_time(0.094))
    sig = ring * amp_env

    sig = moog_ladder(sig, cutoff_hz=60 + 300 * uni("0xBA000000"), resonance01=uni("0xD2000000") * 0.6)
    sig = one_pole_hpf(sig, cutoff_hz=15)
    return sig


# ---------------------------------------------------------------------------
# U3 -- SNARE (subtractive: sine+square tone + noise, shared decay envelope)
# ---------------------------------------------------------------------------
def render_snare():
    dur = 0.6
    n = int(dur * SR)
    tone_freq = semitones_to_freq(2)
    tone = 0.6 * sine(tone_freq, n) + 0.4 * square(semitones_to_freq(0), n)

    body_env = adsr(n, attack=0.0005, decay=env_time(0.04, 0.005, 0.5), sustain=0.0,
                     release=env_time(0.04, 0.005, 0.5))
    noise_env = adsr(n, attack=0.0003, decay=env_time(0.02, 0.005, 0.4), sustain=0.0,
                      release=env_time(0.02, 0.005, 0.4))

    nz = noise(n, seed=1)
    sig = tone * body_env * 0.5 + nz * noise_env * 1.0

    # pitch-drop on the tone via envelope1 (small amount)
    sig = moog_ladder(sig, cutoff_hz=1800 + 6000 * uni("0x4C000000"), resonance01=0.15)
    sig = one_pole_hpf(sig, cutoff_hz=120)
    return sig


# ---------------------------------------------------------------------------
# U4 -- CLOSED HI-HAT (subtractive: mostly noise + squares, short decay, HPF'd)
# ---------------------------------------------------------------------------
def render_closed_hat():
    dur = 0.3
    n = int(dur * SR)
    tone = 0.5 * (square(semitones_to_freq(0), n) + square(semitones_to_freq(7), n))
    nz = noise(n, seed=2)
    env = adsr(n, attack=0.0003, decay=env_time(0.02, 0.004, 0.3), sustain=0.0,
               release=env_time(0.008, 0.004, 0.3))
    sig = (0.25 * tone + 0.9 * nz) * env
    sig = one_pole_hpf(sig, cutoff_hz=4000 + 6000 * uni("0x24000000"))
    return sig


# ---------------------------------------------------------------------------
# U6 -- OPEN HAT / PERC (subtractive + drive, LFO-chattered noise depth)
# ---------------------------------------------------------------------------
def render_open_hat():
    dur = 1.0
    n = int(dur * SR)
    tone = 0.5 * (saw(semitones_to_freq(-15), n) + square(semitones_to_freq(0), n))
    nz = noise(n, seed=3)

    amp_env = adsr(n, attack=env_time(0.32, 0.0005, 0.05), decay=env_time(0.4, 0.02, 0.6),
                   sustain=0.0, release=env_time(0.24, 0.05, 1.0))
    # LFO chattering the noise depth, itself scaled by the amp envelope (nested patch)
    lfo_rate = 8.0
    lfo = square(lfo_rate, n) * 0.5 + 0.5
    chatter_depth = amp_env  # "envelope1 controls lfo1->noiseVolume depth"
    nz_mod = nz * (0.3 + 0.7 * lfo * chatter_depth)

    sig = (0.3 * tone + 0.9 * nz_mod) * amp_env
    sig = np.tanh(sig * 1.5) / 1.5  # clippingAmount drive
    sig = moog_ladder(sig, cutoff_hz=3000 + 9000 * uni("0x4A000000"), resonance01=uni("0xBE000000") * 0.4)
    sig = one_pole_hpf(sig, cutoff_hz=2500)
    return sig


# ---------------------------------------------------------------------------
# U8 -- COWBELL (chained FM: modulator2 -> modulator1 -> carrier)
# ---------------------------------------------------------------------------
def render_cowbell():
    dur = 0.5
    n = int(dur * SR)
    t = np.arange(n) / SR

    carrier_freq = semitones_to_freq(21)
    mod1_freq = semitones_to_freq(13)
    mod2_freq = semitones_to_freq(21)

    mod2_amt = uni("0x3C000000") * 8.0
    mod1_amt = uni("0xFAE14798") * 10.0

    mod2 = np.sin(2 * np.pi * mod2_freq * t)
    mod1_phase = 2 * np.pi * mod1_freq * t + mod2_amt * mod2
    mod1 = np.sin(mod1_phase)
    carrier_phase = 2 * np.pi * carrier_freq * t + mod1_amt * mod1
    carrier = np.sin(carrier_phase)

    env = adsr(n, attack=0.0003, decay=env_time(0.089, 0.01, 0.4), sustain=0.0,
               release=env_time(0.10, 0.02, 0.5))
    sig = carrier * env
    sig = np.tanh(sig * 1.3) / 1.3
    return sig


# ---------------------------------------------------------------------------
# U9 -- CYMBAL/CRASH (detuned saws + noise, wide open filter, long decay)
# ---------------------------------------------------------------------------
def render_cymbal():
    dur = 2.0
    n = int(dur * SR)
    f1 = semitones_to_freq(28)
    f2 = semitones_to_freq(23) * 1.01  # detune
    tone = 0.5 * (saw(f1, n) + saw(f2, n))
    nz = noise(n, seed=4)
    sig = 0.3 * tone + 0.94 * nz

    env = adsr(n, attack=0.0005, decay=env_time(0.4, 0.05, 1.5), sustain=0.0,
               release=env_time(0.4, 0.05, 1.5))
    sig = sig * env
    sig = one_pole_hpf(sig, cutoff_hz=3000 + 6000 * uni("0x32000000"))
    return sig


if __name__ == "__main__":
    import os
    outdir = os.path.dirname(os.path.abspath(__file__))
    write(os.path.join(outdir, "01_kick.wav"), render_kick())
    write(os.path.join(outdir, "02_snare.wav"), render_snare())
    write(os.path.join(outdir, "03_closed_hat.wav"), render_closed_hat())
    write(os.path.join(outdir, "04_open_hat.wav"), render_open_hat())
    write(os.path.join(outdir, "05_cowbell.wav"), render_cowbell())
    write(os.path.join(outdir, "06_cymbal.wav"), render_cymbal())
