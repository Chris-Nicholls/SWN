"""
Spot-check the chiptune C port's default-setting renders against the python
reference renders in drum_prototype/ (py_chip_kick.wav ... py_chip_cowbell.wav).

Peak frequency is a poor metric for this family -- LFSR noise is broadband
and harsh by design -- so the checks here are the two things that actually
define a 2A03 voice:

  * noise/tone clock rate, measured as onset zero-crossing rate and spectral
    centroid (this is what the fixed noise-period table selects), and
  * the 16-step linear staircase envelope: its step interval and the fact
    that the decay is linear rather than exponential.

The python reference runs at 44.1kHz and normalises every render; the port
runs at 48kHz through the shared SVF, so tolerances are deliberately loose.
"""
import os
import sys

import numpy as np
from scipy.io import wavfile

HERE = os.path.dirname(os.path.abspath(__file__))
REF_DIR = os.path.normpath(os.path.join(HERE, ".."))
OUT_DIR = os.path.join(HERE, "out")

PAIRS = [
    ("kick", "py_chip_kick.wav", "fw_chip_kick_default.wav"),
    ("snare", "py_chip_snare.wav", "fw_chip_snare_default.wav"),
    ("chat", "py_chip_chat.wav", "fw_chip_chat_default.wav"),
    ("ohat", "py_chip_ohat.wav", "fw_chip_ohat_default.wav"),
    ("perc", "py_chip_perc.wav", "fw_chip_perc_default.wav"),
    ("cowbell", "py_chip_cowbell.wav", "fw_chip_cowbell_default.wav"),
]

ONSET_S = 0.020

CENTROID_TOL_OCT = 1.0
ZCR_TOL_OCT = 1.0
# The staircase clocks at 240/(period+1) Hz; allow a factor of ~1.6 either
# way to absorb the shared filter smearing the step edges.
STEP_TOL_OCT = 0.7


def load(path):
    sr, x = wavfile.read(path)
    x = x.astype(float) / 32768.0
    if x.ndim > 1:
        x = x.mean(axis=1)
    return sr, x


def onset_stats(sr, x):
    n = int(ONSET_S * sr)
    seg = x[:n] * np.hanning(n)
    mag = np.abs(np.fft.rfft(seg, 1 << 15))
    freq = np.fft.rfftfreq(1 << 15, 1.0 / sr)
    centroid = float((mag * freq).sum() / max(mag.sum(), 1e-12))
    zcr = float(np.mean(np.abs(np.diff(np.sign(x[:n]))) > 0)) * sr / 2.0
    return centroid, zcr


def envelope(sr, x, hop_s=0.001):
    hop = max(1, int(hop_s * sr))
    n = len(x) // hop
    env = np.sqrt(np.array([np.mean(x[i * hop:(i + 1) * hop] ** 2) for i in range(n)]))
    # Same 3-point smooth the C harness applies, so the step counts are
    # measured on comparable envelopes.
    if len(env) >= 3:
        env = np.convolve(env, np.ones(3) / 3.0, mode="same")
    if env.max() <= 0:
        return env, hop_s
    keep = np.nonzero(env > 0.02 * env.max())[0]
    return env[: keep[-1] + 1], hop_s


def staircase(sr, x):
    """Mean interval (s) between downward amplitude steps, and whether the
    decay fits a straight line better than an exponential."""
    env, hop_s = envelope(sr, x)
    if len(env) < 4:
        return float("nan"), "n/a"

    drops = []
    falling = False
    for i in range(1, len(env)):
        if env[i] < max(env[i - 1], 1e-9) * 0.90:
            if not falling:
                drops.append(i)
            falling = True
        elif env[i] > max(env[i - 1], 1e-9) * 0.98:
            falling = False
    step_s = float(np.mean(np.diff(drops)) * hop_s) if len(drops) >= 2 else float("nan")

    pk = int(np.argmax(env))
    tail = env[pk:]
    if len(tail) < 6:
        return step_s, "n/a"
    t = np.arange(len(tail))
    r2_lin = r2(t, tail)
    r2_exp = r2(t, np.log(np.maximum(tail, 1e-9)))
    return step_s, ("lin" if r2_lin >= r2_exp else "exp")


def r2(t, v):
    a, b = np.polyfit(t, v, 1)
    res = v - (a * t + b)
    tot = v - v.mean()
    denom = float((tot ** 2).sum())
    return 1.0 - float((res ** 2).sum()) / denom if denom > 0 else 0.0


def oct_diff(a, b):
    if not np.isfinite(a) or not np.isfinite(b):
        return float("inf")
    return abs(np.log2(max(a, 1e-9) / max(b, 1e-9)))


def main():
    print(f"{'voice':<8} {'centroidHz py/c':>21} {'zcrHz py/c':>19} "
          f"{'step_ms py/c':>17} {'shape py/c':>11}  verdict")
    bad = 0
    for name, ref, got in PAIRS:
        rp, gp = os.path.join(REF_DIR, ref), os.path.join(OUT_DIR, got)
        if not (os.path.exists(rp) and os.path.exists(gp)):
            print(f"{name:<8} missing {rp if not os.path.exists(rp) else gp}")
            bad += 1
            continue
        sr_r, xr = load(rp)
        sr_g, xg = load(gp)
        cn_r, zc_r = onset_stats(sr_r, xr)
        cn_g, zc_g = onset_stats(sr_g, xg)
        st_r, sh_r = staircase(sr_r, xr)
        st_g, sh_g = staircase(sr_g, xg)

        spectral_ok = (oct_diff(cn_r, cn_g) <= CENTROID_TOL_OCT
                       and oct_diff(zc_r, zc_g) <= ZCR_TOL_OCT)
        # A missing step interval on either side is not a failure: some
        # voices are gated before enough staircase steps are exposed.
        step_ok = (not np.isfinite(st_r) or not np.isfinite(st_g)
                   or oct_diff(st_r, st_g) <= STEP_TOL_OCT)
        shape_ok = (sh_r != "lin") or (sh_g == "lin")
        ok = spectral_ok and step_ok and shape_ok
        bad += not ok

        print(f"{name:<8} {cn_r:10.0f} /{cn_g:10.0f} {zc_r:9.0f} /{zc_g:9.0f} "
              f"{st_r * 1e3:8.1f} /{st_g * 1e3:7.1f} {sh_r:>5} /{sh_g:<5}  "
              f"{'OK' if ok else 'CHECK!'}")
    return bad


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
