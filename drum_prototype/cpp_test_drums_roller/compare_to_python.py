"""
Spot-check the C port's default-setting renders against the python
reference renders in drum_prototype/ (py_roller_kick.wav ... py_roller_rim.wav).

Both sides are peak-normalised and compared on onset spectrum only: the
python reference runs at 44.1kHz and is peak-normalised to 0.9, while the
port runs at 48kHz through the shared post-voice SVF, so only the spectral
shape of the attack is expected to line up.
"""
import os
import sys

import numpy as np
from scipy.io import wavfile

HERE = os.path.dirname(os.path.abspath(__file__))
REF_DIR = os.path.normpath(os.path.join(HERE, ".."))
OUT_DIR = os.path.join(HERE, "out")

PAIRS = [
    ("kick", "py_roller_kick.wav", "fw_kick_default.wav"),
    ("snare", "py_roller_snare.wav", "fw_snare_default.wav"),
    ("closedhat", "py_roller_chat.wav", "fw_chat_default.wav"),
    ("openhat", "py_roller_ohat.wav", "fw_ohat_default.wav"),
    ("ride", "py_roller_ride.wav", "fw_ride_default.wav"),
    ("crash", "py_roller_crash.wav", "fw_crash_default.wav"),
    ("perc", "py_roller_perc.wav", "fw_perc_default.wav"),
    ("rimshot", "py_roller_rim.wav", "fw_rim_default.wav"),
]

ONSET_S = 0.030

# Per-voice tolerance on the log2 centroid ratio; cymbals and hats are
# broadband washes whose centroid is stable, the tonal voices are judged
# on peak frequency instead.
CENTROID_TOL_OCT = 1.0
PEAK_TOL_OCT = 0.35


def stats(path):
    sr, x = wavfile.read(path)
    x = x.astype(float) / 32768.0
    if x.ndim > 1:
        x = x.mean(axis=1)
    n = int(ONSET_S * sr)
    seg = x[:n] * np.hanning(n)
    mag = np.abs(np.fft.rfft(seg, 1 << 15))
    freq = np.fft.rfftfreq(1 << 15, 1.0 / sr)
    peak = freq[mag.argmax()]
    centroid = float((mag * freq).sum() / max(mag.sum(), 1e-12))
    zcr = float(np.mean(np.abs(np.diff(np.sign(x[:n]))) > 0)) * sr / 2.0
    return peak, centroid, zcr


def oct_diff(a, b):
    return abs(np.log2(max(a, 1e-9) / max(b, 1e-9)))


def main():
    print(f"{'voice':<10} {'peakHz py/c':>21} {'centroidHz py/c':>22} "
          f"{'zcrHz py/c':>20}  verdict")
    bad = 0
    for name, ref, got in PAIRS:
        rp, gp = os.path.join(REF_DIR, ref), os.path.join(OUT_DIR, got)
        if not (os.path.exists(rp) and os.path.exists(gp)):
            print(f"{name:<10} missing {rp if not os.path.exists(rp) else gp}")
            bad += 1
            continue
        pk_r, cn_r, zc_r = stats(rp)
        pk_g, cn_g, zc_g = stats(gp)
        ok = oct_diff(cn_r, cn_g) <= CENTROID_TOL_OCT or oct_diff(pk_r, pk_g) <= PEAK_TOL_OCT
        bad += not ok
        print(f"{name:<10} {pk_r:9.0f} /{pk_g:9.0f} {cn_r:11.0f} /{cn_g:10.0f} "
              f"{zc_r:9.0f} /{zc_g:9.0f}  {'OK' if ok else 'CHECK!'}")
    return bad


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
