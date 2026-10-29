"""
Spot-check the C port's default-setting renders against the python
reference renders in drum_prototype/ (01_kick.wav ... 05_cowbell.wav).

Both are peak-normalised and compared on onset spectrum only: the
python reference runs at 44.1kHz with ~7ms envelopes, while the port
runs at 48kHz with musically rescaled decay knobs, so only the spectral
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
    ("01_kick.wav", "fw_kick_default.wav"),
    ("02_snare.wav", "fw_snare_default.wav"),
    ("03_closed_hat.wav", "fw_chat_default.wav"),
    ("04_open_hat.wav", "fw_ohat_default.wav"),
    ("05_cowbell.wav", "fw_cowbell_default.wav"),
]

ONSET_S = 0.030


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
    # energy fraction below 200Hz / above 5kHz, a coarse spectral shape check
    low = mag[freq < 200].sum() / max(mag.sum(), 1e-12)
    high = mag[freq > 5000].sum() / max(mag.sum(), 1e-12)
    return peak, centroid, low, high


def main():
    print(f"{'voice':<12} {'peakHz py/c':>20} {'centroidHz py/c':>22} "
          f"{'<200Hz py/c':>16} {'>5kHz py/c':>16}")
    bad = 0
    for ref, got in PAIRS:
        rp = os.path.join(REF_DIR, ref)
        gp = os.path.join(OUT_DIR, got)
        if not (os.path.exists(rp) and os.path.exists(gp)):
            print(f"missing: {rp if not os.path.exists(rp) else gp}")
            bad += 1
            continue
        pk_r, cn_r, lo_r, hi_r = stats(rp)
        pk_g, cn_g, lo_g, hi_g = stats(gp)
        name = ref.split("_", 1)[1].replace(".wav", "")
        print(f"{name:<12} {pk_r:9.0f} /{pk_g:9.0f} {cn_r:11.0f} /{cn_g:10.0f} "
              f"{lo_r:7.2f} /{lo_g:7.2f} {hi_r:7.2f} /{hi_g:7.2f}")
    return bad


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
