import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.io import wavfile
from scipy.signal import spectrogram

MIC_KIT_DIR = "/Users/chrisnicholls/Desktop/deluge2/SAMPLES/Artists/Samuel Verburg/Deluge Mic Kit"
OUTDIR = "/private/tmp/claude-501/-Users-chrisnicholls-eurorack-DLD/e1134b67-12f5-4375-8935-b152b9fa5398/scratchpad"

pairs = [
    ("Kick", f"{MIC_KIT_DIR}/DelugeMicKit_Kick.wav", f"{OUTDIR}/py_wt_capture_kick.wav"),
    ("Snare", f"{MIC_KIT_DIR}/DelugeMicKit_Snare.wav", f"{OUTDIR}/py_wt_capture_snare.wav"),
    ("HiHat", f"{MIC_KIT_DIR}/DelugeMicKit_HiHat.wav", f"{OUTDIR}/py_wt_capture_hihat.wav"),
    ("Crash", f"{MIC_KIT_DIR}/DelugeMicKit_Crash.wav", f"{OUTDIR}/py_wt_capture_crash.wav"),
]


def load_mono(path):
    sr, x = wavfile.read(path)
    if x.ndim > 1:
        x = x.mean(axis=1)
    return sr, x.astype(np.float64)


def plot_spec(ax, sr, x, title, fmax=12000):
    nperseg = 512
    f, t, Sxx = spectrogram(x, fs=sr, nperseg=nperseg, noverlap=nperseg - 64, scaling="spectrum")
    Sxx_db = 10 * np.log10(Sxx + 1e-12)
    vmax = Sxx_db.max()
    im = ax.pcolormesh(t * 1000, f, Sxx_db, shading="gouraud", vmin=vmax - 80, vmax=vmax, cmap="magma")
    ax.set_ylim(0, fmax)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("ms")
    return im


fig, axes = plt.subplots(4, 2, figsize=(11, 14))
for row, (name, target_path, capture_path) in enumerate(pairs):
    sr_t, x_t = load_mono(target_path)
    sr_c, x_c = load_mono(capture_path)
    fmax = 4000 if name in ("Kick",) else (8000 if name == "Snare" else 16000)
    im1 = plot_spec(axes[row, 0], sr_t, x_t, f"{name} -- ACOUSTIC TARGET", fmax=fmax)
    im2 = plot_spec(axes[row, 1], sr_c, x_c, f"{name} -- WAVETABLE CAPTURE", fmax=fmax)
    axes[row, 0].set_ylabel("Hz")

fig.tight_layout()
outpath = f"{OUTDIR}/spectrogram_comparison.png"
fig.savefig(outpath, dpi=130)
print("wrote", outpath)
