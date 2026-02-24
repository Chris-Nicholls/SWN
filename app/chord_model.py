from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Literal, Sequence, Tuple

import numpy as np


NOTE_TO_SEMITONE = {
    "C": 0,
    "C#": 1,
    "D": 2,
    "D#": 3,
    "E": 4,
    "F": 5,
    "F#": 6,
    "G": 7,
    "G#": 8,
    "A": 9,
    "A#": 10,
    "B": 11,
}
SEMITONE_TO_NOTE = {v: k for k, v in NOTE_TO_SEMITONE.items()}


def note_to_midi(note: str) -> int:
    # Expected format like 'C#4' or 'B2'
    pitch = note[:-1]
    octave = int(note[-1])
    return 12 * (octave + 1) + NOTE_TO_SEMITONE[pitch]


def midi_to_note(midi: int) -> str:
    pitch = SEMITONE_TO_NOTE[midi % 12]
    octave = midi // 12 - 1
    return f"{pitch}{octave}"


def midi_to_note_microtonal(midi: float, cents_precision: int = 1) -> str:
    """Format a (possibly fractional) MIDI as a note name with cents offset."""

    m = float(midi)
    nearest = int(round(m))
    base = midi_to_note(nearest)
    cents = (m - float(nearest)) * 100.0
    if abs(cents) < 0.5 * (10 ** (-int(cents_precision))):
        return base
    return f"{base} {cents:+.{int(cents_precision)}f}c"


def midi_to_frequency(midi: int, a4_hz: float = 440.0) -> float:
    return float(a4_hz * (2.0 ** ((midi - 69) / 12.0)))


def midi_to_frequency_continuous(midi: np.ndarray, a4_hz: float = 440.0) -> np.ndarray:
    """Frequency for (possibly fractional) MIDI values."""
    m = np.asarray(midi, dtype=np.float64)
    return a4_hz * (2.0 ** ((m - 69.0) / 12.0))


def note_to_frequency(note: str, a4_hz: float = 440.0) -> float:
    return midi_to_frequency(note_to_midi(note), a4_hz=a4_hz)


def frequency_to_midi(frequency_hz: float, a4_hz: float = 440.0) -> int:
    return int(round(69 + 12 * np.log2(float(frequency_hz) / a4_hz)))


def frequency_to_note_name(frequency_hz: float, a4_hz: float = 440.0) -> str:
    return midi_to_note(frequency_to_midi(frequency_hz, a4_hz=a4_hz))


def normalize_overtone_weights(weights: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    """Normalize to a probability simplex to prevent degenerate scaling."""
    w = np.asarray(weights, dtype=np.float64)
    w = np.clip(w, 0.0, None)
    s = float(w.sum())
    if s <= eps:
        return np.ones_like(w) / w.size
    return w / s


def _lowpass_gain(
    freq_hz: np.ndarray,
    cutoff_hz: float,
    slope_db_per_oct: float,
) -> np.ndarray:
    """Amplitude gain for a simple low-pass rolloff above cutoff.

    For f <= cutoff: gain = 1
    For f > cutoff:  gain = 10^(-(slope_db_per_oct * log2(f/cutoff))/20)
    """

    f = np.asarray(freq_hz, dtype=np.float64)
    fc = float(cutoff_hz)
    slope = float(slope_db_per_oct)
    if slope <= 0.0 or fc <= 0.0:
        return np.ones_like(f)

    ratio = np.maximum(f, 1e-12) / max(fc, 1e-12)
    octaves = np.maximum(0.0, np.log2(ratio))
    db = slope * octaves
    return np.power(10.0, -db / 20.0)


def _normalize_weights_last_dim(w: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    w = np.asarray(w, dtype=np.float64)
    w = np.clip(w, 0.0, None)
    s = np.sum(w, axis=-1, keepdims=True)
    return np.where(s > eps, w / s, np.ones_like(w) / float(w.shape[-1]))


def _geometric_lerp(a: np.ndarray, b: np.ndarray, t: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    t = np.asarray(t, dtype=np.float64)
    a = np.maximum(a, 1e-12)
    b = np.maximum(b, 1e-12)
    return np.exp((1.0 - t) * np.log(a) + t * np.log(b))


def _register_t(
    f_hz: np.ndarray,
    anchor_low_hz: float | None = None,
    anchor_high_hz: float | None = None,
) -> np.ndarray:
    """Compute register interpolation coordinate t from frequency.

    t is 0 at C2, 1 at C6, and extrapolates outside that range.
    """

    f = np.asarray(f_hz, dtype=np.float64)
    f = np.maximum(f, 1e-9)

    low_hz = float(anchor_low_hz) if anchor_low_hz is not None else midi_to_frequency(note_to_midi("C2"))
    high_hz = float(anchor_high_hz) if anchor_high_hz is not None else midi_to_frequency(note_to_midi("C6"))
    low_hz = max(low_hz, 1e-6)
    high_hz = max(high_hz, low_hz * 1.0001)

    return (np.log2(f) - np.log2(low_hz)) / (np.log2(high_hz) - np.log2(low_hz))


def _register_geometric(
    f_hz: np.ndarray,
    value_c2: float,
    value_c6: float,
    anchor_low_hz: float | None = None,
    anchor_high_hz: float | None = None,
) -> np.ndarray:
    """Geometric interpolation/extrapolation for positive parameters."""

    v_low = float(value_c2)
    v_high = float(value_c6)
    if v_low <= 0.0 or v_high <= 0.0:
        raise ValueError("Interpolated parameters must be > 0")

    t = _register_t(f_hz, anchor_low_hz=anchor_low_hz, anchor_high_hz=anchor_high_hz)
    return _geometric_lerp(v_low, v_high, t)


def _register_linear(
    f_hz: np.ndarray,
    value_c2: float,
    value_c6: float,
    anchor_low_hz: float | None = None,
    anchor_high_hz: float | None = None,
) -> np.ndarray:
    """Linear interpolation/extrapolation (allows zero)."""

    v_low = float(value_c2)
    v_high = float(value_c6)
    t = _register_t(f_hz, anchor_low_hz=anchor_low_hz, anchor_high_hz=anchor_high_hz)
    return (1.0 - t) * v_low + t * v_high


def _solve_u_for_y(y: np.ndarray, n_iter: int = 10) -> np.ndarray:
    """Solve y = u/(exp(u)-1) for u>0 (vectorized Newton iterations).

    Valid for y in (0, 1). This inversion is used to match a desired peak
    location given an exponential tail rate.
    """

    y = np.asarray(y, dtype=np.float64)
    if np.any(y <= 0.0) or np.any(y >= 1.0):
        raise ValueError("Internal constraint violated: require 0 < a*peak_semitones < 1")

    # Initial guess:
    # - near y=1 => u small, u ≈ 2(1-y)
    # - near y=0 => u large, u ≈ log(1/y) + log(log(1/y))
    inv_y = 1.0 / y
    u_small = 2.0 * (1.0 - y)
    u_large = np.log(inv_y) + np.log(np.maximum(np.log(inv_y), 1e-12))
    u = np.where(y > 0.5, u_small, u_large)
    u = np.maximum(u, 1e-12)

    for _ in range(int(n_iter)):
        eu = np.exp(u)
        denom = eu - 1.0
        f = (u / denom) - y
        fp = (denom - u * eu) / (denom * denom)
        step = f / fp
        u = u - step
        u = np.maximum(u, 1e-12)

    return u


def _ab_from_peak_semitones_and_decay_db_per_oct(
    peak_semitones: np.ndarray,
    decay_db_per_oct: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute (a,b) for R(x)=exp(-a x)-exp(-b x) in semitone-distance domain.

    x is a semitone distance (>=0). Tail decay is specified as dB per octave of
    interval (i.e. +12 semitones). For large x, R(x)≈exp(-a x).

    We choose a from the desired tail decay, then solve for b so the peak occurs
    at the desired `peak_semitones`.
    """

    x_peak = np.asarray(peak_semitones, dtype=np.float64)
    if np.any(x_peak <= 0.0):
        raise ValueError("peak_semitones must be > 0")

    decay = np.asarray(decay_db_per_oct, dtype=np.float64)
    if np.any(decay <= 0.0):
        raise ValueError("decay_db_per_oct must be > 0")

    # Convert dB/oct (amplitude) to exponential rate per semitone.
    # If decay is 20 dB per octave, then exp(-a*12) = 10^(-20/20)=0.1.
    a = (decay * np.log(10.0)) / (12.0 * 20.0)

    y = a * x_peak
    if np.any(y >= 1.0):
        raise ValueError("Invalid parameters: decay too steep for the chosen peak position (need a*peak_semitones < 1)")

    u = _solve_u_for_y(y)
    k = np.exp(u)  # k = b/a
    b = k * a
    return a, b


def _boundary_penalty_midis(
    candidate_midis: np.ndarray,
    root_midi: int,
    extension_midi: int,
    below_root_db_per_oct: float = 0.0,
    above_extension_db_per_oct: float = 0.0,
) -> np.ndarray:
    """Additive penalty for notes below root / above extension.

        Penalty grows *linearly* with distance beyond the boundary (measured in octaves).
        The dB/oct sliders define the per-octave slope by converting dB to an amplitude
        ratio and subtracting 1:

            slope_per_oct = 10^(db_per_oct/20) - 1
            penalty = slope_per_oct * octaves_outside

        This keeps the UI in familiar dB terms while making the ramp linear.
    """

    m = np.asarray(candidate_midis, dtype=np.float64)
    r = float(root_midi)
    e = float(extension_midi)

    low = min(r, e)
    high = max(r, e)

    out = np.zeros_like(m)

    below = float(below_root_db_per_oct)
    if below > 0.0:
        oct_below = np.maximum(0.0, (low - m) / 12.0)
        slope = np.power(10.0, below / 20.0) - 1.0
        out = out + slope * oct_below

    above = float(above_extension_db_per_oct)
    if above > 0.0:
        oct_above = np.maximum(0.0, (m - high) / 12.0)
        slope = np.power(10.0, above / 20.0) - 1.0
        out = out + slope * oct_above

    return out


def sine_dissonance(
    f1_hz: np.ndarray,
    f2_hz: np.ndarray,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    fall_to_zero_semitones_c2: float = 12.0,
    fall_to_zero_semitones_c6: float = 12.0,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    sine_kernel: Literal["linear", "exponential"] = "linear",
) -> np.ndarray:
    """Sensory dissonance (roughness) between two pure sine waves (deterministic).

    The kernel is piecewise-linear in semitone distance:
    - rises linearly from 0 at unison to `height` at `peak_semitones`
    - falls linearly back to 0

        Kernel modes:
        - `linear`: piecewise-linear (triangle). Tail is set by
            `fall_to_zero_semitones_*` (semitones from peak to zero).
        - `exponential`: kernel `exp(-a x) - exp(-b x)` with parameters chosen
            so the peak occurs at `peak_semitones` and the tail decays at
            `decay_db_per_oct_*` dB/oct.
    """
    f1 = np.asarray(f1_hz, dtype=np.float64)
    f2 = np.asarray(f2_hz, dtype=np.float64)

    # Semitone distance between frequencies (symmetric, 0 at unison).
    ratio = np.maximum(f2, 1e-12) / np.maximum(f1, 1e-12)
    x = np.abs(12.0 * np.log2(ratio))

    # Interpolate/extrapolate parameters by register using the lower frequency.
    f_min = np.maximum(np.minimum(f1, f2), 1e-9)
    x_peak = _register_geometric(f_min, peak_semitones_c2, peak_semitones_c6)
    decay_db = _register_geometric(f_min, decay_db_per_oct_c2, decay_db_per_oct_c6)
    fall = _register_geometric(f_min, float(fall_to_zero_semitones_c2), float(fall_to_zero_semitones_c6))
    height = np.maximum(_register_linear(f_min, height_c2, height_c6), 0.0)

    mode = str(sine_kernel).strip().lower()
    if mode not in ("linear", "exponential"):
        raise ValueError("sine_kernel must be 'linear' or 'exponential'")

    if mode == "exponential":
        a, b = _ab_from_peak_semitones_and_decay_db_per_oct(x_peak, decay_db)

        base = np.exp(-a * x) - np.exp(-b * x)

        # Scale so the peak height matches `height`.
        peak_val = np.exp(-a * x_peak) - np.exp(-b * x_peak)
        scale = np.where(peak_val > 1e-12, height / peak_val, 0.0)
        D = scale * base
        return np.maximum(D, 0.0)

    # Piecewise-linear triangular kernel in semitone-distance domain:
    # - Linear rise from 0 at x=0 to height at x=x_peak
    # - Linear fall to 0 at x=x_peak + fall_semitones
    fall = np.maximum(fall, 1e-12)
    x_zero = x_peak + fall

    up = np.where(x_peak > 1e-12, height * (x / x_peak), 0.0)
    down = height * (1.0 - (x - x_peak) / fall)

    D = np.where(x <= x_peak, up, down)
    return np.clip(D, 0.0, None)


def overtone_dissonance(
    f1_hz: np.ndarray,
    f2_hz: np.ndarray,
    overtone_weights: np.ndarray,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    fall_to_zero_semitones_c2: float = 12.0,
    fall_to_zero_semitones_c6: float = 12.0,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    sine_kernel: Literal["linear", "exponential"] = "linear",
    lowpass_cutoff_hz: float = 20000.0,
    lowpass_slope_db_per_oct: float = 0.0,
    lowpass_renormalize: bool = True,
) -> np.ndarray:
    """Total dissonance between two complex tones with given overtone weights.

    Accepts broadcasting arrays for f1_hz/f2_hz.
    """
    w = np.asarray(overtone_weights, dtype=np.float64)
    w = np.clip(w, 0.0, None)
    if w.size == 0:
        return np.asarray(0.0)
    if float(np.sum(w)) <= 1e-12:
        # Avoid all-zero weights degeneracy.
        w = w.copy()
        w[0] = 1.0
    n = np.arange(1, w.size + 1, dtype=np.float64)

    f1_base = np.asarray(f1_hz, dtype=np.float64)
    f2_base = np.asarray(f2_hz, dtype=np.float64)

    # Overtone frequencies per tone (used for low-pass weighting).
    f1_over = f1_base[..., None] * n[None, :]
    f2_over = f2_base[..., None] * n[None, :]

    g1 = _lowpass_gain(f1_over, cutoff_hz=lowpass_cutoff_hz, slope_db_per_oct=lowpass_slope_db_per_oct)
    g2 = _lowpass_gain(f2_over, cutoff_hz=lowpass_cutoff_hz, slope_db_per_oct=lowpass_slope_db_per_oct)

    # Apply low-pass. Optionally renormalize per tone so overall loudness stays comparable.
    w1_raw = g1 * w[None, :]
    w2_raw = g2 * w[None, :]
    if bool(lowpass_renormalize):
        w1 = _normalize_weights_last_dim(w1_raw)
        w2 = _normalize_weights_last_dim(w2_raw)
    else:
        w1 = w1_raw
        w2 = w2_raw

    f1 = f1_base[..., None, None] * n[None, :, None]
    f2 = f2_base[..., None, None] * n[None, None, :]

    w_outer = (w1[..., :, None] * w2[..., None, :])

    base = sine_dissonance(
        f1,
        f2,
        peak_semitones_c2=peak_semitones_c2,
        peak_semitones_c6=peak_semitones_c6,
        fall_to_zero_semitones_c2=fall_to_zero_semitones_c2,
        fall_to_zero_semitones_c6=fall_to_zero_semitones_c6,
        decay_db_per_oct_c2=decay_db_per_oct_c2,
        decay_db_per_oct_c6=decay_db_per_oct_c6,
        height_c2=height_c2,
        height_c6=height_c6,
        sine_kernel=sine_kernel,
    )


    # Sum over harmonic pairs; squeeze so scalar inputs return a scalar.
    return np.sum(w_outer * base, axis=(-2, -1)).squeeze()


def dissonance_to_set(
    candidate_freqs_hz: np.ndarray,
    existing_freqs_hz: Sequence[float],
    overtone_weights: np.ndarray,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    fall_to_zero_semitones_c2: float = 12.0,
    fall_to_zero_semitones_c6: float = 12.0,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    lowpass_cutoff_hz: float = 20000.0,
    lowpass_slope_db_per_oct: float = 0.0,
    lowpass_renormalize: bool = True,
    sine_kernel: Literal["linear", "exponential"] = "linear",
    set_aggregation: Literal["sum", "max", "rms"] = "sum",
    existing_weights: Sequence[float] | None = None,
    candidate_weight: float | np.ndarray = 1.0,
) -> np.ndarray:
    """Dissonance of candidate(s) against a fixed set."""

    candidate = np.asarray(candidate_freqs_hz, dtype=np.float64)
    mode = str(set_aggregation).strip().lower()
    if mode not in ("sum", "max", "rms"):
        raise ValueError("set_aggregation must be 'sum', 'max', or 'rms'")

    existing = [float(f) for f in existing_freqs_hz]
    if existing_weights is None:
        weights_existing = [1.0] * len(existing)
    else:
        weights_existing = [float(w) for w in existing_weights]
        if len(weights_existing) != len(existing):
            raise ValueError("existing_weights must have the same length as existing_freqs_hz")

    w_cand = np.asarray(candidate_weight, dtype=np.float64)

    def _total_at(freqs: np.ndarray) -> np.ndarray:
        if not existing:
            return np.zeros_like(freqs)

        if mode == "rms":
            sum_sq = np.zeros_like(freqs)
            count = 0
            for f_exist, w_exist in zip(existing, weights_existing):
                d = overtone_dissonance(
                    freqs,
                    f_exist,
                    overtone_weights,
                    peak_semitones_c2=peak_semitones_c2,
                    peak_semitones_c6=peak_semitones_c6,
                    fall_to_zero_semitones_c2=fall_to_zero_semitones_c2,
                    fall_to_zero_semitones_c6=fall_to_zero_semitones_c6,
                    decay_db_per_oct_c2=decay_db_per_oct_c2,
                    decay_db_per_oct_c6=decay_db_per_oct_c6,
                    height_c2=height_c2,
                    height_c6=height_c6,
                    sine_kernel=sine_kernel,
                    lowpass_cutoff_hz=lowpass_cutoff_hz,
                    lowpass_slope_db_per_oct=lowpass_slope_db_per_oct,
                    lowpass_renormalize=lowpass_renormalize,
                )
                v = float(w_exist) * d
                sum_sq = sum_sq + (v * v)
                count += 1

            denom = float(count) if count > 0 else 1.0
            total_here = np.sqrt(sum_sq / denom)
            return total_here * w_cand

        total_here = np.zeros_like(freqs)
        for f_exist, w_exist in zip(existing, weights_existing):
            d = overtone_dissonance(
                freqs,
                f_exist,
                overtone_weights,
                peak_semitones_c2=peak_semitones_c2,
                peak_semitones_c6=peak_semitones_c6,
                fall_to_zero_semitones_c2=fall_to_zero_semitones_c2,
                fall_to_zero_semitones_c6=fall_to_zero_semitones_c6,
                decay_db_per_oct_c2=decay_db_per_oct_c2,
                decay_db_per_oct_c6=decay_db_per_oct_c6,
                height_c2=height_c2,
                height_c6=height_c6,
                sine_kernel=sine_kernel,
                lowpass_cutoff_hz=lowpass_cutoff_hz,
                lowpass_slope_db_per_oct=lowpass_slope_db_per_oct,
                lowpass_renormalize=lowpass_renormalize,
            )
            if mode == "sum":
                total_here = total_here + float(w_exist) * d
            else:
                total_here = np.maximum(total_here, float(w_exist) * d)

        return total_here * w_cand

    return _total_at(candidate)
    


def average_note_dissonance(
    chord_freqs_hz: Sequence[float],
    overtone_weights: np.ndarray,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    fall_to_zero_semitones_c2: float = 12.0,
    fall_to_zero_semitones_c6: float = 12.0,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    lowpass_cutoff_hz: float = 20000.0,
    lowpass_slope_db_per_oct: float = 0.0,
    lowpass_renormalize: bool = True,
    sine_kernel: Literal["linear", "exponential"] = "linear",
    set_aggregation: Literal["sum", "max", "rms"] = "sum",
    chord_aggregation: Literal["sum", "mean", "rms"] = "sum",
    extension_freq_hz: float | None = None,
    extension_weight: float = 1.0,
) -> float:
    """Chord dissonance proxy aggregated across notes.

    For each note i, compute dissonance of i against the rest of the chord
    (using `dissonance_to_set` with the configured `set_aggregation`).

    Returns an aggregate across chord notes:
    - `sum`: sum of per-note dissonances (scales with chord size)
    - `mean`: mean of per-note dissonances
    - `rms`: root-mean-square of per-note dissonances (penalizes large values)

    Note:
    - With `set_aggregation='sum'`, per-note dissonance double-counts unordered
      pairs (i,j) and (j,i). That's usually fine for comparisons.
    """
    freqs = [float(f) for f in chord_freqs_hz]
    n = len(freqs)
    if n < 2:
        return 0.0

    ext = float(extension_freq_hz) if extension_freq_hz is not None else None
    ext_w = float(extension_weight)

    def _is_ext(f: float) -> bool:
        if ext is None:
            return False
        return bool(np.isclose(float(f), ext, rtol=1e-9, atol=1e-6))

    per_note = []
    for i in range(n):
        others = [freqs[j] for j in range(n) if j != i]

        cand_w = ext_w if _is_ext(freqs[i]) else 1.0
        other_ws = None
        if ext is not None:
            other_ws = [ext_w if _is_ext(f) else 1.0 for f in others]

        d = float(
            dissonance_to_set(
                np.array([freqs[i]], dtype=np.float64),
                others,
                overtone_weights,
                peak_semitones_c2=peak_semitones_c2,
                peak_semitones_c6=peak_semitones_c6,
                fall_to_zero_semitones_c2=fall_to_zero_semitones_c2,
                fall_to_zero_semitones_c6=fall_to_zero_semitones_c6,
                decay_db_per_oct_c2=decay_db_per_oct_c2,
                decay_db_per_oct_c6=decay_db_per_oct_c6,
                height_c2=height_c2,
                height_c6=height_c6,
                lowpass_cutoff_hz=lowpass_cutoff_hz,
                lowpass_slope_db_per_oct=lowpass_slope_db_per_oct,
                lowpass_renormalize=lowpass_renormalize,
                sine_kernel=sine_kernel,
                set_aggregation=set_aggregation,
                existing_weights=other_ws,
                candidate_weight=cand_w,
            )[0]
        )
        per_note.append(d)
    per_note_arr = np.asarray(per_note, dtype=np.float64)

    agg = str(chord_aggregation).strip().lower()
    if agg == "sum":
        return float(np.sum(per_note_arr))
    if agg == "mean":
        return float(np.mean(per_note_arr))
    if agg == "rms":
        return float(np.sqrt(np.mean(per_note_arr * per_note_arr)))
    raise ValueError("chord_aggregation must be 'sum', 'mean', or 'rms'")


def chord_partial_dissonance_matrix(
    chord_freqs_hz: Sequence[float],
    overtone_weights: np.ndarray,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    fall_to_zero_semitones_c2: float = 12.0,
    fall_to_zero_semitones_c6: float = 12.0,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    lowpass_cutoff_hz: float = 20000.0,
    lowpass_slope_db_per_oct: float = 0.0,
    lowpass_renormalize: bool = True,
    sine_kernel: Literal["linear", "exponential"] = "linear",
    set_aggregation: Literal["sum", "max", "rms"] = "sum",
    extension_freq_hz: float | None = None,
    extension_weight: float = 1.0,
) -> np.ndarray:
    """Per-note/per-overtone dissonance heatmap for a chord.

    Returns an array shaped (n_overtones, n_notes). Each entry corresponds to the
    weighted roughness contributed by *that* partial (note i, harmonic k) against
    the rest of the chord.

    Weighting includes:
    - overtone weights (including optional low-pass + optional per-note renormalization)
    - extension weighting (pairs involving the extension note are scaled)

    Aggregation uses the same semantics as the chord objective:
    - set_aggregation='sum': sums contributions against all other chord notes
    - set_aggregation='max': takes the worst (maximum) contribution against any other note
      (still summed/maxed across the other note's harmonics).
    """

    freqs = np.asarray([float(f) for f in chord_freqs_hz], dtype=np.float64)
    n_notes = int(freqs.size)
    if n_notes == 0:
        return np.zeros((int(np.asarray(overtone_weights).size), 0), dtype=np.float64)

    w_base = np.asarray(overtone_weights, dtype=np.float64)
    w_base = np.clip(w_base, 0.0, None)
    if float(np.sum(w_base)) <= 1e-12 and w_base.size > 0:
        w_base = w_base.copy()
        w_base[0] = 1.0
    n_over = int(w_base.size)
    if n_over <= 0:
        return np.zeros((0, n_notes), dtype=np.float64)

    harmonics = np.arange(1, n_over + 1, dtype=np.float64)
    partial_freqs = freqs[:, None] * harmonics[None, :]

    # Per-note harmonic weights (after low-pass), optionally renormalized per note.
    g = _lowpass_gain(partial_freqs, cutoff_hz=lowpass_cutoff_hz, slope_db_per_oct=lowpass_slope_db_per_oct)
    w_note_raw = g * w_base[None, :]
    if bool(lowpass_renormalize):
        w_note = _normalize_weights_last_dim(w_note_raw)
    else:
        w_note = w_note_raw

    ext = float(extension_freq_hz) if extension_freq_hz is not None else None
    ext_w = float(extension_weight)

    def _is_ext(f0: float) -> bool:
        if ext is None:
            return False
        return bool(np.isclose(float(f0), ext, rtol=1e-9, atol=1e-6))

    note_weights = np.array([ext_w if _is_ext(float(f)) else 1.0 for f in freqs], dtype=np.float64)

    mode = str(set_aggregation).strip().lower()
    if mode not in ("sum", "max", "rms"):
        raise ValueError("set_aggregation must be 'sum', 'max', or 'rms'")

    out = np.zeros((n_notes, n_over), dtype=np.float64)
    for i in range(n_notes):
        if mode == "rms":
            acc_sum_sq = np.zeros((n_over,), dtype=np.float64)
            acc_count = 0
        else:
            acc = np.zeros((n_over,), dtype=np.float64)
        for j in range(n_notes):
            if j == i:
                continue

            # Kernel for all harmonic pairs between note i and j.
            f_i = partial_freqs[i][:, None]  # (H,1)
            f_j = partial_freqs[j][None, :]  # (1,H)
            base = sine_dissonance(
                f_i,
                f_j,
                peak_semitones_c2=peak_semitones_c2,
                peak_semitones_c6=peak_semitones_c6,
                fall_to_zero_semitones_c2=fall_to_zero_semitones_c2,
                fall_to_zero_semitones_c6=fall_to_zero_semitones_c6,
                decay_db_per_oct_c2=decay_db_per_oct_c2,
                decay_db_per_oct_c6=decay_db_per_oct_c6,
                height_c2=height_c2,
                height_c6=height_c6,
                sine_kernel=sine_kernel,
            )

            w_outer = (w_note[i][:, None] * w_note[j][None, :])
            pair = (w_outer * base)  # (H,H)

            pair_scale = note_weights[i] * note_weights[j]
            if mode == "rms":
                v = pair_scale * pair.sum(axis=1)
                acc_sum_sq = acc_sum_sq + (v * v)
                acc_count += 1
            elif mode == "sum":
                acc = acc + pair_scale * pair.sum(axis=1)
            else:
                acc = np.maximum(acc, pair_scale * pair.max(axis=1))

        if mode == "rms":
            denom = float(acc_count) if acc_count > 0 else 1.0
            out[i] = np.sqrt(acc_sum_sq / denom)
        else:
            out[i] = acc

    # Return as (H, N)
    return out.T


def chord_pairwise_dissonance_matrix(
    chord_freqs_hz: Sequence[float],
    overtone_weights: np.ndarray,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    fall_to_zero_semitones_c2: float = 12.0,
    fall_to_zero_semitones_c6: float = 12.0,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    lowpass_cutoff_hz: float = 20000.0,
    lowpass_slope_db_per_oct: float = 0.0,
    lowpass_renormalize: bool = True,
    sine_kernel: Literal["linear", "exponential"] = "linear",
    extension_freq_hz: float | None = None,
    extension_weight: float = 1.0,
) -> np.ndarray:
    """Pairwise note-vs-note dissonance matrix for a chord.

    Returns an array shaped (n_notes, n_notes) where entry (i,j) is the
    overtone-weighted roughness between note i and note j (summing over all
    harmonic pairs), including low-pass shaping and extension weighting.

    Diagonal is 0.
    """

    freqs = np.asarray([float(f) for f in chord_freqs_hz], dtype=np.float64)
    n = int(freqs.size)
    if n == 0:
        return np.zeros((0, 0), dtype=np.float64)

    ext = float(extension_freq_hz) if extension_freq_hz is not None else None
    ext_w = float(extension_weight)

    def _is_ext(f0: float) -> bool:
        if ext is None:
            return False
        return bool(np.isclose(float(f0), ext, rtol=1e-9, atol=1e-6))

    note_weights = np.array([ext_w if _is_ext(float(f)) else 1.0 for f in freqs], dtype=np.float64)

    M = np.zeros((n, n), dtype=np.float64)
    for i in range(n):
        for j in range(i + 1, n):
            d = float(
                overtone_dissonance(
                    freqs[i],
                    freqs[j],
                    overtone_weights,
                    peak_semitones_c2=peak_semitones_c2,
                    peak_semitones_c6=peak_semitones_c6,
                    fall_to_zero_semitones_c2=fall_to_zero_semitones_c2,
                    fall_to_zero_semitones_c6=fall_to_zero_semitones_c6,
                    decay_db_per_oct_c2=decay_db_per_oct_c2,
                    decay_db_per_oct_c6=decay_db_per_oct_c6,
                    height_c2=height_c2,
                    height_c6=height_c6,
                    sine_kernel=sine_kernel,
                    lowpass_cutoff_hz=lowpass_cutoff_hz,
                    lowpass_slope_db_per_oct=lowpass_slope_db_per_oct,
                    lowpass_renormalize=lowpass_renormalize,
                )
            )
            d = d * float(note_weights[i]) * float(note_weights[j])
            M[i, j] = d
            M[j, i] = d

    return M


def _mask_excluding_nearby_ticks(
    all_ticks: np.ndarray,
    chosen_ticks: Sequence[int],
    radius_ticks: int,
) -> np.ndarray:
    """Mask for excluding ticks within +/- radius_ticks of any chosen tick.

    Assumes all_ticks is a dense arange with step=1.
    """

    ticks = np.asarray(all_ticks, dtype=np.int64)
    mask = np.ones_like(ticks, dtype=bool)
    r = int(radius_ticks)
    if ticks.size == 0 or len(chosen_ticks) == 0:
        return mask

    # If r == 0, exclude only the exact chosen ticks (prevents duplicates).
    if r == 0:
        min_tick = int(ticks[0])
        n = int(mask.size)
        for t in chosen_ticks:
            idx = int(t) - min_tick
            if 0 <= idx < n:
                mask[idx] = False
        return mask

    if r < 0:
        return mask

    min_tick = int(ticks[0])
    n = int(mask.size)
    for t in chosen_ticks:
        start = max(0, int(t) - r - min_tick)
        end = min(n, int(t) + r - min_tick + 1)
        if start < end:
            mask[start:end] = False
    return mask


def _microtonal_exclusion_radius_ticks(steps_per_semitone: int, radius_semitones: float = 0.25) -> int:
    """Radius in integer ticks to exclude notes closer than `radius_semitones`.

    We treat the radius as *strictly less than* the given semitone distance.
    This ensures that with 4 steps/semitone (quarter-tone grid), a distance of
    exactly 0.25 semitones is allowed (so microtonal mode doesn't remove the
    original semitone-grid candidates once a quarter-step note is chosen).
    """

    steps = int(steps_per_semitone)
    if steps <= 1:
        return 0

    r = float(radius_semitones)
    if r <= 0.0:
        return 0

    return max(0, int(np.floor(r * steps - 1e-12)))


def select_chord_midis_greedy_harmonic_subharmonic(
    root_note: str,
    extension_note: str,
    overtone_weights: np.ndarray,
    min_note: str,
    max_note: str,
    n_additional: int,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    fall_to_zero_semitones_c2: float = 12.0,
    fall_to_zero_semitones_c6: float = 12.0,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    lowpass_cutoff_hz: float = 20000.0,
    lowpass_slope_db_per_oct: float = 0.0,
    lowpass_renormalize: bool = True,
    sine_kernel: Literal["linear", "exponential"] = "linear",
    set_aggregation: Literal["sum", "max", "rms"] = "sum",
    extension_weight: float = 1.0,
    below_root_penalty_db_per_oct: float = 0.0,
    above_extension_penalty_db_per_oct: float = 0.0,
    a4_hz: float = 440.0,
    candidate_steps_per_semitone: int = 1,
    search_n_harmonics: int = 16,
) -> List[float]:
    """Greedy selection where candidates come only from harmonics+subharmonics.

        Candidate fundamentals are generated from each currently-chosen note f as:
            - harmonics:    f * k
            - subharmonics: f / k
        for k=2..search_n_harmonics.

        Pitch handling:
            - If candidate_steps_per_semitone == 1: candidates are snapped to semitones.
            - If candidate_steps_per_semitone > 1 (microtonal mode): candidates are
                evaluated as *continuous* frequencies with no grid snapping.

    `search_n_harmonics` controls how many harmonic/subharmonic relations are used
    to generate candidates (k=2..K). This is independent of the number of overtones
    used in the dissonance calculation (i.e. len(overtone_weights)).
    """

    min_midi = note_to_midi(min_note)
    max_midi = note_to_midi(max_note)
    if max_midi <= min_midi:
        raise ValueError("max_note must be above min_note")

    steps = int(candidate_steps_per_semitone)
    if steps <= 0:
        raise ValueError("candidate_steps_per_semitone must be >= 1")

    root_midi = note_to_midi(root_note)
    ext_midi = note_to_midi(extension_note)
    root_freq = midi_to_frequency(root_midi, a4_hz=a4_hz)
    ext_freq = midi_to_frequency(ext_midi, a4_hz=a4_hz)
    chosen_freqs: list[float] = [float(root_freq)]
    chosen_midis_int: list[int] = [int(root_midi)]
    if int(ext_midi) != int(root_midi):
        chosen_freqs.append(float(ext_freq))
        chosen_midis_int.append(int(ext_midi))
    ext_w = float(extension_weight)
    if int(ext_midi) == int(root_midi):
        ext_w = 1.0

    min_freq = midi_to_frequency(min_midi, a4_hz=a4_hz)
    max_freq = midi_to_frequency(max_midi, a4_hz=a4_hz)

    K = int(search_n_harmonics)
    if K < 2:
        K = 2

    def _freq_to_midi(f_hz: float) -> float | None:
        f = float(f_hz)
        if not np.isfinite(f) or f <= 0.0:
            return None
        return float(69.0 + 12.0 * np.log2(f / float(a4_hz)))

    def _exclude_nearby_midis(cand_midis: np.ndarray, chosen_midis: Sequence[float], radius_semitones: float = 0.25) -> np.ndarray:
        if cand_midis.size == 0:
            return cand_midis
        r = float(radius_semitones)
        if r <= 0:
            return cand_midis

        chosen = np.asarray([float(m) for m in chosen_midis], dtype=np.float64)
        if chosen.size == 0:
            return cand_midis

        # Exclude candidates within strictly < r semitones of any chosen.
        d = np.abs(cand_midis[:, None] - chosen[None, :])
        keep = np.all(d >= (r - 1e-12), axis=1)
        return cand_midis[keep]

    for _ in range(int(n_additional)):
        # Generate candidate fundamentals from harmonic/subharmonic relations.
        cand_freqs: list[float] = []
        for f0 in chosen_freqs:
            f0 = float(f0)
            if not np.isfinite(f0) or f0 <= 0.0:
                continue

            for k in range(2, K + 1):
                cand_freqs.append(f0 * float(k))
                cand_freqs.append(f0 / float(k))

        if not cand_freqs:
            raise RuntimeError("No harmonic/subharmonic candidates generated")

        cand_freqs_arr = np.asarray(cand_freqs, dtype=np.float64)
        cand_freqs_arr = cand_freqs_arr[np.isfinite(cand_freqs_arr)]
        cand_freqs_arr = cand_freqs_arr[(cand_freqs_arr > 0.0) & (cand_freqs_arr >= (min_freq - 1e-12)) & (cand_freqs_arr <= (max_freq + 1e-12))]

        # Convert to midis for boundary penalties and de-dup.
        cand_midis_arr = np.asarray([m for m in (_freq_to_midi(f) for f in cand_freqs_arr.tolist()) if m is not None], dtype=np.float64)
        if cand_midis_arr.size == 0:
            # If range is too tight, fall back to semitone grid candidates.
            all_midis = np.arange(min_midi, max_midi + 1, dtype=np.float64)
            cand_midis_arr = all_midis

        if steps > 1:
            # Microtonal: do NOT snap; keep continuous candidates.
            candidate_midis = _exclude_nearby_midis(cand_midis_arr, chosen_midis=[_freq_to_midi(f) for f in chosen_freqs if _freq_to_midi(f) is not None])
            # De-dup with a fine rounding to prevent huge repeats.
            candidate_midis = np.unique(np.round(candidate_midis, 6))
        else:
            # Semitone mode: snap to integer midis.
            candidate_midis = np.unique(np.round(cand_midis_arr))
            if chosen_midis_int:
                candidate_midis = candidate_midis[~np.isin(candidate_midis, np.asarray(chosen_midis_int, dtype=np.float64))]

        # Remove any out-of-range candidates (after snapping/rounding).
        candidate_midis = candidate_midis[(candidate_midis >= float(min_midi) - 1e-9) & (candidate_midis <= float(max_midi) + 1e-9)]

        if candidate_midis.size == 0:
            # As a last resort, fall back to the full semitone grid.
            candidate_midis = np.arange(min_midi, max_midi + 1, dtype=np.float64)

        candidate_freqs = midi_to_frequency_continuous(candidate_midis, a4_hz=a4_hz)

        if int(ext_midi) == int(root_midi):
            chosen_weights = [1.0] + [1.0] * max(0, len(chosen_freqs) - 1)
        else:
            chosen_weights = [1.0, ext_w] + [1.0] * max(0, len(chosen_freqs) - 2)
        D = dissonance_to_set(
            candidate_freqs,
            chosen_freqs,
            overtone_weights,
            peak_semitones_c2=peak_semitones_c2,
            peak_semitones_c6=peak_semitones_c6,
            fall_to_zero_semitones_c2=fall_to_zero_semitones_c2,
            fall_to_zero_semitones_c6=fall_to_zero_semitones_c6,
            decay_db_per_oct_c2=decay_db_per_oct_c2,
            decay_db_per_oct_c6=decay_db_per_oct_c6,
            height_c2=height_c2,
            height_c6=height_c6,
            lowpass_cutoff_hz=lowpass_cutoff_hz,
            lowpass_slope_db_per_oct=lowpass_slope_db_per_oct,
            lowpass_renormalize=lowpass_renormalize,
            sine_kernel=sine_kernel,
            set_aggregation=set_aggregation,
            existing_weights=chosen_weights,
        )
        D = D + _boundary_penalty_midis(
            candidate_midis,
            root_midi=root_midi,
            extension_midi=ext_midi,
            below_root_db_per_oct=below_root_penalty_db_per_oct,
            above_extension_db_per_oct=above_extension_penalty_db_per_oct,
        )

        best_idx = int(np.argmin(D))
        chosen_freqs.append(float(candidate_freqs[best_idx]))
        if steps == 1:
            chosen_midis_int.append(int(np.round(float(candidate_midis[best_idx]))))

    # Return as (possibly fractional) midis.
    out_midis = [float(69.0 + 12.0 * np.log2(float(f) / float(a4_hz))) for f in chosen_freqs]
    return sorted(out_midis)


@dataclass(frozen=True)
class ChordResult:
    chord_midis_sorted: List[float]
    chord_notes_sorted: List[str]
    chord_freqs_sorted_hz: List[float]
    dissonance_curve_midis: List[float]
    dissonance_curve_freqs_hz: List[float]
    dissonance_curve_values: List[float]


def curve_for_fixed_chord(
    chord_midis: Sequence[float],
    root_note: str,
    extension_note: str,
    min_note: str,
    max_note: str,
    overtone_weights: np.ndarray,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    fall_to_zero_semitones_c2: float = 12.0,
    fall_to_zero_semitones_c6: float = 12.0,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    lowpass_cutoff_hz: float = 20000.0,
    lowpass_slope_db_per_oct: float = 0.0,
    lowpass_renormalize: bool = True,
    sine_kernel: Literal["linear", "exponential"] = "linear",
    set_aggregation: Literal["sum", "max", "rms"] = "sum",
    extension_weight: float = 1.0,
    below_root_penalty_db_per_oct: float = 0.0,
    above_extension_penalty_db_per_oct: float = 0.0,
    curve_steps_per_semitone: int = 5,
    a4_hz: float = 440.0,
) -> ChordResult:
    """Compute a high-resolution dissonance curve for a fixed chord assignment."""

    if not chord_midis:
        raise ValueError("chord_midis must be non-empty")

    min_midi = note_to_midi(min_note)
    max_midi = note_to_midi(max_note)
    if max_midi <= min_midi:
        raise ValueError("max_note must be above min_note")

    raw_midis_sorted = sorted(float(m) for m in chord_midis)
    chord_midis_sorted: list[float] = []
    for m in raw_midis_sorted:
        if not chord_midis_sorted:
            chord_midis_sorted.append(float(m))
            continue
        if not np.isclose(float(m), float(chord_midis_sorted[-1]), rtol=0.0, atol=1e-9):
            chord_midis_sorted.append(float(m))
    chord_midis_arr = np.asarray(chord_midis_sorted, dtype=np.float64)
    chord_freqs_sorted = midi_to_frequency_continuous(chord_midis_arr, a4_hz=a4_hz).tolist()
    chord_notes_sorted = [midi_to_note_microtonal(m) for m in chord_midis_sorted]

    steps = int(curve_steps_per_semitone)
    if steps <= 0:
        raise ValueError("curve_steps_per_semitone must be >= 1")
    midi_step = 1.0 / float(steps)

    curve_midis_arr = np.arange(min_midi, (max_midi + 1) + 1e-9, midi_step, dtype=np.float64)
    curve_freqs_arr = midi_to_frequency_continuous(curve_midis_arr, a4_hz=a4_hz)

    ext_midi = float(note_to_midi(extension_note))
    ext_w = float(extension_weight)
    chord_weights = [ext_w if np.isclose(m, ext_midi, rtol=0.0, atol=1e-9) else 1.0 for m in chord_midis_sorted]
    cand_weight = np.where(np.isclose(curve_midis_arr, ext_midi, rtol=0.0, atol=1e-9), ext_w, 1.0)

    curve_values = dissonance_to_set(
        curve_freqs_arr,
        chord_freqs_sorted,
        overtone_weights,
        peak_semitones_c2=peak_semitones_c2,
        peak_semitones_c6=peak_semitones_c6,
        fall_to_zero_semitones_c2=fall_to_zero_semitones_c2,
        fall_to_zero_semitones_c6=fall_to_zero_semitones_c6,
        decay_db_per_oct_c2=decay_db_per_oct_c2,
        decay_db_per_oct_c6=decay_db_per_oct_c6,
        height_c2=height_c2,
        height_c6=height_c6,
        lowpass_cutoff_hz=lowpass_cutoff_hz,
        lowpass_slope_db_per_oct=lowpass_slope_db_per_oct,
        lowpass_renormalize=lowpass_renormalize,
        sine_kernel=sine_kernel,
        set_aggregation=set_aggregation,
        existing_weights=chord_weights,
        candidate_weight=cand_weight,
    )
    root_midi = note_to_midi(root_note)
    ext_midi_i = note_to_midi(extension_note)
    curve_values = (
        curve_values
        + _boundary_penalty_midis(
            curve_midis_arr,
            root_midi=root_midi,
            extension_midi=ext_midi_i,
            below_root_db_per_oct=below_root_penalty_db_per_oct,
            above_extension_db_per_oct=above_extension_penalty_db_per_oct,
        )
    ).tolist()

    return ChordResult(
        chord_midis_sorted=chord_midis_sorted,
        chord_notes_sorted=chord_notes_sorted,
        chord_freqs_sorted_hz=chord_freqs_sorted,
        dissonance_curve_midis=curve_midis_arr.tolist(),
        dissonance_curve_freqs_hz=curve_freqs_arr.tolist(),
        dissonance_curve_values=curve_values,
    )


def make_note_list(min_note: str = "A2", max_note: str = "E6") -> List[str]:
    min_midi = note_to_midi(min_note)
    max_midi = note_to_midi(max_note)
    return [midi_to_note(m) for m in range(min_midi, max_midi + 1)]
