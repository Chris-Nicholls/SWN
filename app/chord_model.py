from __future__ import annotations

from dataclasses import dataclass
from typing import List, Literal, Sequence

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


def _boundary_penalty_freqs(
    candidate_freqs_hz: np.ndarray,
    root_freq_hz: float,
    extension_freq_hz: float,
    below_root_db_per_oct: float = 0.0,
    above_extension_db_per_oct: float = 0.0,
) -> np.ndarray:
    """Additive penalty for notes below root / above extension in frequency domain.

    Penalty increases linearly with octaves outside the
    [min(root_freq, ext_freq), max(root_freq, ext_freq)] boundaries.
    """

    f = np.asarray(candidate_freqs_hz, dtype=np.float64)
    f = np.maximum(f, 1e-12)

    low = float(min(float(root_freq_hz), float(extension_freq_hz)))
    high = float(max(float(root_freq_hz), float(extension_freq_hz)))
    low = max(low, 1e-12)
    high = max(high, low * 1.0000001)

    out = np.zeros_like(f)

    below = float(below_root_db_per_oct)
    if below > 0.0:
        # If f < low, octaves below = log2(low/f)
        oct_below = np.maximum(0.0, np.log2(low / f))
        slope = np.power(10.0, below / 20.0) - 1.0
        out = out + slope * oct_below

    above = float(above_extension_db_per_oct)
    if above > 0.0:
        # If f > high, octaves above = log2(f/high)
        oct_above = np.maximum(0.0, np.log2(f / high))
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

    # Piecewise-linear triangular kernel with constant slope.
    # - Calculate as if peak = 1.0
    # - Then shift entire curve down by (1.0 - height)
    # - Slope on falling edge: -1/fall_to_zero (constant, independent of height)
    # - Zero crossing occurs at: peak + (height * fall_to_zero)
    fall = np.maximum(fall, 1e-12)

    # Calculate kernel as if peak height is 1.0
    up = np.where(x_peak > 1e-12, x / x_peak, 0.0)
    down = 1.0 - (x - x_peak) / fall

    # Shift entire curve down by (1.0 - height)
    shift = 1.0 - height
    D = np.where(x <= x_peak, up - shift, down - shift)
    return np.clip(D, 0.0, None)


def overtone_dissonance(
    f1_hz: np.ndarray,
    f2_hz: np.ndarray,
    overtone_weights: np.ndarray,
    include_subharmonics: bool = False,
    subharmonic_weights: np.ndarray | None = None,
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
    fundamental_only_f1: bool = False,
    fundamental_only_f2: bool = False,
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

    n_over = int(w.size)

    def _weights_for_tone(use_sub: bool) -> np.ndarray:
        if not use_sub:
            return w

        if subharmonic_weights is None:
            w_sub = w[1:]
        else:
            ws = np.asarray(subharmonic_weights, dtype=np.float64)
            ws = np.clip(ws, 0.0, None)
            if ws.size == n_over:
                w_sub = ws[1:]
            elif ws.size == (n_over - 1):
                w_sub = ws
            else:
                # Fall back to the legacy behavior if shape is unexpected.
                w_sub = w[1:]

        return np.concatenate([w, w_sub])

    def _mult_and_weights(*, fundamental_only: bool) -> tuple[np.ndarray, np.ndarray]:
        if fundamental_only:
            return np.asarray([1.0], dtype=np.float64), np.asarray([w[0]], dtype=np.float64)

        use_sub = bool(include_subharmonics) and n_over >= 2
        mult_h = np.arange(1, n_over + 1, dtype=np.float64)
        if use_sub:
            mult_s = 1.0 / np.arange(2, n_over + 1, dtype=np.float64)
            mult = np.concatenate([mult_h, mult_s])
            return mult, _weights_for_tone(True)

        return mult_h, _weights_for_tone(False)

    mult1, w_full1 = _mult_and_weights(fundamental_only=bool(fundamental_only_f1))
    mult2, w_full2 = _mult_and_weights(fundamental_only=bool(fundamental_only_f2))

    f1_base = np.asarray(f1_hz, dtype=np.float64)
    f2_base = np.asarray(f2_hz, dtype=np.float64)

    # Partial frequencies per tone (used for low-pass weighting).
    f1_part = f1_base[..., None] * mult1[None, :]
    f2_part = f2_base[..., None] * mult2[None, :]

    g1 = _lowpass_gain(f1_part, cutoff_hz=lowpass_cutoff_hz, slope_db_per_oct=lowpass_slope_db_per_oct)
    g2 = _lowpass_gain(f2_part, cutoff_hz=lowpass_cutoff_hz, slope_db_per_oct=lowpass_slope_db_per_oct)

    w1_raw = g1 * w_full1[None, :]
    w2_raw = g2 * w_full2[None, :]
    if bool(lowpass_renormalize):
        w1 = _normalize_weights_last_dim(w1_raw)
        w2 = _normalize_weights_last_dim(w2_raw)
    else:
        w1 = w1_raw
        w2 = w2_raw

    f1 = f1_base[..., None, None] * mult1[None, :, None]
    f2 = f2_base[..., None, None] * mult2[None, None, :]
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

    # Apply harmonic weights using appropriate semantics for the kernel type
    mode = str(sine_kernel).strip().lower()
    if mode == "linear":
        # Linear kernel: use shift semantics to preserve constant slope
        # Shift down by (1 - w_outer) instead of multiplying
        result = np.maximum(base - (1.0 - w_outer), 0.0)
        return np.sum(result, axis=(-2, -1)).squeeze()
    else:
        # Exponential kernel: use multiplicative scaling
        return np.sum(w_outer * base, axis=(-2, -1)).squeeze()


def dissonance_to_set(
    candidate_freqs_hz: np.ndarray,
    existing_freqs_hz: Sequence[float],
    overtone_weights: np.ndarray,
    include_subharmonics: bool = False,
    subharmonic_weights: np.ndarray | None = None,
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
    set_aggregation: Literal["mean", "sum", "max", "rms"] = "mean",
    candidate_fundamental_only: bool = False,
    existing_weights: Sequence[float] | None = None,
    candidate_weight: float | np.ndarray = 1.0,
) -> np.ndarray:
    """Dissonance of candidate(s) against a fixed set."""

    candidate = np.asarray(candidate_freqs_hz, dtype=np.float64)
    mode = str(set_aggregation).strip().lower()
    # Backward compatible alias: historically this mode was called "sum".
    if mode == "sum":
        mode = "mean"
    if mode not in ("mean", "max", "rms"):
        raise ValueError("set_aggregation must be 'mean', 'max', or 'rms'")

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
                    include_subharmonics=include_subharmonics,
                    subharmonic_weights=subharmonic_weights,
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
                    fundamental_only_f1=bool(candidate_fundamental_only),
                )
                v = float(w_exist) * d
                sum_sq = sum_sq + (v * v)
                count += 1

            denom = float(count) if count > 0 else 1.0
            total_here = np.sqrt(sum_sq / denom)
            return total_here * w_cand

        total_here = np.zeros_like(freqs)
        count = 0
        for f_exist, w_exist in zip(existing, weights_existing):
            d = overtone_dissonance(
                freqs,
                f_exist,
                overtone_weights,
                include_subharmonics=include_subharmonics,
                subharmonic_weights=subharmonic_weights,
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
                fundamental_only_f1=bool(candidate_fundamental_only),
            )
            if mode == "mean":
                total_here = total_here + float(w_exist) * d
                count += 1
            else:
                total_here = np.maximum(total_here, float(w_exist) * d)

        if mode == "mean":
            denom = float(count) if count > 0 else 1.0
            total_here = total_here / denom

        return total_here * w_cand

    return _total_at(candidate)
    

def select_chord_freqs_greedy_harmonic_subharmonic(
    root_freq_hz: float,
    extension_freq_hz: float,
    overtone_weights: np.ndarray,
    min_freq_hz: float,
    max_freq_hz: float,
    n_additional: int,
    include_subharmonics: bool = False,
    subharmonic_weights: np.ndarray | None = None,
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
    set_aggregation: Literal["mean", "sum", "max", "rms"] = "mean",
    candidate_fundamental_only: bool = False,
    below_root_penalty_db_per_oct: float = 0.0,
    above_extension_penalty_db_per_oct: float = 0.0,
    a4_hz: float = 440.0,
    quantize_to_semitones: bool = True,
    search_n_harmonics: int = 16,
    prev_chord_freqs_hz: Sequence[float] | None = None,
    prev_chord_weight: float = 0.0,
    return_stats: bool = False,
) -> List[float] | tuple[List[float], "ChordSearchStats"]:
    """Greedy selection where candidates come only from harmonics+subharmonics.

        Candidate fundamentals are generated from each currently-chosen note f as:
            - harmonics:    f * k
            - subharmonics: f / k
        for k=2..search_n_harmonics.

        Pitch handling:
            - If quantize_to_semitones is True: candidates are snapped to semitones.
            - If quantize_to_semitones is False: candidates are evaluated as continuous
              frequencies with no snapping.

        Return value:
            - Always returns chord frequencies in Hz (sorted, deduplicated).

    `search_n_harmonics` controls how many harmonic/subharmonic relations are used
    to generate candidates (k=2..K). This is independent of the number of overtones
    used in the dissonance calculation (i.e. len(overtone_weights)).
    """

    snap = bool(quantize_to_semitones)

    root_freq = float(root_freq_hz)
    ext_freq = float(extension_freq_hz)
    if not np.isfinite(root_freq) or root_freq <= 0.0:
        raise ValueError("root_freq_hz must be finite and > 0")
    if not np.isfinite(ext_freq) or ext_freq <= 0.0:
        raise ValueError("extension_freq_hz must be finite and > 0")

    min_freq = float(min_freq_hz)
    max_freq = float(max_freq_hz)
    if not np.isfinite(min_freq) or not np.isfinite(max_freq) or min_freq <= 0.0 or max_freq <= 0.0:
        raise ValueError("min_freq_hz/max_freq_hz must be finite and > 0")
    if max_freq <= min_freq:
        raise ValueError("max_freq_hz must be above min_freq_hz")

    def _freqs_to_midis(freqs_hz: np.ndarray) -> np.ndarray:
        f = np.asarray(freqs_hz, dtype=np.float64)
        # Caller is expected to have filtered to finite, >0 frequencies.
        return 69.0 + 12.0 * np.log2(f / float(a4_hz))

    chosen_freqs: list[float] = [float(root_freq)]
    chosen_midis_int: list[int] = []
    if snap:
        root_midi_int = int(round(float(_freqs_to_midis(np.asarray([root_freq], dtype=np.float64))[0])))
        chosen_midis_int = [root_midi_int]
        chosen_freqs = [float(midi_to_frequency(root_midi_int, a4_hz=a4_hz))]

    chosen_midis_set: set[int] = set(int(m) for m in chosen_midis_int)

    def _append_snapped_midi(midi_int: int) -> None:
        mi = int(midi_int)
        if mi in chosen_midis_set:
            return
        chosen_midis_set.add(mi)
        chosen_midis_int.append(mi)
        chosen_freqs.append(float(midi_to_frequency(mi, a4_hz=a4_hz)))

    same_root_ext = bool(np.isclose(ext_freq, float(chosen_freqs[0]), rtol=0.0, atol=1e-12))

    # Only add extension note if it's different from root.
    include_extension = not same_root_ext
    if include_extension:
        if snap:
            ext_midi_int = int(round(float(_freqs_to_midis(np.asarray([ext_freq], dtype=np.float64))[0])))
            _append_snapped_midi(ext_midi_int)
        else:
            chosen_freqs.append(float(ext_freq))

    candidates_evaluated = 0

    # Voice leading (sequencer mode): seed the chord from the previous chord,
    # then improve by replacing the most dissonant carried-over notes first.
    vl_strength = float(prev_chord_weight)
    if not np.isfinite(vl_strength):
        vl_strength = 0.0
    # Interpret as a [0,1] "keep current note" strength.
    # 0 => no bias to keep; 1 => strongly prefer keeping the carried note.
    vl_strength = float(np.clip(vl_strength, 0.0, 1.0))

    K = int(search_n_harmonics)
    if K < 2:
        K = 2

    def _exclude_nearby_freqs_mask(
        cand_freqs_hz: np.ndarray,
        chosen_freqs_hz: Sequence[float],
        radius_semitones: float = 0.25,
    ) -> np.ndarray:
        cand = np.asarray(cand_freqs_hz, dtype=np.float64)
        if cand.size == 0:
            return np.zeros((0,), dtype=bool)

        r = float(radius_semitones)
        if r <= 0.0:
            return np.ones((cand.size,), dtype=bool)

        chosen = np.asarray([float(f) for f in chosen_freqs_hz], dtype=np.float64)
        chosen = chosen[np.isfinite(chosen)]
        chosen = chosen[chosen > 0.0]
        if chosen.size == 0:
            return np.ones((cand.size,), dtype=bool)

        cand = np.maximum(cand, 1e-12)
        chosen = np.maximum(chosen, 1e-12)

        # Semitone distance: 12*abs(log2(f_cand/f_chosen)).
        d = 12.0 * np.abs(np.log2(cand[:, None] / chosen[None, :]))
        keep = np.all(d >= (r - 1e-12), axis=1)
        return keep

    def _weights_for_existing(existing_freqs_hz: Sequence[float]) -> list[float]:
        return [1.0] * len(existing_freqs_hz)

    def _objective(
        cand_freqs_hz: np.ndarray,
        existing_freqs_hz: Sequence[float],
        *,
        existing_weights: Sequence[float],
    ) -> np.ndarray:
        diss = dissonance_to_set(
            cand_freqs_hz,
            existing_freqs_hz,
            overtone_weights,
            include_subharmonics=include_subharmonics,
            subharmonic_weights=subharmonic_weights,
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
            candidate_fundamental_only=bool(candidate_fundamental_only),
            existing_weights=existing_weights,
        )
        return diss + _boundary_penalty_freqs(
            np.asarray(cand_freqs_hz, dtype=np.float64),
            root_freq_hz=root_freq,
            extension_freq_hz=ext_freq,
            below_root_db_per_oct=below_root_penalty_db_per_oct,
            above_extension_db_per_oct=above_extension_penalty_db_per_oct,
        )

    def _candidate_objective(
        cand_freqs_hz: np.ndarray,
        existing_freqs_hz: Sequence[float],
    ) -> np.ndarray:
        return _objective(
            cand_freqs_hz,
            existing_freqs_hz,
            existing_weights=_weights_for_existing(existing_freqs_hz),
        )

    def _freq_to_midi_int(freq_hz: float) -> int:
        m = _freqs_to_midis(np.asarray([float(freq_hz)], dtype=np.float64))
        return int(round(float(m[0])))

    def _generate_candidates(
        existing_freqs_hz: Sequence[float],
        quantize: bool = False,
    ) -> np.ndarray:
        cand_freqs: list[float] = []
        for f0 in existing_freqs_hz:
            f0 = float(f0)
            if not np.isfinite(f0) or f0 <= 0.0:
                continue
            for k in range(2, K + 1):
                cand_freqs.append(f0 * float(k))
                cand_freqs.append(f0 / float(k))

        if not cand_freqs:
            return np.zeros((0,), dtype=np.float64)

        cand_freqs_arr = np.asarray(cand_freqs, dtype=np.float64)
        cand_freqs_arr = cand_freqs_arr[np.isfinite(cand_freqs_arr)]
        cand_freqs_arr = cand_freqs_arr[(cand_freqs_arr > 0.0) & (cand_freqs_arr >= (min_freq - 1e-12)) & (cand_freqs_arr <= (max_freq + 1e-12))]
        if cand_freqs_arr.size == 0:
            return np.zeros((0,), dtype=np.float64)

        keep = _exclude_nearby_freqs_mask(cand_freqs_arr, chosen_freqs_hz=existing_freqs_hz, radius_semitones=0.25)
        candidate_freqs = cand_freqs_arr[keep]
        if candidate_freqs.size:
            candidate_freqs = np.unique(candidate_freqs)
        if candidate_freqs.size == 0:
            return np.zeros((0,), dtype=np.float64)

        if not quantize:
            return candidate_freqs
        
        # Quantize to MIDI and deduplicate before evaluation.
        # Use rounded MIDI values on both sides when excluding existing notes.
        candidate_midis_int = np.unique(np.rint(_freqs_to_midis(candidate_freqs)).astype(np.int64))
        exclude_midis_int = np.unique(
            np.rint(_freqs_to_midis(np.asarray(existing_freqs_hz, dtype=np.float64))).astype(np.int64)
        )
        candidate_midis_int = candidate_midis_int[~np.isin(candidate_midis_int, exclude_midis_int)]
    
        if candidate_midis_int.size == 0:
            return np.zeros((0,), dtype=np.float64)
        
        # Convert back to frequencies for evaluation
        return midi_to_frequency_continuous(candidate_midis_int.astype(np.float64), a4_hz=a4_hz)


    # If voice leading is enabled and we have a previous chord, build the chord
    # incrementally from the old chord:
    # - set new root+extension
    # - then for each remaining old note (one-by-one), either keep it or replace it
    #   with a better candidate given the notes chosen so far.
    if vl_strength > 0.0 and prev_chord_freqs_hz is not None:
        carried = [float(f) for f in prev_chord_freqs_hz if np.isfinite(float(f)) and float(f) > 0.0]

        # In snap mode, quantize carried tones consistently via MIDI int round-trip.
        if snap and carried:
            carried_midis_int = np.rint(_freqs_to_midis(np.asarray(carried, dtype=np.float64))).astype(np.int64)
            carried = midi_to_frequency_continuous(carried_midis_int.astype(np.float64), a4_hz=a4_hz).tolist()

        if not carried:
            carried = None

        # Processing order: start with the most dissonant carried-over note (against
        # the previous chord + new root and extension, then proceed in that sorted order.
        # Tie-breaker: lower frequency first for determinism.
        if carried:
            contrib: list[tuple[float, float]] = []
            mixed_chord = chosen_freqs + list(carried)
            for f in carried:
                d = float(_candidate_objective(np.asarray([float(f)], dtype=np.float64), mixed_chord)[0])
                contrib.append((d, float(f)))
            contrib.sort(key=lambda t: (-t[0], t[1]))
            carried = [float(f) for _, f in contrib]

        # Add up to n_additional voices from the previous chord, one-by-one.
        while len(chosen_freqs) < 2 + int(n_additional) and carried:
            f = carried.pop(0)
            if f in chosen_freqs:
                continue

            # Generate candidates from harmonic/subharmonic relations.
              
            cand_freqs = _generate_candidates(carried, quantize=snap)

            # Always include the current note as a candidate, and apply a
            # (1 - vl_strength) discount to *its dissonance term*.
            if cand_freqs.size:
                cand_freqs_all = np.concatenate([
                    np.asarray([f], dtype=np.float64),
                    np.asarray(cand_freqs, dtype=np.float64),
                ])
            else:
                cand_freqs_all = np.asarray([f], dtype=np.float64)
               
            #dedup candidates that are exactly the same frequency (e.g. from different relations)
            cand_freqs_all = np.unique(cand_freqs_all)
            candidates_evaluated += int(np.asarray(cand_freqs_all).size)

            # Evaluate objective for each candidate against the rest of the chord.
            mixed_chord = chosen_freqs + carried
            D = _objective(
                cand_freqs_all,
                mixed_chord,
                existing_weights=_weights_for_existing(chosen_freqs) + [vl_strength] * len(carried),
            )

            # Add the selected candidate (which may be the original note if it's still best) to the chord, and proceed to the next carried note.
            best_i = int(np.argmin(D))
            
            if snap:
                chosen_midi_int = _freq_to_midi_int(float(cand_freqs_all[best_i]))
                _append_snapped_midi(chosen_midi_int)
            else:
                if float(cand_freqs_all[best_i]) not in chosen_freqs:
                    chosen_freqs.append(float(cand_freqs_all[best_i]))
                

    # Add any remaining voices using the original greedy growth rule.
    n_existing_additional = max(0, len(chosen_freqs))
    n_to_add = max(0, int(n_additional) - int(n_existing_additional) +2)

    for _ in range(int(n_to_add)):
        # Generate candidate fundamentals from harmonic/subharmonic relations.
        # In snap mode, quantize and deduplicate before evaluation
        candidate_freqs = _generate_candidates(
            chosen_freqs,
            quantize=snap,
        )
        
        if candidate_freqs.size == 0:
            raise RuntimeError("No harmonic/subharmonic candidates generated")
        
        candidates_evaluated += int(candidate_freqs.size)

        D = _candidate_objective(candidate_freqs, chosen_freqs)

        best_idx = int(np.argmin(D))
        if snap:
            chosen_midi_int = _freq_to_midi_int(float(candidate_freqs[best_idx]))
            _append_snapped_midi(chosen_midi_int)
        else:
            chosen_freqs.append(float(candidate_freqs[best_idx]))

    if snap:
        # In snap mode we already maintain MIDI ints for uniqueness.
        uniq_midis_int = np.asarray(sorted(chosen_midis_set), dtype=np.float64)
        out_freqs = midi_to_frequency_continuous(uniq_midis_int, a4_hz=a4_hz).astype(np.float64)
        out = [float(f) for f in out_freqs.tolist()]
        if return_stats:
            return out, ChordSearchStats(candidates_evaluated=int(candidates_evaluated))
        return out

    # Continuous mode: return frequencies (dedup + sort for determinism).
    out_freqs = np.asarray([float(f) for f in chosen_freqs if np.isfinite(float(f)) and float(f) > 0.0], dtype=np.float64)
    if out_freqs.size:
        out_freqs = np.unique(out_freqs)
        out_freqs = np.sort(out_freqs)
    out = [float(f) for f in out_freqs.tolist()]
    if return_stats:
        return out, ChordSearchStats(candidates_evaluated=int(candidates_evaluated))
    return out

@dataclass(frozen=True)
class ChordSearchStats:
    candidates_evaluated: int

@dataclass(frozen=True)
class ChordResult:
    chord_notes_sorted: List[str]
    chord_freqs_sorted_hz: List[float]
    dissonance_curve_freqs_hz: List[float]
    dissonance_curve_values: List[float]


def curve_for_fixed_chord(
    chord_freqs_hz: Sequence[float],
    root_freq_hz: float,
    extension_freq_hz: float,
    min_freq_hz: float,
    max_freq_hz: float,
    overtone_weights: np.ndarray,
    include_subharmonics: bool = False,
    subharmonic_weights: np.ndarray | None = None,
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
    set_aggregation: Literal["mean", "sum", "max", "rms"] = "mean",
    candidate_fundamental_only: bool = False,
    below_root_penalty_db_per_oct: float = 0.0,
    above_extension_penalty_db_per_oct: float = 0.0,
    curve_steps_per_semitone: int = 5,
    a4_hz: float = 440.0,
) -> ChordResult:
    """Compute a high-resolution dissonance curve for a fixed chord assignment."""

    if not chord_freqs_hz:
        raise ValueError("chord_freqs_hz must be non-empty")

    root_freq = float(root_freq_hz)
    ext_freq = float(extension_freq_hz)
    min_freq = float(min_freq_hz)
    max_freq = float(max_freq_hz)
    if not np.isfinite(root_freq) or root_freq <= 0.0:
        raise ValueError("root_freq_hz must be finite and > 0")
    if not np.isfinite(ext_freq) or ext_freq <= 0.0:
        raise ValueError("extension_freq_hz must be finite and > 0")
    if not np.isfinite(min_freq) or not np.isfinite(max_freq) or min_freq <= 0.0 or max_freq <= 0.0:
        raise ValueError("min_freq_hz/max_freq_hz must be finite and > 0")
    if max_freq <= min_freq:
        raise ValueError("max_freq_hz must be above min_freq_hz")

    chord_freqs_arr = np.asarray([float(f) for f in chord_freqs_hz], dtype=np.float64)
    chord_freqs_arr = chord_freqs_arr[np.isfinite(chord_freqs_arr)]
    chord_freqs_arr = chord_freqs_arr[chord_freqs_arr > 0.0]
    if chord_freqs_arr.size == 0:
        raise ValueError("chord_freqs_hz must contain at least one finite, positive frequency")

    chord_freqs_arr = np.unique(chord_freqs_arr)
    chord_freqs_arr = np.sort(chord_freqs_arr)
    chord_freqs_sorted = chord_freqs_arr.tolist()

    # Note names are for display only; format from continuous pitch.
    chord_midis_cont = 69.0 + 12.0 * np.log2(chord_freqs_arr / float(a4_hz))
    chord_notes_sorted = [midi_to_note_microtonal(float(m)) for m in chord_midis_cont.tolist()]

    steps = int(curve_steps_per_semitone)
    if steps <= 0:
        raise ValueError("curve_steps_per_semitone must be >= 1")
    midi_step = 1.0 / float(steps)

    min_midi_cont = 69.0 + 12.0 * np.log2(float(min_freq) / float(a4_hz))
    max_midi_cont = 69.0 + 12.0 * np.log2(float(max_freq) / float(a4_hz))
    curve_midis_arr = np.arange(float(min_midi_cont), float(max_midi_cont) + (0.5 * midi_step), midi_step, dtype=np.float64)
    curve_freqs_arr = midi_to_frequency_continuous(curve_midis_arr, a4_hz=a4_hz)

    curve_values = dissonance_to_set(
        curve_freqs_arr,
        chord_freqs_sorted,
        overtone_weights,
        include_subharmonics=include_subharmonics,
        subharmonic_weights=subharmonic_weights,
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
        candidate_fundamental_only=bool(candidate_fundamental_only),
    )
    curve_values = (
        curve_values
        + _boundary_penalty_freqs(
            curve_freqs_arr,
            root_freq_hz=root_freq,
            extension_freq_hz=ext_freq,
            below_root_db_per_oct=below_root_penalty_db_per_oct,
            above_extension_db_per_oct=above_extension_penalty_db_per_oct,
        )
    ).tolist()

    return ChordResult(
        chord_notes_sorted=chord_notes_sorted,
        chord_freqs_sorted_hz=chord_freqs_sorted,
        dissonance_curve_freqs_hz=curve_freqs_arr.tolist(),
        dissonance_curve_values=curve_values,
    )


def make_note_list(min_note: str = "A2", max_note: str = "E6") -> List[str]:
    min_midi = note_to_midi(min_note)
    max_midi = note_to_midi(max_note)
    return [midi_to_note(m) for m in range(min_midi, max_midi + 1)]
