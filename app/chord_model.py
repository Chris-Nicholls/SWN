from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

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
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
) -> np.ndarray:
    """Sensory dissonance (roughness) between two pure sine waves (deterministic)."""
    f1 = np.asarray(f1_hz, dtype=np.float64)
    f2 = np.asarray(f2_hz, dtype=np.float64)

    # Semitone distance between frequencies (symmetric, 0 at unison).
    ratio = np.maximum(f2, 1e-12) / np.maximum(f1, 1e-12)
    x = np.abs(12.0 * np.log2(ratio))

    # Interpolate/extrapolate parameters by register using the lower frequency.
    f_min = np.maximum(np.minimum(f1, f2), 1e-9)
    x_peak = _register_geometric(f_min, peak_semitones_c2, peak_semitones_c6)
    decay = _register_geometric(f_min, decay_db_per_oct_c2, decay_db_per_oct_c6)
    height = np.maximum(_register_linear(f_min, height_c2, height_c6), 0.0)

    a, b = _ab_from_peak_semitones_and_decay_db_per_oct(x_peak, decay)

    base = np.exp(-a * x) - np.exp(-b * x)

    # Scale so the peak height matches `height`.
    peak_val = np.exp(-a * x_peak) - np.exp(-b * x_peak)
    scale = np.where(peak_val > 1e-12, height / peak_val, 0.0)
    D = scale * base
    return np.maximum(D, 0.0)


def overtone_dissonance(
    f1_hz: np.ndarray,
    f2_hz: np.ndarray,
    overtone_weights: np.ndarray,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
) -> np.ndarray:
    """Total dissonance between two complex tones with given overtone weights.

    Accepts broadcasting arrays for f1_hz/f2_hz.
    """
    w = normalize_overtone_weights(overtone_weights)
    n = np.arange(1, w.size + 1, dtype=np.float64)

    f1 = np.asarray(f1_hz, dtype=np.float64)[..., None, None] * n[None, :, None]
    f2 = np.asarray(f2_hz, dtype=np.float64)[..., None, None] * n[None, None, :]

    w_outer = (w[None, :, None] * w[None, None, :])

    base = sine_dissonance(
        f1,
        f2,
        peak_semitones_c2=peak_semitones_c2,
        peak_semitones_c6=peak_semitones_c6,
        decay_db_per_oct_c2=decay_db_per_oct_c2,
        decay_db_per_oct_c6=decay_db_per_oct_c6,
        height_c2=height_c2,
        height_c6=height_c6,
    )
    # det1 = sine_dissonance(f1 * 1.005, f2)
    # det2 = sine_dissonance(f1, f2 * 1.01)

    return (w_outer * base).sum(axis=(-2, -1))


def dissonance_to_set(
    candidate_freqs_hz: np.ndarray,
    existing_freqs_hz: Sequence[float],
    overtone_weights: np.ndarray,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    slope_weight: float = 0.0,
    slope_h_semitones: float = 0.2,
) -> np.ndarray:
    """Dissonance of candidate(s) against a fixed set, optionally penalising slope.

    If slope_weight > 0, we add a first-derivative term (magnitude of dD/dmidi)
    estimated by central difference using a small pitch step.

    This tends to prefer notes sitting in *wider basins* of low dissonance
    instead of narrow, steep valleys.
    """

    candidate = np.asarray(candidate_freqs_hz, dtype=np.float64)

    def _total_at(freqs: np.ndarray) -> np.ndarray:
        total_here = np.zeros_like(freqs)
        for f_exist in existing_freqs_hz:
            total_here = total_here + overtone_dissonance(
                freqs,
                float(f_exist),
                overtone_weights,
                peak_semitones_c2=peak_semitones_c2,
                peak_semitones_c6=peak_semitones_c6,
                decay_db_per_oct_c2=decay_db_per_oct_c2,
                decay_db_per_oct_c6=decay_db_per_oct_c6,
                height_c2=height_c2,
                height_c6=height_c6,
            )
        return total_here

    base = _total_at(candidate)

    w = float(slope_weight)
    if w <= 0.0:
        return base

    h = float(slope_h_semitones)
    if h <= 0.0:
        raise ValueError("slope_h_semitones must be > 0")

    # Central difference in MIDI (semitone) space: f(m+h) = f * 2^(h/12)
    ratio = float(2.0 ** (h / 12.0))
    plus = _total_at(candidate * ratio)
    minus = _total_at(candidate / ratio)
    slope_mag = np.abs(plus - minus) / (2.0 * h)

    return base + w * slope_mag


def average_note_dissonance(
    chord_freqs_hz: Sequence[float],
    overtone_weights: np.ndarray,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
) -> float:
    """Average per-note dissonance within the chord.

    For each note i, compute sum_j!=i D(i,j); return mean_i.
    """
    freqs = [float(f) for f in chord_freqs_hz]
    n = len(freqs)
    if n < 2:
        return 0.0

    per_note = []
    for i in range(n):
        others = [freqs[j] for j in range(n) if j != i]
        d = float(
            dissonance_to_set(
                np.array([freqs[i]], dtype=np.float64),
                others,
                overtone_weights,
                peak_semitones_c2=peak_semitones_c2,
                peak_semitones_c6=peak_semitones_c6,
                decay_db_per_oct_c2=decay_db_per_oct_c2,
                decay_db_per_oct_c6=decay_db_per_oct_c6,
                height_c2=height_c2,
                height_c6=height_c6,
            )[0]
        )
        per_note.append(d)
    return float(np.mean(per_note))


def select_chord_midis_greedy(
    root_note: str,
    extension_note: str,
    overtone_weights: np.ndarray,
    min_note: str,
    max_note: str,
    n_additional: int,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    below_root_penalty_db_per_oct: float = 0.0,
    above_extension_penalty_db_per_oct: float = 0.0,
    slope_weight: float = 0.0,
    slope_h_semitones: float = 0.2,
    a4_hz: float = 440.0,
) -> List[int]:
    """Greedy selection: iteratively add the least-dissonant candidate note."""
    min_midi = note_to_midi(min_note)
    max_midi = note_to_midi(max_note)
    if max_midi <= min_midi:
        raise ValueError("max_note must be above min_note")

    root_midi = note_to_midi(root_note)
    ext_midi = note_to_midi(extension_note)

    chosen_midis = [root_midi, ext_midi]
    chosen_freqs = [midi_to_frequency(root_midi, a4_hz=a4_hz), midi_to_frequency(ext_midi, a4_hz=a4_hz)]

    all_midis = np.arange(min_midi, max_midi + 1)
    all_freqs = np.array([midi_to_frequency(int(m), a4_hz=a4_hz) for m in all_midis], dtype=np.float64)

    for _ in range(int(n_additional)):
        mask = np.ones_like(all_midis, dtype=bool)
        for m in chosen_midis:
            mask &= all_midis != m
        candidate_midis = all_midis[mask]
        candidate_freqs = all_freqs[mask]

        D = dissonance_to_set(
            candidate_freqs,
            chosen_freqs,
            overtone_weights,
            peak_semitones_c2=peak_semitones_c2,
            peak_semitones_c6=peak_semitones_c6,
            decay_db_per_oct_c2=decay_db_per_oct_c2,
            decay_db_per_oct_c6=decay_db_per_oct_c6,
            height_c2=height_c2,
            height_c6=height_c6,
            slope_weight=slope_weight,
            slope_h_semitones=slope_h_semitones,
        )
        D = D + _boundary_penalty_midis(
            candidate_midis,
            root_midi=root_midi,
            extension_midi=ext_midi,
            below_root_db_per_oct=below_root_penalty_db_per_oct,
            above_extension_db_per_oct=above_extension_penalty_db_per_oct,
        )
        best_idx = int(np.argmin(D))
        chosen_midis.append(int(candidate_midis[best_idx]))
        chosen_freqs.append(float(candidate_freqs[best_idx]))

    return sorted(int(m) for m in chosen_midis)


def select_chord_midis_root_extension_priority(
    root_note: str,
    extension_note: str,
    overtone_weights: np.ndarray,
    min_note: str,
    max_note: str,
    n_additional: int,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    below_root_penalty_db_per_oct: float = 0.0,
    above_extension_penalty_db_per_oct: float = 0.0,
    slope_weight: float = 0.0,
    slope_h_semitones: float = 0.2,
    a4_hz: float = 440.0,
) -> List[int]:
    """Selection that prioritises keeping root/extension dissonance low.

    Each added note is chosen to minimise dissonance to the *root and extension only*.
    """
    min_midi = note_to_midi(min_note)
    max_midi = note_to_midi(max_note)
    if max_midi <= min_midi:
        raise ValueError("max_note must be above min_note")

    root_midi = note_to_midi(root_note)
    ext_midi = note_to_midi(extension_note)
    root_freq = midi_to_frequency(root_midi, a4_hz=a4_hz)
    ext_freq = midi_to_frequency(ext_midi, a4_hz=a4_hz)

    chosen_midis = [root_midi, ext_midi]
    anchor_freqs = [root_freq, ext_freq]

    all_midis = np.arange(min_midi, max_midi + 1)
    all_freqs = np.array([midi_to_frequency(int(m), a4_hz=a4_hz) for m in all_midis], dtype=np.float64)

    for _ in range(int(n_additional)):
        mask = np.ones_like(all_midis, dtype=bool)
        for m in chosen_midis:
            mask &= all_midis != m
        candidate_midis = all_midis[mask]
        candidate_freqs = all_freqs[mask]

        D = dissonance_to_set(
            candidate_freqs,
            anchor_freqs,
            overtone_weights,
            peak_semitones_c2=peak_semitones_c2,
            peak_semitones_c6=peak_semitones_c6,
            decay_db_per_oct_c2=decay_db_per_oct_c2,
            decay_db_per_oct_c6=decay_db_per_oct_c6,
            height_c2=height_c2,
            height_c6=height_c6,
            slope_weight=slope_weight,
            slope_h_semitones=slope_h_semitones,
        )
        D = D + _boundary_penalty_midis(
            candidate_midis,
            root_midi=root_midi,
            extension_midi=ext_midi,
            below_root_db_per_oct=below_root_penalty_db_per_oct,
            above_extension_db_per_oct=above_extension_penalty_db_per_oct,
        )
        best_idx = int(np.argmin(D))
        chosen_midis.append(int(candidate_midis[best_idx]))

    return sorted(int(m) for m in chosen_midis)


def select_chord_midis_optimal_search(
    root_note: str,
    extension_note: str,
    overtone_weights: np.ndarray,
    min_note: str,
    max_note: str,
    n_additional: int,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    below_root_penalty_db_per_oct: float = 0.0,
    above_extension_penalty_db_per_oct: float = 0.0,
    slope_weight: float = 0.0,
    slope_h_semitones: float = 0.2,
    a4_hz: float = 440.0,
    beam_width: int = 32,
) -> List[int]:
    """Beam search for an (approx) optimal chord assignment.

    Objective: minimise total pairwise dissonance within the final chord.
    (Exact search is combinatorial; beam search is a practical approximation.)
    """
    min_midi = note_to_midi(min_note)
    max_midi = note_to_midi(max_note)
    if max_midi <= min_midi:
        raise ValueError("max_note must be above min_note")
    if beam_width <= 0:
        raise ValueError("beam_width must be >= 1")

    root_midi = note_to_midi(root_note)
    ext_midi = note_to_midi(extension_note)
    initial_midis = [root_midi, ext_midi]
    initial_freqs = [midi_to_frequency(root_midi, a4_hz=a4_hz), midi_to_frequency(ext_midi, a4_hz=a4_hz)]

    # Initial pairwise cost is just between root and extension
    initial_cost = float(
        overtone_dissonance(
            np.array([initial_freqs[0]]),
            np.array([initial_freqs[1]]),
            overtone_weights,
            peak_semitones_c2=peak_semitones_c2,
            peak_semitones_c6=peak_semitones_c6,
            decay_db_per_oct_c2=decay_db_per_oct_c2,
            decay_db_per_oct_c6=decay_db_per_oct_c6,
            height_c2=height_c2,
            height_c6=height_c6,
        )[0]
    )

    all_midis = list(range(min_midi, max_midi + 1))
    all_freqs = {m: midi_to_frequency(int(m), a4_hz=a4_hz) for m in all_midis}

    # Each beam entry: (total_pairwise_cost, midis_list, freqs_list)
    beam: List[Tuple[float, List[int], List[float]]] = [(initial_cost, initial_midis, initial_freqs)]

    for _ in range(int(n_additional)):
        new_beam: List[Tuple[float, List[int], List[float]]] = []
        for cost, midis_list, freqs_list in beam:
            used = set(midis_list)
            candidates = [m for m in all_midis if m not in used]
            if not candidates:
                continue

            cand_freqs = np.array([all_freqs[m] for m in candidates], dtype=np.float64)
            # incremental cost: sum dissonance(new, each existing)
            inc = dissonance_to_set(
                cand_freqs,
                freqs_list,
                overtone_weights,
                peak_semitones_c2=peak_semitones_c2,
                peak_semitones_c6=peak_semitones_c6,
                decay_db_per_oct_c2=decay_db_per_oct_c2,
                decay_db_per_oct_c6=decay_db_per_oct_c6,
                height_c2=height_c2,
                height_c6=height_c6,
                slope_weight=slope_weight,
                slope_h_semitones=slope_h_semitones,
            )
            inc = inc + _boundary_penalty_midis(
                np.asarray(candidates, dtype=np.float64),
                root_midi=root_midi,
                extension_midi=ext_midi,
                below_root_db_per_oct=below_root_penalty_db_per_oct,
                above_extension_db_per_oct=above_extension_penalty_db_per_oct,
            )

            # take best few expansions from this state
            k = min(beam_width, len(candidates))
            best_idx = np.argpartition(inc, k - 1)[:k]
            for idx in best_idx:
                m = int(candidates[int(idx)])
                f = float(all_freqs[m])
                new_beam.append((float(cost + inc[int(idx)]), midis_list + [m], freqs_list + [f]))

        if not new_beam:
            break
        new_beam.sort(key=lambda t: t[0])
        beam = new_beam[:beam_width]

    best = min(beam, key=lambda t: t[0])
    return sorted(int(m) for m in best[1])


@dataclass(frozen=True)
class ChordResult:
    chord_midis_sorted: List[int]
    chord_notes_sorted: List[str]
    chord_freqs_sorted_hz: List[float]
    dissonance_curve_midis: List[float]
    dissonance_curve_freqs_hz: List[float]
    dissonance_curve_values: List[float]


def curve_for_fixed_chord(
    chord_midis: Sequence[int],
    root_note: str,
    extension_note: str,
    min_note: str,
    max_note: str,
    overtone_weights: np.ndarray,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    below_root_penalty_db_per_oct: float = 0.0,
    above_extension_penalty_db_per_oct: float = 0.0,
    slope_weight: float = 0.0,
    slope_h_semitones: float = 0.2,
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

    chord_midis_sorted = sorted(int(m) for m in chord_midis)
    chord_freqs_sorted = [midi_to_frequency(m, a4_hz=a4_hz) for m in chord_midis_sorted]
    chord_notes_sorted = [midi_to_note(m) for m in chord_midis_sorted]

    steps = int(curve_steps_per_semitone)
    if steps <= 0:
        raise ValueError("curve_steps_per_semitone must be >= 1")
    midi_step = 1.0 / float(steps)

    curve_midis_arr = np.arange(min_midi, (max_midi + 1) + 1e-9, midi_step, dtype=np.float64)
    curve_freqs_arr = midi_to_frequency_continuous(curve_midis_arr, a4_hz=a4_hz)
    curve_values = dissonance_to_set(
        curve_freqs_arr,
        chord_freqs_sorted,
        overtone_weights,
        peak_semitones_c2=peak_semitones_c2,
        peak_semitones_c6=peak_semitones_c6,
        decay_db_per_oct_c2=decay_db_per_oct_c2,
        decay_db_per_oct_c6=decay_db_per_oct_c6,
        height_c2=height_c2,
        height_c6=height_c6,
        slope_weight=slope_weight,
        slope_h_semitones=slope_h_semitones,
    )
    root_midi = note_to_midi(root_note)
    ext_midi = note_to_midi(extension_note)
    curve_values = (
        curve_values
        + _boundary_penalty_midis(
            curve_midis_arr,
            root_midi=root_midi,
            extension_midi=ext_midi,
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


def build_greedy_chord_and_curve(
    root_note: str,
    extension_note: str,
    overtone_weights: np.ndarray,
    min_note: str,
    max_note: str,
    n_additional: int,
    peak_semitones_c2: float = 1.00,
    peak_semitones_c6: float = 1.00,
    decay_db_per_oct_c2: float = 20.0,
    decay_db_per_oct_c6: float = 20.0,
    height_c2: float = 1.0,
    height_c6: float = 1.0,
    below_root_penalty_db_per_oct: float = 0.0,
    above_extension_penalty_db_per_oct: float = 0.0,
    slope_weight: float = 0.0,
    slope_h_semitones: float = 0.2,
    curve_steps_per_semitone: int = 5,
    a4_hz: float = 440.0,
) -> ChordResult:
    """Greedy chord builder: iteratively adds the least-dissonant candidate note.

    Also computes a dissonance curve for the final chord set.
    """

    min_midi = note_to_midi(min_note)
    max_midi = note_to_midi(max_note)
    if max_midi <= min_midi:
        raise ValueError("max_note must be above min_note")

    root_midi = note_to_midi(root_note)
    ext_midi = note_to_midi(extension_note)

    chosen_midis = [root_midi, ext_midi]
    chosen_freqs = [midi_to_frequency(root_midi, a4_hz=a4_hz), midi_to_frequency(ext_midi, a4_hz=a4_hz)]

    all_midis = np.arange(min_midi, max_midi + 1)
    all_freqs = np.array([midi_to_frequency(int(m), a4_hz=a4_hz) for m in all_midis], dtype=np.float64)

    for _ in range(int(n_additional)):
        mask = np.ones_like(all_midis, dtype=bool)
        for m in chosen_midis:
            mask &= all_midis != m
        candidate_midis = all_midis[mask]
        candidate_freqs = all_freqs[mask]

        D = dissonance_to_set(
            candidate_freqs,
            chosen_freqs,
            overtone_weights,
            peak_semitones_c2=peak_semitones_c2,
            peak_semitones_c6=peak_semitones_c6,
            decay_db_per_oct_c2=decay_db_per_oct_c2,
            decay_db_per_oct_c6=decay_db_per_oct_c6,
            height_c2=height_c2,
            height_c6=height_c6,
            slope_weight=slope_weight,
            slope_h_semitones=slope_h_semitones,
        )
        D = D + _boundary_penalty_midis(
            candidate_midis,
            root_midi=root_midi,
            extension_midi=ext_midi,
            below_root_db_per_oct=below_root_penalty_db_per_oct,
            above_extension_db_per_oct=above_extension_penalty_db_per_oct,
        )
        best_idx = int(np.argmin(D))

        chosen_midis.append(int(candidate_midis[best_idx]))
        chosen_freqs.append(float(candidate_freqs[best_idx]))

    chord_midis_sorted = sorted(chosen_midis)
    chord_freqs_sorted = [midi_to_frequency(m, a4_hz=a4_hz) for m in chord_midis_sorted]
    chord_notes_sorted = [midi_to_note(m) for m in chord_midis_sorted]

    # Dissonance curve for the final chord (higher resolution than semitone grid)
    steps = int(curve_steps_per_semitone)
    if steps <= 0:
        raise ValueError("curve_steps_per_semitone must be >= 1")
    midi_step = 1.0 / float(steps)
    curve_midis_arr = np.arange(min_midi, (max_midi + 1) + 1e-9, midi_step, dtype=np.float64)
    curve_freqs_arr = midi_to_frequency_continuous(curve_midis_arr, a4_hz=a4_hz)
    curve_values = dissonance_to_set(
        curve_freqs_arr,
        chord_freqs_sorted,
        overtone_weights,
        peak_semitones_c2=peak_semitones_c2,
        peak_semitones_c6=peak_semitones_c6,
        decay_db_per_oct_c2=decay_db_per_oct_c2,
        decay_db_per_oct_c6=decay_db_per_oct_c6,
        height_c2=height_c2,
        height_c6=height_c6,
        slope_weight=slope_weight,
        slope_h_semitones=slope_h_semitones,
    )
    curve_values = (
        curve_values
        + _boundary_penalty_midis(
            curve_midis_arr,
            root_midi=root_midi,
            extension_midi=ext_midi,
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
