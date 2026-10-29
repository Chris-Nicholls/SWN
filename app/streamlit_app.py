from __future__ import annotations

from pathlib import Path
import time
from typing import Any

import numpy as np
import streamlit as st

import streamlit.components.v1 as components

import json

from chord_model import (
    curve_for_fixed_chord,
    get_scale_mask,
    make_note_list,
    midi_to_frequency,
    midi_to_note_microtonal,
    midi_to_note,
    note_to_midi,
    SCALE_NAMES,
    select_chord_freqs_greedy_harmonic_subharmonic,
)


st.set_page_config(page_title="Chord Dissonance Explorer", layout="wide")

st.title("Chord Dissonance Explorer")


MAX_OVERTONES = 24
SETTINGS_PATH = Path.home() / ".swn_chord_explorer_settings.json"
RESET_DEFAULTS_FLAG = "__reset_to_defaults__"


# If the user requested a full reset, do it before we instantiate widgets.
if bool(st.session_state.get(RESET_DEFAULTS_FLAG, False)):
    try:
        if SETTINGS_PATH.exists():
            SETTINGS_PATH.unlink()
    except Exception:
        pass
    try:
        st.cache_data.clear()
    except Exception:
        pass
    try:
        st.session_state.clear()
    except Exception:
        pass
    st.rerun()


def _load_settings() -> dict[str, Any]:
    try:
        if SETTINGS_PATH.exists():
            return json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
    except Exception:
        pass
    return {}


def _save_settings(data: dict[str, Any]) -> None:
    try:
        SETTINGS_PATH.write_text(json.dumps(data, indent=2, sort_keys=True), encoding="utf-8")
    except Exception:
        # Settings are best-effort; don't break the app.
        return


def _init_session_defaults(settings: dict[str, Any]) -> None:
    defaults: dict[str, Any] = {
        "root_note": "C4",
        "extension_note": "E5",
        "extension_midi_cont": 76.0,
        "n_additional": 4,
        "unquantized_mode": False,
        "freeze_assignment": False,
        "search_n_harmonics": 16,
        "lowpass_cutoff_hz": 20000,
        "lowpass_slope_db_per_oct": 0.0,
        "lowpass_renormalize": False,
        "include_subharmonics": False,
        "candidate_fundamental_only": False,
        "n_overtones": 12,
        "set_aggregation_ui": "Mean",
        "sine_kernel_ui": "Linear (triangle)",
        "peak_semitones_c2": 0.75,
        "peak_semitones_c6": 0.4,
        # Unified tail unit helpers (semitones to drop by half height).
        # These are stored for *both* kernels so switching kernels restores the
        # previous values.
        "tail_half_linear_c2": 3.0,
        "tail_half_linear_c6": 1.5,
        "tail_half_exponential_c2": 2.25,
        "tail_half_exponential_c6": 1.2,
        "height_c2": 1.0,
        "height_c6": 1.0,
        "below_root_penalty_db_per_oct": 1.0,
        "above_extension_penalty_db_per_oct": 1.0,
        "scale_name": "Chromatic",
        "scale_key": "C",
        "scale_penalty": 0.0,

        # Sequencer
        "sequencer_run": False,
        "sequencer_step_s": 0.5,
        "sequencer_steps_text": "C4 E5\nD4 F5\nE4 G5\nF4 A5",
        "sequencer_voice_leading": False,
        "sequencer_prev_weight": 0.5,
    }

    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = settings.get(k, v)

    raw = settings.get("raw_weights")
    sub = settings.get("sub_weights")
    if not isinstance(raw, list):
        raw = [1.0 / float(defaults["n_overtones"]) for _ in range(MAX_OVERTONES)]

    # Default subharmonics to match the legacy behavior: same weights as overtones.
    if not isinstance(sub, list):
        sub = list(raw)
    raw = (raw + [0.0] * MAX_OVERTONES)[:MAX_OVERTONES]
    sub = (sub + [0.0] * MAX_OVERTONES)[:MAX_OVERTONES]
    # There is no 1/1 subharmonic term; keep this at 0 for clarity.
    if sub:
        sub[0] = 0.0
    for i in range(MAX_OVERTONES):
        rk = f"raw_w_{i+1}"
        sk = f"sub_w_{i+1}"
        if rk not in st.session_state:
            st.session_state[rk] = float(raw[i])
        if sk not in st.session_state:
            st.session_state[sk] = float(sub[i])


def _persist_settings() -> None:
    data: dict[str, Any] = {}
    keys = [
        "root_note",
        "extension_note",
        "extension_midi_cont",
        "n_additional",
        "unquantized_mode",
        "freeze_assignment",
        "search_n_harmonics",
        "lowpass_cutoff_hz",
        "lowpass_slope_db_per_oct",
        "lowpass_renormalize",
        "include_subharmonics",
        "candidate_fundamental_only",
        "n_overtones",
        "set_aggregation_ui",
        "sine_kernel_ui",
        "peak_semitones_c2",
        "peak_semitones_c6",
        "tail_half_linear_c2",
        "tail_half_linear_c6",
        "tail_half_exponential_c2",
        "tail_half_exponential_c6",
        "height_c2",
        "height_c6",
        "below_root_penalty_db_per_oct",
        "above_extension_penalty_db_per_oct",
        "scale_name",
        "scale_key",
        "scale_penalty",

        # Sequencer
        "sequencer_run",
        "sequencer_step_s",
        "sequencer_steps_text",
        "sequencer_voice_leading",
        "sequencer_prev_weight",
    ]
    for k in keys:
        if k in st.session_state:
            data[k] = st.session_state[k]

    data["raw_weights"] = [float(st.session_state.get(f"raw_w_{i+1}", 0.0)) for i in range(MAX_OVERTONES)]
    data["sub_weights"] = [float(st.session_state.get(f"sub_w_{i+1}", 0.0)) for i in range(MAX_OVERTONES)]
    _save_settings(data)


_init_session_defaults(_load_settings())


@st.cache_data
def _note_list(min_note: str, max_note: str) -> list[str]:
    return make_note_list(min_note=min_note, max_note=max_note)


def _parse_sequencer_steps(text: str, full_notes: list[str]) -> tuple[list[tuple[str, float]], str | None]:
    """Parse sequencer steps from text.

    Format: one step per line: "ROOT EXT".
    EXT can be a note name (e.g. E5) or a float MIDI value (e.g. 76.25).
    Returns (steps, error_message).
    """

    steps: list[tuple[str, float]] = []
    bad_lines: list[str] = []

    midi_max = float(note_to_midi("C8"))

    for raw in str(text or "").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue

        parts = line.replace(",", " ").split()
        if len(parts) < 2:
            bad_lines.append(raw)
            continue

        root = parts[0]
        if root not in full_notes:
            bad_lines.append(raw)
            continue

        root_midi = float(note_to_midi(root))
        ext_token = parts[1]
        try:
            ext_midi = float(ext_token)
        except Exception:
            try:
                ext_midi = float(note_to_midi(ext_token))
            except Exception:
                bad_lines.append(raw)
                continue

        ext_midi = float(np.clip(ext_midi, root_midi, midi_max))
        steps.append((root, ext_midi))

    if bad_lines and not steps:
        return [], "Invalid sequence lines (expected: ROOT EXT per line)."
    if bad_lines:
        return steps, f"Ignored {len(bad_lines)} invalid line(s)."
    return steps, None


def _init_sequencer_state() -> None:
    # These are normally initialized via _init_session_defaults(). Keep this as a
    # safety net for older settings files.
    if "sequencer_steps_text" not in st.session_state:
        st.session_state["sequencer_steps_text"] = "C4 E5\nD4 F5\nE4 G5\nF4 A5"
    if "sequencer_step_s" not in st.session_state:
        st.session_state["sequencer_step_s"] = 0.5
    if "sequencer_run" not in st.session_state:
        st.session_state["sequencer_run"] = False
    if "sequencer_voice_leading" not in st.session_state:
        st.session_state["sequencer_voice_leading"] = False
    if "sequencer_prev_weight" not in st.session_state:
        st.session_state["sequencer_prev_weight"] = 0.5
    if "sequencer_index" not in st.session_state:
        st.session_state["sequencer_index"] = 0
    if "sequencer_next_ts" not in st.session_state:
        st.session_state["sequencer_next_ts"] = 0.0


def _request_sequencer_step() -> None:
    st.session_state["_sequencer_step_once"] = True
    st.session_state["_sequencer_manual_step_active"] = True


def _advance_sequencer_if_due(full_notes: list[str], *, force: bool = False) -> bool:
    """Advance sequencer by one step if due.

    Must be called before widgets are instantiated so we can safely set
    st.session_state values for widget keys like "root_note".
    """

    if (not force) and (not bool(st.session_state.get("sequencer_run", False))):
        return False

    steps, _ = _parse_sequencer_steps(str(st.session_state.get("sequencer_steps_text", "")), full_notes)
    if not steps:
        return False

    now = time.monotonic()
    if not force:
        next_ts = float(st.session_state.get("sequencer_next_ts", 0.0) or 0.0)
        if now < next_ts:
            return False

    idx = int(st.session_state.get("sequencer_index", 0) or 0)
    root, ext_midi = steps[idx % len(steps)]

    st.session_state["root_note"] = root
    st.session_state["extension_midi_cont"] = float(ext_midi)
    ext_note_name = midi_to_note(int(round(float(ext_midi))))
    st.session_state["extension_note"] = ext_note_name
    st.session_state["extension_note_quantized"] = ext_note_name

    step_s = float(st.session_state.get("sequencer_step_s", 0.5) or 0.5)
    step_s = float(np.clip(step_s, 0.05, 60.0))
    st.session_state["sequencer_index"] = int((idx + 1) % len(steps))
    st.session_state["sequencer_next_ts"] = float(now + step_s)
    return True


# Precompute note list and run sequencer tick BEFORE widgets are created.
full_notes = _note_list("A0", "C8")
_init_sequencer_state()

# Manual step-through: apply the step before widgets are created.
if bool(st.session_state.pop("_sequencer_step_once", False)):
    _advance_sequencer_if_due(full_notes, force=True)

# Reset the sequencer position when toggled on.
prev_run = st.session_state.get("_prev_sequencer_run")
cur_run = bool(st.session_state.get("sequencer_run", False))
if prev_run is None:
    st.session_state["_prev_sequencer_run"] = cur_run
elif bool(prev_run) != cur_run:
    st.session_state["_prev_sequencer_run"] = cur_run
    if cur_run:
        st.session_state["sequencer_index"] = 0
        st.session_state["sequencer_next_ts"] = 0.0
        st.session_state.pop("voice_leading_prev_chord_freqs_hz", None)
        # Advance immediately once on start so the UI reflects step 0 without
        # relying on the fragment to call st.rerun() before widgets mount.
        try:
            _advance_sequencer_if_due(full_notes, force=True)
        except Exception:
            pass


@st.fragment(run_every=0.1)
def _sequencer_fragment() -> None:
    if _advance_sequencer_if_due(full_notes):
        st.rerun()


_sequencer_fragment()


with st.sidebar:
    st.subheader("App")
    if st.button("Reset to defaults"):
        st.session_state[RESET_DEFAULTS_FLAG] = True
        st.rerun()
    st.caption(f"Settings persist to: {SETTINGS_PATH}")
    st.divider()

    st.header("Notes")

    if st.session_state["root_note"] not in full_notes:
        st.session_state["root_note"] = "C4" if "C4" in full_notes else full_notes[0]
    root_note = st.select_slider("Root note", options=full_notes, key="root_note")

    unquantized_mode = st.checkbox(
        "Unquantized mode (continuous pitches)",
        key="unquantized_mode",
        help="When enabled, additional notes are not snapped to semitones (harmonic/subharmonic candidates are evaluated as continuous pitches).",
    )

    prev_unquantized = st.session_state.get("_prev_unquantized_mode")
    mode_toggled = (prev_unquantized is not None) and (bool(prev_unquantized) != bool(unquantized_mode))
    st.session_state["_prev_unquantized_mode"] = bool(unquantized_mode)

    midi_min = note_to_midi("A0")
    midi_max = note_to_midi("C8")
    root_midi = note_to_midi(root_note)

    # Keep extension at/above root to match the range rule.
    root_idx = full_notes.index(root_note)
    ext_options = full_notes[root_idx:]

    # Canonical extension pitch state is always a (possibly fractional) MIDI value.
    # We clamp it for safety any time root changes.
    st.session_state["extension_midi_cont"] = float(
        np.clip(float(st.session_state.get("extension_midi_cont", note_to_midi("E5"))), float(root_midi), float(midi_max))
    )

    if unquantized_mode:
        # Continuous extension pitch in fractional MIDI.
        extension_midi_cont = st.slider(
            "Extension note (continuous)",
            min_value=float(root_midi),
            max_value=float(midi_max),
            step=0.1,
            key="extension_midi_cont",
            help="In unquantized mode the extension pitch is continuous (fractional MIDI).",
        )

        # Keep a nearest-semitone note name around for compatibility/persistence.
        extension_note = midi_to_note(int(round(float(extension_midi_cont))))
        if extension_note not in ext_options:
            extension_note = ext_options[0]
        st.session_state["extension_note"] = extension_note
        st.caption(f"Extension: {midi_to_note_microtonal(float(extension_midi_cont))}")
    else:
        # Quantized (semitone) extension.
        # Important: do NOT overwrite extension_midi_cont unless the user actually changes
        # the discrete extension slider; this preserves the last selected continuous pitch
        # across mode toggles.
        cont_val = float(st.session_state.get("extension_midi_cont", float(note_to_midi("E5"))))
        nearest_note = midi_to_note(int(round(cont_val)))
        if nearest_note not in ext_options:
            nearest_note = "E5" if "E5" in ext_options else ext_options[0]

        # Use a dedicated widget key so we can safely seed it on mode transitions.
        if (
            mode_toggled
            or ("extension_note_quantized" not in st.session_state)
            or (st.session_state.get("extension_note_quantized") not in ext_options)
        ):
            st.session_state["extension_note_quantized"] = nearest_note

        extension_note = st.select_slider(
            "Extension note",
            options=ext_options,
            key="extension_note_quantized",
        )
        st.session_state["extension_note"] = extension_note
        selected_midi = int(note_to_midi(extension_note))
        if selected_midi != int(round(cont_val)):
            st.session_state["extension_midi_cont"] = float(selected_midi)
            cont_val = float(selected_midi)
        extension_midi_cont = float(cont_val)

    st.divider()
    st.subheader("Sequencer")

    st.toggle("Run sequencer", key="sequencer_run")
    st.button("Step", on_click=_request_sequencer_step)
    st.slider(
        "Step (seconds)",
        min_value=0.1,
        max_value=5.0,
        step=0.05,
        key="sequencer_step_s",
    )
    st.toggle(
        "Voice leading",
        key="sequencer_voice_leading",
        help="When enabled, the next chord is seeded from the previous chord (with the new root/extension) and then improved by replacing the most dissonant carried-over voices first.",
    )
    st.slider(
        "Voice leading strength",
        min_value=0.0,
        max_value=1.0,
        step=0.05,
        key="sequencer_prev_weight",
        help="Penalizes moving a carried-over voice (in semitones). 0 disables voice leading behavior.",
    )
    st.text_area(
        "Sequence (one per line: ROOT EXT)",
        key="sequencer_steps_text",
        height=120,
    )
    if st.button("Reset step"):
        st.session_state["sequencer_index"] = 0
        st.session_state["sequencer_next_ts"] = 0.0

    steps, parse_msg = _parse_sequencer_steps(str(st.session_state.get("sequencer_steps_text", "")), full_notes)
    if parse_msg:
        st.caption(parse_msg)
    st.caption(f"Steps: {len(steps)}")

    # Candidate range rule
    ext_midi_cont = float(extension_midi_cont)
    ext_midi_for_range = int(round(ext_midi_cont))

    min_candidate_midi = max(midi_min, root_midi - 24)
    max_candidate_midi = min(midi_max, ext_midi_for_range + 24)
    min_note = midi_to_note(min_candidate_midi)
    max_note = midi_to_note(max_candidate_midi)

    candidates = _note_list(min_note, max_note) if max_candidate_midi > min_candidate_midi else [min_note]
    st.caption(f"Candidate range: {min_note} … {max_note}")

    st.divider()

    n_additional = st.slider("Additional notes", min_value=0, max_value=8, step=1, key="n_additional")

    st.divider()
    st.header("Assignment")

    search_n_harmonics = st.slider(
        "Search harmonics",
        min_value=2,
        max_value=64,
        step=1,
        key="search_n_harmonics",
        help="Controls candidate generation for the (sub)harmonic search (k=2..K). Independent of the number of overtones used in the dissonance calculation.",
    )

    freeze_assignment = st.checkbox("Freeze chord notes", key="freeze_assignment")
    refresh_frozen = st.button("Refresh frozen chord")

    st.divider()
    st.header("Overtones")
    n_overtones = st.slider("Number of overtones", min_value=1, max_value=MAX_OVERTONES, step=1, key="n_overtones")

    include_subharmonics = st.checkbox(
        "Include subharmonics",
        key="include_subharmonics",
        help="If enabled, the dissonance model (and synth timbre) also includes subharmonic partials 1/2..1/N with the same per-index weights.",
    )

    st.checkbox(
        "Evaluate candidates by fundamental only",
        key="candidate_fundamental_only",
        help="When enabled, candidate notes are scored using only their fundamental; existing chord notes still use full harmonics (and optional subharmonics). This affects the search + curve, not playback timbre.",
    )

    lowpass_cutoff_hz = st.slider(
        "Low-pass cutoff (Hz)",
        min_value=50,
        max_value=20000,
        step=50,
        key="lowpass_cutoff_hz",
        help="Attenuates overtones above this frequency.",
    )
    lowpass_slope_db_per_oct = st.slider(
        "Low-pass slope (dB/oct)",
        min_value=0.0,
        max_value=48.0,
        step=1.0,
        key="lowpass_slope_db_per_oct",
        help="Attenuation rate above the cutoff (0 disables).",
    )
    lowpass_renormalize = st.checkbox(
        "Normalize after low-pass",
        key="lowpass_renormalize",
        help="If enabled, re-normalizes the overtone weights after filtering (keeps overall energy similar). If disabled, filtering reduces brightness and total energy.",
    )

    st.divider()
    st.header("Dissonance")

    st.caption(
        "Roughness kernel parameters can vary by register; values are anchored at C2 and C6 and extrapolated smoothly for other notes."
    )

    kernel_opts = ["Linear (triangle)", "Exponential (exp)"]
    if st.session_state["sine_kernel_ui"] not in kernel_opts:
        st.session_state["sine_kernel_ui"] = kernel_opts[0]
    sine_kernel_ui = st.radio(
        "Roughness kernel",
        kernel_opts,
        horizontal=True,
        key="sine_kernel_ui",
        help="Switch between the piecewise-linear triangular kernel and the exponential kernel exp(-a x) - exp(-b x).",
    )
    sine_kernel = "linear" if sine_kernel_ui.startswith("Linear") else "exponential"

    agg_opts = ["Mean", "Max", "RMS"]
    if st.session_state["set_aggregation_ui"] not in agg_opts:
        st.session_state["set_aggregation_ui"] = agg_opts[0]
    set_aggregation_ui = st.radio(
        "Dissonance aggregation",
        agg_opts,
        horizontal=True,
        key="set_aggregation_ui",
        help="How to combine dissonance against a set of existing notes: Mean, Max, or RMS over pairwise dissonances.",
    )
    if set_aggregation_ui == "Mean":
        set_aggregation = "mean"
    elif set_aggregation_ui == "Max":
        set_aggregation = "max"
    else:
        set_aggregation = "rms"

    peak_semitones_c2 = st.slider(
        "Peak position @ C2 (semitones)",
        min_value=0.01,
        max_value=2.0,
        step=0.01,
        key="peak_semitones_c2",
        help="The interval (in semitones) where roughness peaks around the low register (C2).",
    )

    peak_semitones_c6 = st.slider(
        "Peak position @ C6 (semitones)",
        min_value=0.01,
        max_value=2.0,
        step=0.01,
        key="peak_semitones_c6",
        help="The interval (in semitones) where roughness peaks around the high register (C6).",
    )

    # Unified tail unit: semitones to drop by half height.
    # We store values for both kernels so switching kernels restores state.
    DB_PER_OCT_FROM_HALF_SEMITONES = 72.24719895935512  # 20*12*log10(2)

    if sine_kernel == "linear":
        ui_k = "tail_half_linear_c2_ui"
        if ui_k not in st.session_state:
            st.session_state[ui_k] = float(st.session_state["tail_half_linear_c2"])
        v = st.slider(
            "Tail half-drop @ C2 (semitones)",
            min_value=0.05,
            max_value=12.0,
            step=0.05,
            help="Linear kernel: distance (in semitones) after the peak where the roughness has dropped to half its peak height.",
            key=ui_k,
        )
        st.session_state["tail_half_linear_c2"] = float(v)

        ui_k = "tail_half_linear_c6_ui"
        if ui_k not in st.session_state:
            st.session_state[ui_k] = float(st.session_state["tail_half_linear_c6"])
        v = st.slider(
            "Tail half-drop @ C6 (semitones)",
            min_value=0.05,
            max_value=12.0,
            step=0.05,
            help="Linear kernel: distance (in semitones) after the peak where the roughness has dropped to half its peak height.",
            key=ui_k,
        )
        st.session_state["tail_half_linear_c6"] = float(v)
    else:
        ui_k = "tail_half_exponential_c2_ui"
        if ui_k not in st.session_state:
            st.session_state[ui_k] = float(st.session_state["tail_half_exponential_c2"])
        v = st.slider(
            "Tail half-drop @ C2 (semitones)",
            min_value=0.05,
            max_value=12.0,
            step=0.05,
            help="Exponential kernel: interpreted as the tail half-life in semitones (tail drops by 1/2 at this distance).",
            key=ui_k,
        )
        st.session_state["tail_half_exponential_c2"] = float(v)

        ui_k = "tail_half_exponential_c6_ui"
        if ui_k not in st.session_state:
            st.session_state[ui_k] = float(st.session_state["tail_half_exponential_c6"])
        v = st.slider(
            "Tail half-drop @ C6 (semitones)",
            min_value=0.05,
            max_value=12.0,
            step=0.05,
            help="Exponential kernel: interpreted as the tail half-life in semitones (tail drops by 1/2 at this distance).",
            key=ui_k,
        )
        st.session_state["tail_half_exponential_c6"] = float(v)

    # Derive kernel-specific tail params from the shared half-drop unit.
    tail_half_linear_c2 = max(float(st.session_state["tail_half_linear_c2"]), 1e-6)
    tail_half_linear_c6 = max(float(st.session_state["tail_half_linear_c6"]), 1e-6)
    tail_half_exponential_c2 = max(float(st.session_state["tail_half_exponential_c2"]), 1e-6)
    tail_half_exponential_c6 = max(float(st.session_state["tail_half_exponential_c6"]), 1e-6)

    fall_to_zero_semitones_c2 = 2.0 * tail_half_linear_c2
    fall_to_zero_semitones_c6 = 2.0 * tail_half_linear_c6

    decay_db_per_oct_c2 = DB_PER_OCT_FROM_HALF_SEMITONES / tail_half_exponential_c2
    decay_db_per_oct_c6 = DB_PER_OCT_FROM_HALF_SEMITONES / tail_half_exponential_c6

    height_c2 = st.slider(
        "Peak height @ C2",
        min_value=0.0,
        max_value=2.0,
        key="height_c2",
        help="Scales the overall height of the roughness peak around C2.",
    )

    height_c6 = st.slider(
        "Peak height @ C6",
        min_value=0.0,
        max_value=2.0,
        step=0.05,
        key="height_c6",
        help="Scales the overall height of the roughness peak around C6.",
    )

    below_root_penalty_db_per_oct = st.slider(
        "Penalty below root (dB/oct)",
        min_value=0.0,
        max_value=4.0,
        step=0.1,
        key="below_root_penalty_db_per_oct",
        help="Adds an extra penalty for candidate notes below the root, increasing linearly with distance (in octaves). 0 disables.",
    )

    above_extension_penalty_db_per_oct = st.slider(
        "Penalty above extension (dB/oct)",
        min_value=0.0,
        max_value=4.0,
        step=0.1,
        key="above_extension_penalty_db_per_oct",
        help="Adds an extra penalty for candidate notes above the extension, increasing linearly with distance (in octaves). 0 disables.",
    )

    st.subheader("Scale Constraint")
    
    PITCH_CLASSES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]
    
    col_scale, col_key = st.columns(2)
    with col_scale:
        scale_name = st.selectbox(
            "Scale",
            options=SCALE_NAMES,
            index=SCALE_NAMES.index(st.session_state.get("scale_name", "Chromatic")),
            key="scale_name",
            help="Select a scale to constrain chord notes. Notes outside the scale receive a dissonance penalty.",
        )
    
    with col_key:
        scale_key = st.selectbox(
            "Key",
            options=PITCH_CLASSES,
            index=PITCH_CLASSES.index(st.session_state.get("scale_key", "C")),
            key="scale_key",
            help="Root note of the scale (e.g., 'G' for G Major or G Minor).",
        )
    
    scale_penalty = st.slider(
        "Out-of-scale penalty",
        min_value=0.0,
        max_value=2.0,
        step=0.05,
        key="scale_penalty",
        help="Dissonance penalty added to candidates not in the selected scale. 0 = no constraint (chromatic). Higher values make out-of-scale notes less likely.",
    )

def _get_weights(n: int, include_subharmonics: bool) -> tuple[np.ndarray, np.ndarray | None]:
    st.sidebar.caption("Overtone weights are per-overtone amplitudes")

    raw = []
    for i in range(n):
        raw.append(
            st.sidebar.slider(
                f"w[{i+1}]",
                min_value=0.0,
                max_value=1.0,
                step=0.01,
                key=f"raw_w_{i+1}",
            )
        )
    raw = np.array(raw, dtype=float)
    sub = None
    if bool(include_subharmonics):
        st.sidebar.caption("Subharmonic weights are per-subharmonic amplitudes (1/2..1/N)")
        # Build an array of length n; index i corresponds to harmonic (i+1).
        # Only indices 1..n-1 are used for 1/2..1/N.
        sub_vals = [float(st.session_state.get("sub_w_1", 0.0) or 0.0)]
        for i in range(1, n):
            h = i + 1
            sub_vals.append(
                st.sidebar.slider(
                    f"sub w[{h}] (1/{h})",
                    min_value=0.0,
                    max_value=1.0,
                    step=0.01,
                    key=f"sub_w_{h}",
                )
            )
        sub = np.array(sub_vals, dtype=float)

    return raw, sub


weights, sub_weights = _get_weights(n_overtones, include_subharmonics=bool(include_subharmonics))

# Best-effort persistence of sidebar controls + weight vectors.
_persist_settings()


def _render_continuous_synth(
    freqs_hz: list[float],
    overtone_weights: np.ndarray,
    include_subharmonics: bool,
    subharmonic_weights: np.ndarray | None,
    timbre: str,
    volume: float,
    lowpass_cutoff_hz: float,
    lowpass_slope_db_per_oct: float,
    lowpass_renormalize: bool,
    enabled: bool,
    height: int,
) -> None:
    """Continuous additive synth using WebAudio oscillators.

    To avoid an audible gap when Streamlit re-renders this iframe, the audio engine
    is stored on `window.top` and re-used across reloads. Updates retune an existing
    oscillator pool with short ramps instead of recreating oscillators.
    """

    freqs = [float(f) for f in freqs_hz]
    weights_list = [float(x) for x in np.asarray(overtone_weights, dtype=np.float64).tolist()]
    payload = {
        "enabled": bool(enabled),
        "freqs": freqs,
        "weights": weights_list,
        "subharmonics": bool(include_subharmonics),
        "subweights": None if subharmonic_weights is None else [float(x) for x in np.asarray(subharmonic_weights, dtype=np.float64).tolist()],
        "timbre": str(timbre),
        "volume": float(volume),
        "lp_cutoff": float(lowpass_cutoff_hz),
        "lp_slope": float(lowpass_slope_db_per_oct),
        "lp_norm": bool(lowpass_renormalize),
    }
    data = json.dumps(payload)

    html = """
<!doctype html>
<html>
    <head>
        <meta charset="utf-8" />
        <style>
            :root {{ color-scheme: light dark; }}
            body {{ font-family: sans-serif; margin: 0; padding: 0; color: CanvasText; background: Canvas; }}
            .small {{ font-size: 12px; opacity: 0.8; }}
            .mono {{ font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, 'Liberation Mono', 'Courier New', monospace; font-size: 12px; }}
        </style>
    </head>
    <body>
        <div class="small" id="status" style="padding: 6px 0;">starting…</div>
        <div class="small">Continuous oscillators (no loop pause). Changing Streamlit controls retunes audio.</div>
        <div class="mono" id="info" style="padding-top: 4px;"></div>

        <script>
            const cfg = __CFG__;

            // Store the audio engine somewhere that survives re-renders.
            // Prefer window.top, but fall back to window if cross-origin access is blocked.
            let HOST = window;
            try {
                if (window.top && window.top !== window) {
                    // Property access throws on cross-origin; that's our signal to fall back.
                    void window.top.__swn_audio_engine__;
                    HOST = window.top;
                }
            } catch (e) {
                HOST = window;
            }

            const ENGINE_KEY = '__swn_audio_engine__';

            const statusEl = document.getElementById('status');
            const infoEl = document.getElementById('info');
            function setStatus(s) {{ try {{ statusEl.textContent = String(s); }} catch(e) {{}} }}

            function clamp01(x) {{
                const v = Number(x);
                if (!isFinite(v)) return 0.0;
                return Math.max(0.0, Math.min(1.0, v));
            }}

            function createEngine(host) {{
                const engine = {{
                    host: host,
                    ctx: null,
                    master: null,
                    pool: [],
                    pendingCfg: null,

                    canRun: function() {{
                        try {{
                            return !!(this.ctx && this.ctx.state === 'running');
                        }} catch (e) {{
                            return false;
                        }}
                    },

                    start: function() {{
                        // MUST be called from a user gesture (click/tap) to satisfy autoplay policies.
                        this.lastStartError = null;
                        try {{
                            const st = this.ctx ? String(this.ctx.state) : 'none';
                            if (!this.ctx || st === 'closed') {{
                                // If the old context was created in a torn-down iframe, it will be "closed".
                                // Creating the context on the top window lets it survive Streamlit rerenders.
                                const AC = (this.host && (this.host.AudioContext || this.host.webkitAudioContext))
                                    || (window.AudioContext || window.webkitAudioContext);
                                this.ctx = new AC();
                                // Nodes are tied to a context; reset pool/master when replacing the context.
                                this.master = null;
                                this.pool = [];
                            }}
                        }} catch (e) {{
                            this.lastStartError = e;
                            // Fall back to local window context.
                            try {{
                                const AC = (window.AudioContext || window.webkitAudioContext);
                                this.ctx = new AC();
                                this.master = null;
                                this.pool = [];
                            }} catch (e2) {{
                                this.lastStartError = e2;
                            }}
                        }}
                        // Don't await resume(): some environments never resolve it, and we want the
                        // UI to keep polling ctx.state rather than hanging forever on "starting…".
                        try {{
                            if (this.ctx && this.ctx.state !== 'running' && this.ctx.resume) {{
                                const p = this.ctx.resume();
                                if (p && p.catch) {{
                                    p.catch((e) => {{ this.lastStartError = e; }});
                                }}
                            }}
                        }} catch (e) {{
                            this.lastStartError = e;
                        }}

                        if (this.ctx && !this.master) {{
                            this.master = this.ctx.createGain();
                            this.master.gain.value = 0.0;
                            this.master.connect(this.ctx.destination);
                        }}
                    },

                    stop: function() {{
                        if (!this.ctx || !this.master) return;
                        const now = this.ctx.currentTime;
                        try {{ this.master.gain.setTargetAtTime(0.0, now, 0.02); }} catch(e) {{}}
                        try {{
                            for (const ch of this.pool) {{
                                try {{ ch.g.gain.setTargetAtTime(0.0, now, 0.02); }} catch(e) {{}}
                            }}
                        }} catch(e) {{}}
                    },

                    update: function(nextCfg) {{
                        // Called on every rerender; must NOT start AudioContext here.
                        this.pendingCfg = nextCfg;

                        if (!nextCfg || !nextCfg.enabled) {{
                            this.stop();
                            return 'disabled';
                        }}

                        if (!this.canRun()) {{
                            // Autoplay policy: require a user gesture to start.
                            return 'need_gesture';
                        }

                        const freqs = Array.isArray(nextCfg.freqs) ? nextCfg.freqs : [];
                        if (!freqs.length) {{
                            this.stop();
                            return 'no_freqs';
                        }}

                        const timbre = String(nextCfg.timbre || 'Pure sine');
                        const weights = Array.isArray(nextCfg.weights) ? nextCfg.weights : [];
                        const includeSub = Boolean(nextCfg.subharmonics ?? false);
                        const subWeights = Array.isArray(nextCfg.subweights) ? nextCfg.subweights : null;
                        const lpCutoff = Number(nextCfg.lp_cutoff ?? 0);
                        const lpSlope = Number(nextCfg.lp_slope ?? 0);
                        const lpNorm = Boolean(nextCfg.lp_norm ?? false);
                        const volume = clamp01(nextCfg.volume ?? 0.2);

                        const nyquist = this.ctx.sampleRate / 2.0;
                        const perNote = 1.0 / Math.max(1, freqs.length);

                        function lowpassGain(fh) {{
                            const f = Number(fh);
                            if (!isFinite(f) || f <= 0) return 0;
                            if (!(lpSlope > 0) || !(lpCutoff > 0)) return 1.0;
                            if (f <= lpCutoff) return 1.0;
                            const oct = Math.log2(f / lpCutoff);
                            const db = lpSlope * Math.max(0.0, oct);
                            return Math.pow(10.0, -db / 20.0);
                        }}

                        // Build the desired partial list (frequency + gain).
                        const desired = [];
                        for (const f0 of freqs) {{
                            const f = Number(f0);
                            if (!isFinite(f) || f <= 0) continue;

                            if (timbre === 'Pure sine') {{
                                if (f >= nyquist) continue;
                                desired.push([f, perNote]);
                                continue;
                            }}

                            let parts = [];
                            let sumAmp = 0.0;
                            for (let i = 0; i < weights.length; i++) {{
                                const amp0 = Number(weights[i]);
                                if (!isFinite(amp0) || amp0 <= 1e-4) continue;
                                const harmonic = i + 1;
                                const fh = f * harmonic;
                                if (fh >= nyquist) break;
                                const amp = amp0 * lowpassGain(fh);
                                if (!(amp > 1e-10)) continue;
                                parts.push([fh, amp]);
                                sumAmp += amp;

                                if (includeSub && harmonic >= 2) {{
                                    const fs = f / harmonic;
                                    if (isFinite(fs) && fs > 0 && fs < nyquist) {{
                                        const sw0 = (subWeights && subWeights.length > i) ? Number(subWeights[i]) : amp0;
                                        const ampS = sw0 * lowpassGain(fs);
                                        if (ampS > 1e-10) {{
                                            parts.push([fs, ampS]);
                                            sumAmp += ampS;
                                        }}
                                    }}
                                }}
                            }}
                            const inv = (lpNorm && sumAmp > 1e-12) ? (1.0 / sumAmp) : 1.0;
                            for (const p of parts) {{
                                desired.push([p[0], perNote * p[1] * inv]);
                            }}
                        }}

                        const now = this.ctx.currentTime;
                        try {{ this.master.gain.setTargetAtTime(volume, now, 0.02); }} catch(e) {{}}

                        // Grow the pool if needed.
                        while (this.pool.length < desired.length) {{
                            const osc = this.ctx.createOscillator();
                            const g = this.ctx.createGain();
                            osc.type = 'sine';
                            g.gain.value = 0.0;
                            osc.connect(g).connect(this.master);
                            osc.start();
                            this.pool.push({{ osc, g }});
                        }}

                        // Retune/re-gain active channels.
                        for (let i = 0; i < desired.length; i++) {{
                            const ch = this.pool[i];
                            const f = Number(desired[i][0]);
                            const g = Number(desired[i][1]);
                            try {{ ch.osc.frequency.setTargetAtTime(f, now, 0.02); }} catch(e) {{}}
                            try {{ ch.g.gain.setTargetAtTime(g, now, 0.02); }} catch(e) {{}}
                        }}

                        // Silence unused pool channels (keep them running for reuse).
                        for (let i = desired.length; i < this.pool.length; i++) {{
                            const ch = this.pool[i];
                            try {{ ch.g.gain.setTargetAtTime(0.0, now, 0.02); }} catch(e) {{}}
                        }}
                        return 'playing';
                    }},
                }};
                return engine;
            }

            let engine = null;
            try { engine = HOST[ENGINE_KEY]; } catch (e) { engine = null; }
            if (!engine) {
                engine = createEngine(HOST);
                try { HOST[ENGINE_KEY] = engine; } catch (e) { /* ignore */ }
            }

            try {
                const r = engine.update(cfg);
                if (r === 'playing') {
                    setStatus('playing');
                } else if (r === 'disabled') {
                    setStatus('stopped');
                } else if (r === 'no_freqs') {
                    setStatus('no freqs');
                } else {
                    // need_gesture or unknown
                    const st = (engine.ctx && engine.ctx.state) ? String(engine.ctx.state) : 'not-started';
                    setStatus('click inside this panel to start audio (' + st + ')');
                }
            } catch (e) {
                setStatus('error');
            }

            try {{
                infoEl.textContent = 'timbre=' + String(cfg.timbre || '') + '  freqs=' + (Array.isArray(cfg.freqs) ? cfg.freqs.length : 0);
            }} catch(e) {{}}

            // If autoplay is blocked, a click inside the iframe counts as a user gesture.
            document.body.addEventListener('click', async () => {
                try {
                    setStatus('starting…');
                    // Must be called inside the gesture handler.
                    engine.start();

                    let attempts = 0;
                    const maxAttempts = 20; // ~2s @ 100ms
                    const tickMs = 100;
                    const run = () => {
                        attempts += 1;
                        let r = 'unknown';
                        try {
                            r = engine.update(engine.pendingCfg || cfg);
                        } catch (e) {
                            r = 'error';
                        }

                        const st = (engine.ctx && engine.ctx.state) ? String(engine.ctx.state) : 'unknown';
                        if (r === 'playing') {
                            setStatus('playing');
                            return;
                        }

                        if (attempts < maxAttempts) {
                            setStatus('starting… (' + st + ')');
                            setTimeout(run, tickMs);
                            return;
                        }

                        let msg = 'not playing (' + r + ', ' + st + ')';
                        try {
                            if (engine.lastStartError) {
                                const e = engine.lastStartError;
                                msg = msg + ' start_err=' + (e && e.message ? e.message : String(e));
                            }
                        } catch (e) {}
                        setStatus(msg);
                    };

                    // Run on the next tick so any resume() promise can progress.
                    setTimeout(run, 0);
                } catch (e) {
                    try {
                        setStatus('error: ' + (e && e.message ? e.message : String(e)));
                    } catch (e2) {
                        setStatus('error');
                    }
                }
            });
        </script>
    </body>
</html>
"""

    # The template previously lived in an f-string, so it uses doubled braces.
    # Now it's a plain string; normalize back to valid JS/CSS braces.
    html = html.replace("{{", "{").replace("}}", "}")
    html = html.replace("__CFG__", data)

    components.html(html, height=int(height), scrolling=False)


def _select_freqs_hz() -> list[float]:
    def _midi_to_freq_cont(m: float) -> float:
        return float(440.0 * (2.0 ** ((float(m) - 69.0) / 12.0)))

    voice_leading_enabled = bool(st.session_state.get("sequencer_voice_leading", False)) and (
        bool(st.session_state.get("sequencer_run", False))
        or bool(st.session_state.get("_sequencer_manual_step_active", False))
    )
    prev_weight = float(st.session_state.get("sequencer_prev_weight", 0.0) or 0.0)
    prev_freqs = st.session_state.get("voice_leading_prev_chord_freqs_hz") if voice_leading_enabled else None

    root_freq_hz = float(midi_to_frequency(int(note_to_midi(root_note))))
    if bool(unquantized_mode):
        extension_freq_hz = float(_midi_to_freq_cont(float(ext_midi_cont)))
    else:
        extension_freq_hz = float(midi_to_frequency(int(note_to_midi(extension_note))))
    min_freq_hz = float(midi_to_frequency(int(note_to_midi(min_note))))
    max_freq_hz = float(midi_to_frequency(int(note_to_midi(max_note))))

    common = dict(
        root_freq_hz=root_freq_hz,
        extension_freq_hz=extension_freq_hz,
        overtone_weights=weights,
        include_subharmonics=bool(st.session_state.get("include_subharmonics", False)),
        candidate_fundamental_only=bool(st.session_state.get("candidate_fundamental_only", False)),
        subharmonic_weights=sub_weights,
        min_freq_hz=min_freq_hz,
        max_freq_hz=max_freq_hz,
        n_additional=n_additional,
        quantize_to_semitones=not bool(unquantized_mode),
        lowpass_cutoff_hz=lowpass_cutoff_hz,
        lowpass_slope_db_per_oct=lowpass_slope_db_per_oct,
        lowpass_renormalize=lowpass_renormalize,
        set_aggregation=set_aggregation,
        peak_semitones_c2=peak_semitones_c2,
        peak_semitones_c6=peak_semitones_c6,
        fall_to_zero_semitones_c2=fall_to_zero_semitones_c2,
        fall_to_zero_semitones_c6=fall_to_zero_semitones_c6,
        decay_db_per_oct_c2=decay_db_per_oct_c2,
        decay_db_per_oct_c6=decay_db_per_oct_c6,
        height_c2=height_c2,
        height_c6=height_c6,
        sine_kernel=sine_kernel,
        below_root_penalty_db_per_oct=below_root_penalty_db_per_oct,
        above_extension_penalty_db_per_oct=above_extension_penalty_db_per_oct,
        scale_mask=get_scale_mask(scale_name, scale_key),
        scale_penalty=scale_penalty,
        search_n_harmonics=int(search_n_harmonics),
        prev_chord_freqs_hz=prev_freqs,
        prev_chord_weight=prev_weight,
    )
    t0 = time.perf_counter()
    freqs_hz, stats = select_chord_freqs_greedy_harmonic_subharmonic(**common, return_stats=True)
    st.session_state["last_chord_search_s"] = float(time.perf_counter() - t0)
    st.session_state["last_chord_candidates_evaluated"] = int(getattr(stats, "candidates_evaluated", 0) or 0)
    return freqs_hz

# Compute model
try:
    freeze_key = (
        root_note,
        float(ext_midi_cont),
        min_note,
        max_note,
        int(n_additional),
        bool(unquantized_mode),
        int(search_n_harmonics),
        float(lowpass_cutoff_hz),
        float(lowpass_slope_db_per_oct),
        bool(lowpass_renormalize),
        str(set_aggregation),
        bool(st.session_state.get("candidate_fundamental_only", False)),
    )

    if freeze_assignment:
        prev_key = st.session_state.get("frozen_chord_key")
        prev_freqs = st.session_state.get("frozen_chord_freqs_hz")
        need_refresh = refresh_frozen or (prev_key != freeze_key) or (not prev_freqs)

        if need_refresh:
            st.session_state["frozen_chord_key"] = freeze_key
            st.session_state["frozen_chord_freqs_hz"] = _select_freqs_hz()

        fixed_freqs = st.session_state["frozen_chord_freqs_hz"]

        root_freq_hz = float(midi_to_frequency(int(note_to_midi(root_note))))
        extension_freq_hz = float(440.0 * (2.0 ** ((float(ext_midi_cont) - 69.0) / 12.0))) if bool(unquantized_mode) else float(midi_to_frequency(int(note_to_midi(extension_note))))
        min_freq_hz = float(midi_to_frequency(int(note_to_midi(min_note))))
        max_freq_hz = float(midi_to_frequency(int(note_to_midi(max_note))))
        result = curve_for_fixed_chord(
            chord_freqs_hz=fixed_freqs,
            root_freq_hz=root_freq_hz,
            extension_freq_hz=extension_freq_hz,
            min_freq_hz=min_freq_hz,
            max_freq_hz=max_freq_hz,
            overtone_weights=weights,
            include_subharmonics=bool(st.session_state.get("include_subharmonics", False)),
            candidate_fundamental_only=bool(st.session_state.get("candidate_fundamental_only", False)),
            subharmonic_weights=sub_weights,
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
            set_aggregation=set_aggregation,
            below_root_penalty_db_per_oct=below_root_penalty_db_per_oct,
            above_extension_penalty_db_per_oct=above_extension_penalty_db_per_oct,
            curve_steps_per_semitone=5,
        )
    else:
        chord_freqs = _select_freqs_hz()

        root_freq_hz = float(midi_to_frequency(int(note_to_midi(root_note))))
        extension_freq_hz = float(440.0 * (2.0 ** ((float(ext_midi_cont) - 69.0) / 12.0))) if bool(unquantized_mode) else float(midi_to_frequency(int(note_to_midi(extension_note))))
        min_freq_hz = float(midi_to_frequency(int(note_to_midi(min_note))))
        max_freq_hz = float(midi_to_frequency(int(note_to_midi(max_note))))
        result = curve_for_fixed_chord(
            chord_freqs_hz=chord_freqs,
            root_freq_hz=root_freq_hz,
            extension_freq_hz=extension_freq_hz,
            min_freq_hz=min_freq_hz,
            max_freq_hz=max_freq_hz,
            overtone_weights=weights,
            include_subharmonics=bool(st.session_state.get("include_subharmonics", False)),
            candidate_fundamental_only=bool(st.session_state.get("candidate_fundamental_only", False)),
            subharmonic_weights=sub_weights,
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
            set_aggregation=set_aggregation,
            below_root_penalty_db_per_oct=below_root_penalty_db_per_oct,
            above_extension_penalty_db_per_oct=above_extension_penalty_db_per_oct,
            curve_steps_per_semitone=5,
        )
        st.session_state["frozen_chord_key"] = freeze_key
        st.session_state["frozen_chord_freqs_hz"] = result.chord_freqs_sorted_hz
except Exception as exc:
    st.error(f"Failed to compute chord: {exc}")
    st.stop()

# Update voice-leading history for the *next* sequencer step.
if bool(st.session_state.get("sequencer_voice_leading", False)) and (
    bool(st.session_state.get("sequencer_run", False))
    or bool(st.session_state.get("_sequencer_manual_step_active", False))
):
    try:
        st.session_state["voice_leading_prev_chord_freqs_hz"] = list(result.chord_freqs_sorted_hz)
    except Exception:
        pass

# Clear one-shot manual-step flag after compute.
st.session_state.pop("_sequencer_manual_step_active", None)

col_left, col_right = st.columns([2, 1], gap="large")

with col_right:
    st.subheader("Chord")
    st.write(" ".join(result.chord_notes_sorted))

    search_s = float(st.session_state.get("last_chord_search_s", 0.0) or 0.0)
    if search_s > 0:
        st.caption(f"Chord search time: {search_s * 1000.0:.1f} ms")
    else:
        st.caption("Chord search time: —")

    candidates_evaluated = int(st.session_state.get("last_chord_candidates_evaluated", 0) or 0)
    if candidates_evaluated > 0:
        st.caption(f"Candidates evaluated: {candidates_evaluated:,}")
    else:
        st.caption("Candidates evaluated: —")

    st.subheader("Play")
    timbre = st.radio(
        "Timbre",
        ["Pure sine", "Weighted overtones"],
        index=1,
        horizontal=True,
    )
    play_continuous = st.toggle("Enable continuous synth", value=False)
    volume = st.slider("Volume", min_value=0.0, max_value=1.0, value=0.2, step=0.01)

    _render_continuous_synth(
        freqs_hz=list(result.chord_freqs_sorted_hz) if play_continuous else [],
        overtone_weights=weights,
        include_subharmonics=bool(st.session_state.get("include_subharmonics", False)),
        subharmonic_weights=sub_weights,
        timbre=str(timbre),
        volume=float(volume),
        lowpass_cutoff_hz=float(lowpass_cutoff_hz),
        lowpass_slope_db_per_oct=float(lowpass_slope_db_per_oct),
        lowpass_renormalize=bool(lowpass_renormalize),
        enabled=bool(play_continuous),
        height=110 if play_continuous else 0,
    )

    if not play_continuous:
        st.caption("Enable continuous synth to start playback.")

    st.subheader("Overtone weights")
    st.code(np.array2string(weights, precision=4, floatmode="fixed"))

with col_left:
    st.subheader("Dissonance curve")

    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 4))
    x = np.array(result.dissonance_curve_freqs_hz)
    y = np.array(result.dissonance_curve_values)

    ax.plot(x, y, linewidth=1.5)
    ax.set_xscale("log")
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Objective (arb.)")

    # X-axis tickers: every semitone in the chosen note range
    try:
        tick_midis = list(range(note_to_midi(min_note), note_to_midi(max_note) + 1))
        tick_freqs = [midi_to_frequency(m) for m in tick_midis]
        tick_labels = [midi_to_note(m) for m in tick_midis]
        ax.set_xticks(tick_freqs)
        ax.set_xticklabels(tick_labels, rotation=90, fontsize=7)
    except Exception:
        # If range is invalid, fall back to matplotlib defaults
        pass

    # Vertical markers for chord notes
    for f_hz, note in zip(result.chord_freqs_sorted_hz, result.chord_notes_sorted):
        ax.axvline(f_hz, linestyle="--", linewidth=1.0)
        ax.text(f_hz, float(y.min()), note, rotation=90, va="bottom", ha="right", fontsize=8)

    ax.grid(True, which="both", alpha=0.25)
    st.pyplot(fig, clear_figure=True)

    st.caption("Dashed lines mark the selected chord notes.")
