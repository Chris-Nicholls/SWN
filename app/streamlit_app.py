from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np
import streamlit as st

import streamlit.components.v1 as components

import json

from chord_model import (
    average_note_dissonance,
    chord_pairwise_dissonance_matrix,
    curve_for_fixed_chord,
    make_note_list,
    midi_to_frequency,
    midi_to_note,
    note_to_midi,
    select_chord_midis_greedy_harmonic_subharmonic,
)


st.set_page_config(page_title="Chord Dissonance Explorer", layout="wide")

st.title("Chord Dissonance Explorer")


MAX_OVERTONES = 24
SETTINGS_PATH = Path.home() / ".swn_chord_explorer_settings.json"


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
        "extension_weight": 1.0,
        "n_additional": 4,
        "microtonal_mode": False,
        "candidate_steps_per_semitone": 1,
        "freeze_assignment": False,
        "search_n_harmonics": 16,
        "lowpass_cutoff_hz": 20000,
        "lowpass_slope_db_per_oct": 0.0,
        "lowpass_renormalize": False,
        "n_overtones": 12,
        "weight_mode": "Raw weights (auto-normalized)",
        "weight_preset": "All equal",
        "set_aggregation_ui": "Sum",
        "chord_metric_ui": "Sum",
        "sine_kernel_ui": "Linear (triangle)",
        "peak_semitones_c2": 1.16,
        "peak_semitones_c6": 0.2,
        # Unified tail unit helpers (semitones to drop by half height).
        # These are stored for *both* kernels so switching kernels restores the
        # previous values.
        "tail_half_linear_c2": 12.7 / 2.0,
        "tail_half_linear_c6": 12.0 / 2.0,
        "tail_half_exponential_c2": 72.24719895935512 / 25.0,
        "tail_half_exponential_c6": 72.24719895935512 / 110.0,
        "height_c2": 1.0,
        "height_c6": 0.75,
        "below_root_penalty_db_per_oct": 0.0,
        "above_extension_penalty_db_per_oct": 2.0,
    }

    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = settings.get(k, v)

    raw = settings.get("raw_weights")
    logits = settings.get("logits")
    if not isinstance(raw, list):
        raw = [1.0 / float(defaults["n_overtones"]) for _ in range(MAX_OVERTONES)]
    if not isinstance(logits, list):
        logits = [0.0 for _ in range(MAX_OVERTONES)]

    raw = (raw + [0.0] * MAX_OVERTONES)[:MAX_OVERTONES]
    logits = (logits + [0.0] * MAX_OVERTONES)[:MAX_OVERTONES]
    for i in range(MAX_OVERTONES):
        rk = f"raw_w_{i+1}"
        lk = f"logit_{i+1}"
        if rk not in st.session_state:
            st.session_state[rk] = float(raw[i])
        if lk not in st.session_state:
            st.session_state[lk] = float(logits[i])


def _persist_settings() -> None:
    data: dict[str, Any] = {}
    keys = [
        "root_note",
        "extension_note",
        "extension_weight",
        "n_additional",
        "microtonal_mode",
        "freeze_assignment",
        "search_n_harmonics",
        "lowpass_cutoff_hz",
        "lowpass_slope_db_per_oct",
        "lowpass_renormalize",
        "n_overtones",
        "weight_mode",
        "weight_preset",
        "set_aggregation_ui",
        "chord_metric_ui",
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
    ]
    for k in keys:
        if k in st.session_state:
            data[k] = st.session_state[k]

    data["raw_weights"] = [float(st.session_state.get(f"raw_w_{i+1}", 0.0)) for i in range(MAX_OVERTONES)]
    data["logits"] = [float(st.session_state.get(f"logit_{i+1}", 0.0)) for i in range(MAX_OVERTONES)]
    _save_settings(data)


_init_session_defaults(_load_settings())


@st.cache_data
def _note_list(min_note: str, max_note: str) -> list[str]:
    return make_note_list(min_note=min_note, max_note=max_note)


with st.sidebar:
    st.header("Notes")

    full_notes = _note_list("A0", "C8")

    if st.session_state["root_note"] not in full_notes:
        st.session_state["root_note"] = "C4" if "C4" in full_notes else full_notes[0]
    root_note = st.select_slider("Root note", options=full_notes, key="root_note")

    # Keep extension at/above root to match the range rule.
    root_idx = full_notes.index(root_note)
    ext_options = full_notes[root_idx:]
    if st.session_state["extension_note"] not in ext_options:
        st.session_state["extension_note"] = "E5" if "E5" in ext_options else ext_options[0]
    extension_note = st.select_slider("Extension note", options=ext_options, key="extension_note")

    extension_weight = st.slider(
        "Extension weight",
        min_value=0.0,
        max_value=2.0,
        step=0.05,
        key="extension_weight",
        help="Scales the relative importance of the extension note in the dissonance objective (0 = ignore extension, 1 = normal, 2 = twice as important).",
    )

    # Candidate range rule
    midi_min = note_to_midi("A0")
    midi_max = note_to_midi("C8")
    root_midi = note_to_midi(root_note)
    ext_midi = note_to_midi(extension_note)

    min_candidate_midi = max(midi_min, root_midi - 24)
    max_candidate_midi = min(midi_max, ext_midi + 12)
    min_note = midi_to_note(min_candidate_midi)
    max_note = midi_to_note(max_candidate_midi)

    candidates = _note_list(min_note, max_note) if max_candidate_midi > min_candidate_midi else [min_note]
    st.caption(f"Candidate range: {min_note} … {max_note}")

    st.divider()

    n_additional = st.slider("Additional notes", min_value=0, max_value=8, step=1, key="n_additional")

    microtonal_mode = st.checkbox(
        "Microtonal mode (filler notes)",
        key="microtonal_mode",
        help="When enabled, additional notes can be placed between semitones (root/extension stay on the note list).",
    )
    candidate_steps_per_semitone = 4 if microtonal_mode else 1

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

    agg_opts = ["Sum", "Max", "RMS"]
    if st.session_state["set_aggregation_ui"] not in agg_opts:
        st.session_state["set_aggregation_ui"] = agg_opts[0]
    set_aggregation_ui = st.radio(
        "Dissonance aggregation",
        agg_opts,
        horizontal=True,
        key="set_aggregation_ui",
        help="How to combine dissonance against a set of existing notes: Sum, Max, or RMS over pairwise dissonances.",
    )
    if set_aggregation_ui == "Sum":
        set_aggregation = "sum"
    elif set_aggregation_ui == "Max":
        set_aggregation = "max"
    else:
        set_aggregation = "rms"

    chord_metric_opts = ["Sum", "Mean", "RMS"]
    if st.session_state.get("chord_metric_ui") not in chord_metric_opts:
        st.session_state["chord_metric_ui"] = chord_metric_opts[0]
    chord_metric_ui = st.radio(
        "Chord metric",
        chord_metric_opts,
        horizontal=True,
        key="chord_metric_ui",
        help="How to aggregate per-note dissonances into a single chord score. RMS penalizes a few high dissonances more than Sum/Mean.",
    )
    chord_metric = chord_metric_ui.lower()

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

    weight_mode_opts = ["Raw weights (auto-normalized)", "Logits (softmax)", "Preset"]
    if st.session_state["weight_mode"] not in weight_mode_opts:
        st.session_state["weight_mode"] = weight_mode_opts[0]
    mode = st.radio("Slider mode", weight_mode_opts, key="weight_mode")

    preset = None
    if mode == "Preset":
        preset_opts = [
            "All equal",
            "1/n rolloff",
            "Strong fundamental",
            "Odd harmonics",
            "Even harmonics",
        ]
        if st.session_state["weight_preset"] not in preset_opts:
            st.session_state["weight_preset"] = preset_opts[0]
        preset = st.selectbox("Preset", preset_opts, key="weight_preset")




def _get_weights(n: int, mode: str, preset: str | None) -> np.ndarray:
    if mode == "Preset":
        if preset == "All equal":
            w = np.ones(n)
        elif preset == "1/n rolloff":
            idx = np.arange(1, n + 1)
            w = 1.0 / idx
        elif preset == "Strong fundamental":
            w = np.zeros(n)
            w[0] = 1.0
            if n > 1:
                w[1] = 0.25
            if n > 2:
                w[2] = 0.125
        elif preset == "Odd harmonics":
            w = np.array([1.0 if (i % 2 == 0) else 0.0 for i in range(n)], dtype=float)
        elif preset == "Even harmonics":
            w = np.array([0.0 if (i % 2 == 0) else 1.0 for i in range(n)], dtype=float)
        else:
            w = np.ones(n)
        return np.asarray(w, dtype=np.float64)

    st.sidebar.caption("Overtone weights are per-overtone amplitudes")

    if mode == "Logits (softmax)":
        logits = []
        for i in range(n):
            logits.append(
                st.sidebar.slider(
                    f"logit[{i+1}]",
                    min_value=-6.0,
                    max_value=6.0,
                    step=0.1,
                    key=f"logit_{i+1}",
                )
            )
        logits = np.array(logits, dtype=float)
        exp = np.exp(logits - logits.max())
        return exp

    # Raw weights
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
    return raw


weights = _get_weights(n_overtones, mode, preset)

# Best-effort persistence of sidebar controls + weight vectors.
_persist_settings()


def _render_continuous_synth(
        freqs_hz: list[float],
        overtone_weights: np.ndarray,
        timbre: str,
        volume: float,
    lowpass_cutoff_hz: float,
    lowpass_slope_db_per_oct: float,
    lowpass_renormalize: bool,
):
        """Continuous additive synth using WebAudio oscillators (no loop gap)."""
        freqs = [float(f) for f in freqs_hz]
        weights_list = [float(x) for x in np.asarray(overtone_weights, dtype=np.float64).tolist()]
        payload = {
                "freqs": freqs,
                "weights": weights_list,
                "timbre": str(timbre),
                "volume": float(volume),
            "lp_cutoff": float(lowpass_cutoff_hz),
            "lp_slope": float(lowpass_slope_db_per_oct),
            "lp_norm": bool(lowpass_renormalize),
        }
        data = json.dumps(payload)

        # Note: browsers require a user gesture to start audio; the Start button satisfies this.
        html = f"""
<!doctype html>
<html>
    <head>
        <meta charset="utf-8" />
        <style>
            body {{ font-family: sans-serif; margin: 0; padding: 0; }}
            .small {{ font-size: 12px; opacity: 0.8; }}
            .mono {{ font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, 'Liberation Mono', 'Courier New', monospace; font-size: 12px; }}
        </style>
    </head>
    <body>
        <div class="small" id="status" style="padding: 6px 0;">starting…</div>
        <div class="small">Continuous oscillators (no loop pause). Changing Streamlit controls restarts audio.</div>
        <div class="mono" id="info" style="padding-top: 4px;"></div>

        <script>
            const cfg = {data};

            let ctx = null;
            let master = null;
            let oscillators = [];

            function setStatus(s) {{ document.getElementById('status').textContent = s; }}
            function stopAll() {{
                try {{ oscillators.forEach(o => {{ try {{ o.stop(); }} catch(e) {{}} }}); }} catch(e) {{}}
                oscillators = [];
                if (master) {{ try {{ master.disconnect(); }} catch(e) {{}} }}
                master = null;
            }}

            function start() {{
                if (!cfg.freqs || cfg.freqs.length === 0) {{ setStatus('no freqs'); return; }}
                if (!ctx) {{ ctx = new (window.AudioContext || window.webkitAudioContext)(); }}
                stopAll();

                master = ctx.createGain();
                master.gain.value = Math.max(0.0, Math.min(1.0, cfg.volume ?? 0.2));
                master.connect(ctx.destination);

                const now = ctx.currentTime;
                const fade = 0.01;
                master.gain.setValueAtTime(0.0, now);
                master.gain.linearRampToValueAtTime(Math.max(0.0, Math.min(1.0, cfg.volume ?? 0.2)), now + fade);

                const timbre = (cfg.timbre || 'Pure sine');
                const weights = (cfg.weights || []);
                const lpCutoff = Number(cfg.lp_cutoff ?? 0);
                const lpSlope = Number(cfg.lp_slope ?? 0);
                const lpNorm = Boolean(cfg.lp_norm ?? false);
                const nyquist = ctx.sampleRate / 2.0;
                const perNote = 1.0 / cfg.freqs.length;

                function lowpassGain(fh) {{
                    const f = Number(fh);
                    if (!isFinite(f) || f <= 0) return 0;
                    if (!(lpSlope > 0) || !(lpCutoff > 0)) return 1.0;
                    if (f <= lpCutoff) return 1.0;
                    const oct = Math.log2(f / lpCutoff);
                    const db = lpSlope * Math.max(0.0, oct);
                    return Math.pow(10.0, -db / 20.0);
                }}

                let created = 0;
                for (const f0 of cfg.freqs) {{
                    const f = Number(f0);
                    if (!isFinite(f) || f <= 0) continue;

                    if (timbre === 'Pure sine') {{
                        if (f >= nyquist) continue;
                        const osc = ctx.createOscillator();
                        const g = ctx.createGain();
                        osc.type = 'sine';
                        osc.frequency.value = f;
                        g.gain.value = perNote;
                        osc.connect(g).connect(master);
                        osc.start();
                        oscillators.push(osc);
                        created += 1;
                    }} else {{
                        // Apply low-pass per harmonic; optionally renormalize per note.
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
                        }}
                        const inv = (lpNorm && sumAmp > 1e-12) ? (1.0 / sumAmp) : 1.0;
                        for (const p of parts) {{
                            const fh = p[0];
                            const amp = p[1] * inv;
                            const osc = ctx.createOscillator();
                            const g = ctx.createGain();
                            osc.type = 'sine';
                            osc.frequency.value = fh;
                            g.gain.value = perNote * amp;
                            osc.connect(g).connect(master);
                            osc.start();
                            oscillators.push(osc);
                            created += 1;
                        }}
                    }}
                }}

                document.getElementById('info').textContent = 'timbre=' + timbre + '  freqs=' + cfg.freqs.length + '  osc=' + created;
                setStatus('playing');
            }}

            function stop() {{
                if (ctx && master) {{
                    const now = ctx.currentTime;
                    try {{ master.gain.cancelScheduledValues(now); }} catch(e) {{}}
                    try {{ master.gain.setValueAtTime(master.gain.value, now); }} catch(e) {{}}
                    try {{ master.gain.linearRampToValueAtTime(0.0, now + 0.02); }} catch(e) {{}}
                    setTimeout(() => {{ stopAll(); setStatus('stopped'); }}, 30);
                }} else {{
                    stopAll();
                    setStatus('stopped');
                }}
            }}

            // Auto-start. If the browser blocks autoplay, the user interaction that toggled
            // the Streamlit control usually counts as a gesture; otherwise the next click
            // anywhere in the iframe will start it.
            start();
            document.body.addEventListener('click', () => {{ if (!oscillators.length) start(); }});
            window.addEventListener('pagehide', stop);
            document.addEventListener('visibilitychange', () => {{ if (document.hidden) stop(); }});
        </script>
    </body>
</html>
"""

        components.html(html, height=110, scrolling=False)


def _select_midis() -> list[float]:
    def _select_for_steps(steps: int) -> list[float]:
        common = dict(
            root_note=root_note,
            extension_note=extension_note,
            extension_weight=extension_weight,
            overtone_weights=weights,
            min_note=min_note,
            max_note=max_note,
            n_additional=n_additional,
            candidate_steps_per_semitone=int(steps),
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
            search_n_harmonics=int(search_n_harmonics),
        )
        return select_chord_midis_greedy_harmonic_subharmonic(**common)

    def _avg_d_for_midis(midis: list[float]) -> float:
        m = np.asarray(midis, dtype=np.float64)
        freqs = 440.0 * (2.0 ** ((m - 69.0) / 12.0))
        same_root_ext = str(root_note) == str(extension_note)
        ext_freq_hz = None if same_root_ext else midi_to_frequency(note_to_midi(extension_note))
        ext_w = 1.0 if same_root_ext else float(extension_weight)
        return float(
            average_note_dissonance(
                chord_freqs_hz=freqs.tolist(),
                overtone_weights=weights,
                extension_freq_hz=ext_freq_hz,
                extension_weight=ext_w,
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
                chord_aggregation=chord_metric,
            )
        )

    # Guarantee: microtonal mode can't be worse than semitone-grid for the
    # displayed average dissonance metric.
    if microtonal_mode:
        midis_micro = _select_for_steps(candidate_steps_per_semitone)
        midis_semi = _select_for_steps(1)
        if _avg_d_for_midis(midis_semi) <= _avg_d_for_midis(midis_micro):
            return midis_semi
        return midis_micro

    return _select_for_steps(candidate_steps_per_semitone)

# Compute model
try:
    freeze_key = (
        root_note,
        extension_note,
        float(extension_weight),
        min_note,
        max_note,
        int(n_additional),
        bool(microtonal_mode),
        int(search_n_harmonics),
        float(lowpass_cutoff_hz),
        float(lowpass_slope_db_per_oct),
        bool(lowpass_renormalize),
        str(set_aggregation),
        str(chord_metric),
    )

    if freeze_assignment:
        prev_key = st.session_state.get("frozen_chord_key")
        prev_midis = st.session_state.get("frozen_chord_midis")
        need_refresh = refresh_frozen or (prev_key != freeze_key) or (not prev_midis)

        if need_refresh:
            st.session_state["frozen_chord_key"] = freeze_key
            st.session_state["frozen_chord_midis"] = _select_midis()

        fixed_midis = st.session_state["frozen_chord_midis"]
        result = curve_for_fixed_chord(
            chord_midis=fixed_midis,
            root_note=root_note,
            extension_note=extension_note,
            extension_weight=extension_weight,
            min_note=min_note,
            max_note=max_note,
            overtone_weights=weights,
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
        chord_midis = _select_midis()
        result = curve_for_fixed_chord(
            chord_midis=chord_midis,
            root_note=root_note,
            extension_note=extension_note,
            extension_weight=extension_weight,
            min_note=min_note,
            max_note=max_note,
            overtone_weights=weights,
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
        st.session_state["frozen_chord_midis"] = result.chord_midis_sorted
except Exception as exc:
    st.error(f"Failed to compute chord: {exc}")
    st.stop()

col_left, col_right = st.columns([2, 1], gap="large")

with col_right:
    st.subheader("Chord")
    st.write(" ".join(result.chord_notes_sorted))

    same_root_ext = str(root_note) == str(extension_note)
    ext_freq_hz = None if same_root_ext else midi_to_frequency(note_to_midi(extension_note))
    ext_w = 1.0 if same_root_ext else float(extension_weight)

    avg_d = average_note_dissonance(
        chord_freqs_hz=result.chord_freqs_sorted_hz,
        overtone_weights=weights,
        extension_freq_hz=ext_freq_hz,
        extension_weight=ext_w,
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
        chord_aggregation=chord_metric,
    )
    metric_label = {
        "sum": "Total dissonance",
        "mean": "Mean dissonance",
        "rms": "RMS dissonance",
    }.get(str(chord_metric), "Dissonance")
    st.metric(metric_label, f"{avg_d:.4f}")

    st.subheader("Play")
    timbre = st.radio(
        "Timbre",
        ["Pure sine", "Weighted overtones"],
        index=1,
        horizontal=True,
    )
    play_continuous = st.toggle("Enable continuous synth", value=False)
    volume = st.slider("Volume", min_value=0.0, max_value=1.0, value=0.2, step=0.01)

    if play_continuous:
        _render_continuous_synth(
            freqs_hz=list(result.chord_freqs_sorted_hz),
            overtone_weights=weights,
            timbre=str(timbre),
            volume=float(volume),
            lowpass_cutoff_hz=float(lowpass_cutoff_hz),
            lowpass_slope_db_per_oct=float(lowpass_slope_db_per_oct),
            lowpass_renormalize=bool(lowpass_renormalize),
        )
    else:
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

    st.subheader("Dissonance matrix")
    st.caption("Pairwise overtone-weighted roughness between chord notes.")

    ext_freq_hz = None if same_root_ext else midi_to_frequency(note_to_midi(extension_note))
    mat = chord_pairwise_dissonance_matrix(
        chord_freqs_hz=result.chord_freqs_sorted_hz,
        overtone_weights=weights,
        extension_freq_hz=ext_freq_hz,
        extension_weight=ext_w,
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

    fig2, ax2 = plt.subplots(figsize=(10, 6))
    im = ax2.imshow(mat, aspect="equal", origin="lower")

    labels = [f"{n}\n{f:.1f}Hz" for n, f in zip(result.chord_notes_sorted, result.chord_freqs_sorted_hz)]
    ax2.set_xlabel("Chord note")
    ax2.set_ylabel("Chord note")

    ax2.set_xticks(np.arange(mat.shape[1]))
    ax2.set_xticklabels(labels, rotation=45, ha="right", fontsize=8)

    ax2.set_yticks(np.arange(mat.shape[0]))
    ax2.set_yticklabels(labels, rotation=0, fontsize=8)

    cbar = fig2.colorbar(im, ax=ax2)
    cbar.set_label("Weighted dissonance")
    st.pyplot(fig2, clear_figure=True)
