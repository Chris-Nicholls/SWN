from __future__ import annotations

import numpy as np
import streamlit as st

import streamlit.components.v1 as components

import json

from chord_model import (
    average_note_dissonance,
    build_greedy_chord_and_curve,
    curve_for_fixed_chord,
    make_note_list,
    midi_to_frequency,
    midi_to_note,
    normalize_overtone_weights,
    note_to_midi,
    select_chord_midis_greedy,
    select_chord_midis_optimal_search,
    select_chord_midis_root_extension_priority,
)


st.set_page_config(page_title="Chord Dissonance Explorer", layout="wide")

st.title("Chord Dissonance Explorer")


@st.cache_data
def _note_list(min_note: str, max_note: str) -> list[str]:
    return make_note_list(min_note=min_note, max_note=max_note)


with st.sidebar:
    st.header("Notes")

    full_notes = _note_list("A0", "C8")

    root_note = st.select_slider(
        "Root note",
        options=full_notes,
        value="C4" if "C4" in full_notes else full_notes[0],
    )

    # Keep extension at/above root to match the range rule.
    root_idx = full_notes.index(root_note)
    ext_options = full_notes[root_idx:]
    extension_note = st.select_slider(
        "Extension note",
        options=ext_options,
        value="E5" if "E5" in ext_options else ext_options[0],
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

    n_additional = st.slider("Additional notes", min_value=0, max_value=8, value=4, step=1)

    st.divider()
    st.header("Assignment")

    algorithm = st.selectbox(
        "Algorithm",
        [
            "Greedy",
            "Optimal search",
            "Root+extension priority",
        ],
        index=0,
    )
    freeze_assignment = st.checkbox("Freeze chord notes", value=False)
    refresh_frozen = st.button("Refresh frozen chord")

    st.divider()
    st.header("Overtones")
    n_overtones = st.slider("Number of overtones", min_value=1, max_value=24, value=12, step=1)

    st.divider()
    st.header("Dissonance")

    st.caption(
        "Roughness kernel parameters can vary by register; values are anchored at C2 and C6 and extrapolated smoothly for other notes."
    )

    peak_semitones_c2 = st.slider(
        "Peak position @ C2 (semitones)",
        min_value=0.01,
        max_value=2.0,
        value=1.00,
        step=0.01,
        help="The interval (in semitones) where roughness peaks around the low register (C2).",
    )

    peak_semitones_c6 = st.slider(
        "Peak position @ C6 (semitones)",
        min_value=0.01,
        max_value=2.0,
        value=1.00,
        step=0.01,
        help="The interval (in semitones) where roughness peaks around the high register (C6).",
    )

    decay_db_per_oct_c2 = st.slider(
        "Tail decay @ C2 (dB/oct)",
        min_value=1.0,
        max_value=80.0,
        value=20.0,
        step=1.0,
        help="Exponential tail decay for large intervals, expressed as dB of amplitude drop per octave (12 semitones) around C2.",
    )

    decay_db_per_oct_c6 = st.slider(
        "Tail decay @ C6 (dB/oct)",
        min_value=10.0,
        max_value=200.0,
        value=20.0,
        step=1.0,
        help="Exponential tail decay for large intervals, expressed as dB of amplitude drop per octave (12 semitones) around C6.",
    )

    height_c2 = st.slider(
        "Peak height @ C2",
        min_value=0.0,
        max_value=2.0,
        value=1.0,
        step=0.05,
        help="Scales the overall height of the roughness peak around C2.",
    )

    height_c6 = st.slider(
        "Peak height @ C6",
        min_value=0.0,
        max_value=2.0,
        value=1.0,
        step=0.05,
        help="Scales the overall height of the roughness peak around C6.",
    )

    below_root_penalty_db_per_oct = st.slider(
        "Penalty below root (dB/oct)",
        min_value=0.0,
        max_value=4.0,
        value=0.0,
        step=0.1,
        help="Adds an extra penalty for candidate notes below the root, increasing linearly with distance (in octaves). 0 disables.",
    )

    above_extension_penalty_db_per_oct = st.slider(
        "Penalty above extension (dB/oct)",
        min_value=0.0,
        max_value=4.0,
        value=0.0,
        step=0.1,
        help="Adds an extra penalty for candidate notes above the extension, increasing linearly with distance (in octaves). 0 disables.",
    )

    slope_weight = st.slider(
        "Basin preference (slope penalty)",
        min_value=0.0,
        max_value=5.0,
        value=0.15,
        step=0.05,
        help="Adds a penalty for large |dD/d(semitone)| at the chosen note, preferring wider basins of consonance over narrow steep valleys.",
    )

    slope_h_semitones = st.slider(
        "Slope step (semitones)",
        min_value=0.01,
        max_value=0.25,
        value=0.05,
        step=0.01,
        help="Pitch step used to estimate |dD/d(semitone)| via central difference. Smaller = more local; larger = smoother.",
    )

    mode = st.radio(
        "Slider mode",
        ["Raw weights (auto-normalized)", "Logits (softmax)", "Preset"],
        index=0,
    )

    preset = None
    if mode == "Preset":
        preset = st.selectbox(
            "Preset",
            [
                "All equal",
                "1/n rolloff",
                "Strong fundamental",
                "Odd harmonics",
                "Even harmonics",
            ],
        )




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
        return normalize_overtone_weights(w)

    st.sidebar.caption("Weights are always normalized to sum=1")

    if mode == "Logits (softmax)":
        logits = []
        for i in range(n):
            logits.append(st.sidebar.slider(f"logit[{i+1}]", min_value=-6.0, max_value=6.0, value=0.0, step=0.1))
        logits = np.array(logits, dtype=float)
        exp = np.exp(logits - logits.max())
        return exp / exp.sum()

    # Raw weights
    raw = []
    for i in range(n):
        raw.append(st.sidebar.slider(f"w[{i+1}]", min_value=0.0, max_value=1.0, value=(1.0 / n), step=0.01))
    raw = np.array(raw, dtype=float)
    return normalize_overtone_weights(raw)


weights = _get_weights(n_overtones, mode, preset)


def _render_continuous_synth(
        freqs_hz: list[float],
        overtone_weights: np.ndarray,
        timbre: str,
        volume: float,
):
        """Continuous additive synth using WebAudio oscillators (no loop gap)."""
        freqs = [float(f) for f in freqs_hz]
        weights_list = [float(x) for x in np.asarray(overtone_weights, dtype=np.float64).tolist()]
        payload = {
                "freqs": freqs,
                "weights": weights_list,
                "timbre": str(timbre),
                "volume": float(volume),
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
                const nyquist = ctx.sampleRate / 2.0;
                const perNote = 1.0 / cfg.freqs.length;

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
                        for (let i = 0; i < weights.length; i++) {{
                            const amp = Number(weights[i]);
                            if (!isFinite(amp) || amp <= 1e-4) continue;
                            const harmonic = i + 1;
                            const fh = f * harmonic;
                            if (fh >= nyquist) break;

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


def _select_midis() -> list[int]:
    common = dict(
        root_note=root_note,
        extension_note=extension_note,
        overtone_weights=weights,
        min_note=min_note,
        max_note=max_note,
        n_additional=n_additional,
        peak_semitones_c2=peak_semitones_c2,
        peak_semitones_c6=peak_semitones_c6,
        decay_db_per_oct_c2=decay_db_per_oct_c2,
        decay_db_per_oct_c6=decay_db_per_oct_c6,
        height_c2=height_c2,
        height_c6=height_c6,
        below_root_penalty_db_per_oct=below_root_penalty_db_per_oct,
        above_extension_penalty_db_per_oct=above_extension_penalty_db_per_oct,
        slope_weight=slope_weight,
        slope_h_semitones=slope_h_semitones,
    )
    if algorithm == "Optimal search":
        return select_chord_midis_optimal_search(**common)
    if algorithm == "Root+extension priority":
        return select_chord_midis_root_extension_priority(**common)
    return select_chord_midis_greedy(**common)

# Compute model
try:
    freeze_key = (
        root_note,
        extension_note,
        min_note,
        max_note,
        int(n_additional),
        algorithm,
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
            min_note=min_note,
            max_note=max_note,
            overtone_weights=weights,
            peak_semitones_c2=peak_semitones_c2,
            peak_semitones_c6=peak_semitones_c6,
            decay_db_per_oct_c2=decay_db_per_oct_c2,
            decay_db_per_oct_c6=decay_db_per_oct_c6,
            height_c2=height_c2,
            height_c6=height_c6,
            below_root_penalty_db_per_oct=below_root_penalty_db_per_oct,
            above_extension_penalty_db_per_oct=above_extension_penalty_db_per_oct,
            slope_weight=slope_weight,
            slope_h_semitones=slope_h_semitones,
            curve_steps_per_semitone=5,
        )
    else:
        chord_midis = _select_midis()
        result = curve_for_fixed_chord(
            chord_midis=chord_midis,
            root_note=root_note,
            extension_note=extension_note,
            min_note=min_note,
            max_note=max_note,
            overtone_weights=weights,
            peak_semitones_c2=peak_semitones_c2,
            peak_semitones_c6=peak_semitones_c6,
            decay_db_per_oct_c2=decay_db_per_oct_c2,
            decay_db_per_oct_c6=decay_db_per_oct_c6,
            height_c2=height_c2,
            height_c6=height_c6,
            below_root_penalty_db_per_oct=below_root_penalty_db_per_oct,
            above_extension_penalty_db_per_oct=above_extension_penalty_db_per_oct,
            slope_weight=slope_weight,
            slope_h_semitones=slope_h_semitones,
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

    avg_d = average_note_dissonance(
        chord_freqs_hz=result.chord_freqs_sorted_hz,
        overtone_weights=weights,
        peak_semitones_c2=peak_semitones_c2,
        peak_semitones_c6=peak_semitones_c6,
        decay_db_per_oct_c2=decay_db_per_oct_c2,
        decay_db_per_oct_c6=decay_db_per_oct_c6,
        height_c2=height_c2,
        height_c6=height_c6,
    )
    st.metric("Average dissonance", f"{avg_d:.4f}")

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

    st.caption("Dashed lines mark the selected chord notes. Weights are normalized to sum=1.")
