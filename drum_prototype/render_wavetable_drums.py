"""
Wavetable drum-voice prototypes simulating the SWN's ACTUAL native wavetable
engine (o_wt_osc / sphere.h): 512-sample single-cycle tables, phase-accumulator
playback with linear interpolation, and linear crossfade when the morph
position advances to a new table. Two techniques:

1. synth_scan_*: hand-built noise->tone frame sequences (Serum/Vital-style
   "table scanning"), morph position swept fast on note-on, front-loaded
   (short holds on the noisy early frames, longer holds settling into tone),
   with a genuine overall amplitude-decay envelope so it doesn't just sustain
   once it reaches the tone frame.

2. capture_*: real acoustic drum recordings (Samuel Verburg's mic'd "Deluge
   Mic Kit") sliced into consecutive 512-sample frames and played back
   through the same phase-accumulator + table-crossfade engine at a fixed
   oscillator pitch. This deliberately does NOT sound identical to the
   source recording -- looping an arbitrary 512-sample slice at a fixed
   pitch (rather than streaming it once, as sample playback would) is a
   real "captured wavetable" character. For low-frequency-dominant material
   (kick) the frame boundaries are snapped to zero-crossings of the
   low-passed signal, mirroring how a real wavetable editor picks single-
   cycle loop points -- an arbitrary raw slice of a ~60-100Hz waveform
   otherwise loops with an audible click every cycle.
"""
import numpy as np
from scipy.io import wavfile
from scipy.signal import butter, filtfilt

SR = 44100
TABLE_LEN = 512
XFADE_LEN = 32  # matches SWN's WT_XFADE_LEN

# The oscillator's phase_inc = freq*TABLE_LEN/sr is how many raw sample
# positions we advance through the table per output sample. phase_inc=1
# (freq = sr/TABLE_LEN =~ 86.13Hz) means we're replaying the captured
# samples at their own original rate -- genuinely untransposed. ANY other
# freq linearly transposes whatever pitched content is in the table
# (measured: freq=180 shifted a snare's ~277Hz shell tone to ~551Hz, and
# freq=65 shifted a kick's ~79Hz fundamental down to ~56Hz -- both close to
# the freq/NATURAL_FREQ ratio). Noise-dominated voices (hats, shakers,
# claps) don't have a stable pitch for this to audibly distort, but
# anything with a real tonal body (kicks, snares, toms) needs freq at or
# very near this natural rate to keep its own pitch.
NATURAL_FREQ = SR / TABLE_LEN  # ~86.13 Hz


def normalize(x, peak=0.9):
    m = np.max(np.abs(x)) + 1e-12
    return x / m * peak


def write(path, x, sr=SR):
    x = normalize(x)
    wavfile.write(path, sr, (x * 32767).astype(np.int16))
    print("wrote", path)


def read_table_phase_accum(table, freq, n_samples, sr=SR, start_phase=0.0):
    """Faithful to o_wt_osc: phase accumulator over a single 512-sample table,
    linear interpolation between neighboring samples (rh0/rh1/rhd)."""
    table_len = len(table)
    phase_inc = freq * table_len / sr
    phase = start_phase
    out = np.zeros(n_samples)
    for i in range(n_samples):
        idx0 = int(phase) % table_len
        idx1 = (idx0 + 1) % table_len
        frac = phase - int(phase)
        out[i] = table[idx0] * (1 - frac) + table[idx1] * frac
        phase += phase_inc
        if phase >= table_len:
            phase -= table_len
    return out


def read_table_phase_accum_varying(table, freq_array, sr=SR):
    """Same phase accumulator, but freq is a per-sample array -- lets a
    SINGLE static table be swept in pitch by the oscillator's own frequency,
    exactly like a normal synth voice's pitch envelope, instead of trying to
    encode a moving pitch into the captured table content itself (which a
    fixed 512-sample table structurally can't do well for a fast kick
    sweep)."""
    table_len = len(table)
    n_samples = len(freq_array)
    phase = 0.0
    out = np.zeros(n_samples)
    for i in range(n_samples):
        idx0 = int(phase) % table_len
        idx1 = (idx0 + 1) % table_len
        frac = phase - int(phase)
        out[i] = table[idx0] * (1 - frac) + table[idx1] * frac
        phase += freq_array[i] * table_len / sr
        if phase >= table_len:
            phase -= table_len
    return out


def play_wavetable_continuous(tables, freq, n_samples, sr=SR, scan_frac=1.0):
    """Continuously interpolate the morph position across ALL tables over the
    render (never holding on one static table), with the oscillator phase
    accumulating continuously throughout (never reset at a table boundary).
    Reading a fixed table repeatedly at a fixed pitch is inherently periodic
    -- a discrete-line (comb/harmonic) spectrum -- which is audible as a
    buzzy/metallic artifact on tonal or broadband material. Continuously
    sliding the morph position means no single table is ever repeated
    identically for more than an instant, breaking that periodicity.
    `scan_frac` < 1 reaches the final table before n_samples ends, then holds
    there (paired with an amplitude envelope elsewhere to fade the tail)."""
    n_frames = len(tables)
    table_len = len(tables[0])
    phase_inc = freq * table_len / sr
    phase = 0.0
    scan_samples = max(1, int(n_samples * scan_frac))
    out = np.zeros(n_samples)
    for i in range(n_samples):
        m = min(i, scan_samples - 1) / max(1, scan_samples - 1) * (n_frames - 1)
        m0 = int(m)
        m1 = min(m0 + 1, n_frames - 1)
        frac_m = m - m0
        idx0 = int(phase) % table_len
        idx1 = (idx0 + 1) % table_len
        frac_p = phase - int(phase)
        vA = tables[m0][idx0] * (1 - frac_p) + tables[m0][idx1] * frac_p
        vB = tables[m1][idx0] * (1 - frac_p) + tables[m1][idx1] * frac_p
        out[i] = vA * (1 - frac_m) + vB * frac_m
        phase += phase_inc
        if phase >= table_len:
            phase -= table_len
    return out


def play_wavetable_variable(tables, freq, durations_samples, sr=SR, xfade_len=XFADE_LEN):
    """Like play_wavetable_sequence but each table gets its own explicit hold
    time in samples, so early ('noisy') frames can be held briefly and later
    ('settled') frames held longer."""
    segs = [read_table_phase_accum(t, freq, d, sr) for t, d in zip(tables, durations_samples)]
    if len(segs) == 1:
        return segs[0]
    out = [segs[0][:-xfade_len]]
    for i in range(len(segs) - 1):
        a_tail = segs[i][-xfade_len:]
        b_head = segs[i + 1][:xfade_len]
        ramp = np.linspace(0, 1, xfade_len)
        xfade = a_tail * (1 - ramp) + b_head * ramp
        out.append(xfade)
        is_last = i + 1 == len(segs) - 1
        out.append(segs[i + 1][xfade_len:] if is_last else segs[i + 1][xfade_len:-xfade_len])
    return np.concatenate(out)


def play_wavetable_sequence(tables, freq, cycles_per_table, sr=SR, xfade_len=XFADE_LEN):
    durations = [int(round(cycles_per_table * sr / freq))] * len(tables)
    return play_wavetable_variable(tables, freq, durations, sr, xfade_len)


def extend_and_envelope(sig, target_len, tail_table, freq, sr=SR, floor=0.001):
    """Pad by continuing to read the settled/last table, then apply a genuine
    exponential amplitude-decay envelope over the whole voice so it doesn't
    just sustain once the morph has settled."""
    if len(sig) < target_len:
        extra = read_table_phase_accum(tail_table, freq, target_len - len(sig), sr)
        sig = np.concatenate([sig, extra])
    else:
        sig = sig[:target_len]
    t = np.arange(target_len) / sr
    decay_rate = -np.log(floor) / (target_len / sr)
    env = np.exp(-decay_rate * t)
    return sig * env


# ---------------------------------------------------------------------------
# Technique 1: hand-built noise->tone scanning tables
# ---------------------------------------------------------------------------
def make_scan_tables(n_frames, seed, tone_freq_cycles, noise_decay=3.0, nasal=False):
    """Frame 0 = pure broadband noise (single-cycle-length random table),
    later frames fade toward a clean low-partial waveform -- the
    Serum/Vital 'noise-to-tone scan' trick. Plain sine by default (odd
    harmonics read as nasal/hollow -- avoid unless explicitly requested)."""
    rng = np.random.default_rng(seed)
    t = np.linspace(0, 1, TABLE_LEN, endpoint=False)
    tone = np.sin(2 * np.pi * tone_freq_cycles * t)
    if nasal:
        tone += 0.15 * np.sin(2 * np.pi * tone_freq_cycles * 3 * t)
    tables = []
    for i in range(n_frames):
        frac = i / (n_frames - 1)
        noise_amt = np.exp(-noise_decay * frac)
        nz = rng.uniform(-1, 1, TABLE_LEN)
        tab = noise_amt * nz + (1 - noise_amt) * tone
        tables.append(tab)
    return tables


def front_loaded_durations(n_frames, freq, short_cycles, long_cycles, sr=SR):
    """First half of frames held briefly (transient), second half held
    progressively longer as the morph settles into the tone table."""
    cycles = np.linspace(short_cycles, long_cycles, n_frames)
    return [int(round(c * sr / freq)) for c in cycles]


def synth_scan_kick():
    freq = 65
    tables = make_scan_tables(n_frames=16, seed=1, tone_freq_cycles=2, noise_decay=4.5)
    durations = front_loaded_durations(16, freq, short_cycles=0.5, long_cycles=3)
    sig = play_wavetable_variable(tables, freq, durations)
    return extend_and_envelope(sig, int(0.45 * SR), tables[-1], freq)


def synth_scan_snare():
    freq = 200
    tables = make_scan_tables(n_frames=10, seed=2, tone_freq_cycles=3, noise_decay=2.0)
    durations = front_loaded_durations(10, freq, short_cycles=0.5, long_cycles=2)
    sig = play_wavetable_variable(tables, freq, durations)
    return extend_and_envelope(sig, int(0.16 * SR), tables[-1], freq, floor=0.002)


def synth_scan_hat():
    # never resolves fully to tone -- stays noisy, short overall
    freq = 600
    tables = make_scan_tables(n_frames=8, seed=3, tone_freq_cycles=8, noise_decay=1.0)
    durations = front_loaded_durations(8, freq, short_cycles=1, long_cycles=4)
    sig = play_wavetable_variable(tables, freq, durations)
    return extend_and_envelope(sig, int(0.09 * SR), tables[-1], freq, floor=0.003)


# ---------------------------------------------------------------------------
# Technique 2: real acoustic recordings captured into wavetable frames
# ---------------------------------------------------------------------------
MIC_KIT_DIR = "/Users/chrisnicholls/Desktop/deluge2/SAMPLES/Artists/Samuel Verburg/Deluge Mic Kit"


def load_mono(path):
    sr, data = wavfile.read(path)
    if data.ndim > 1:
        data = data.mean(axis=1)
    data = data.astype(np.float64) / 32768.0
    return sr, data


def nearest_rising_zero_crossing(x, idx, window=250):
    lo = max(0, idx - window)
    hi = min(len(x) - 1, idx + window)
    seg = x[lo:hi]
    signs = np.sign(seg)
    crossings = np.where((signs[:-1] <= 0) & (signs[1:] > 0))[0]
    if len(crossings) == 0:
        return idx
    abs_positions = crossings + lo
    return int(abs_positions[np.argmin(np.abs(abs_positions - idx))])


def extract_capture_frames(path, n_frames, trim_seconds=None, table_fade=32,
                            align_to_lowpassed_zero_crossings=False, lp_cutoff=200):
    sr, x = load_mono(path)
    if trim_seconds:
        x = x[: int(trim_seconds * sr)]
    usable = len(x) - TABLE_LEN - table_fade
    if usable <= 0:
        raise ValueError(f"{path} too short for a {TABLE_LEN}-sample table")
    starts = np.linspace(0, usable, n_frames).astype(int)

    if align_to_lowpassed_zero_crossings:
        b, a = butter(4, lp_cutoff / (sr / 2), btype="low")
        x_lp = filtfilt(b, a, x)
        starts = np.array([nearest_rising_zero_crossing(x_lp, s) for s in starts])
        starts = np.clip(starts, 0, usable)

    tables = []
    alpha = np.linspace(0, 1, table_fade)  # 0 at table start, 1 by table_fade samples in
    for s in starts:
        head = x[s : s + TABLE_LEN].copy()
        # the sample(s) that naturally follow the table's own last sample in
        # the source recording -- blending the table's start toward these
        # (rather than toward its own unrelated original head) makes the
        # wrap point (index -1 -> index 0) land on two literally-adjacent
        # samples of the original waveform, i.e. as close to a true loop
        # point as a fixed-length table permits.
        extra_tail = x[s + TABLE_LEN : s + TABLE_LEN + table_fade]
        head[:table_fade] = head[:table_fade] * alpha + extra_tail * (1 - alpha)
        tables.append(head)
    return tables


def generic_kick_capture(path, freq=NATURAL_FREQ, total_duration=0.5, scan_frac=0.85,
                          n_frames=40, lp_cutoff=150, decay_floor=0.001):
    # continuous morph across many, closely-spaced frames -- no table is ever
    # repeated identically, which breaks the fixed-loop harmonic comb that a
    # sustained low-frequency kick otherwise makes very audible. Frame
    # boundaries are snapped to zero-crossings of the low-passed signal
    # (real wavetable-editor practice) since a raw slice of a ~60-100Hz
    # waveform otherwise loops with an audible click every cycle.
    tables = extract_capture_frames(
        path, n_frames=n_frames, table_fade=48,
        align_to_lowpassed_zero_crossings=True, lp_cutoff=lp_cutoff,
    )
    n = int(total_duration * SR)
    sig = play_wavetable_continuous(tables, freq, n, scan_frac=scan_frac)
    t = np.arange(n) / SR
    decay_rate = -np.log(decay_floor) / total_duration
    return sig * np.exp(-decay_rate * t)


def capture_kick(freq=NATURAL_FREQ, total_duration=0.5, scan_frac=0.85):
    return generic_kick_capture(f"{MIC_KIT_DIR}/DelugeMicKit_Kick.wav", freq, total_duration, scan_frac)


def measure_settled_pitch(path, start_frac=0.35, end_frac=0.75, lp_cutoff=300, min_hz=30, max_hz=250):
    """Autocorrelation pitch estimate on a LATER window of the recording,
    after the kick's characteristic fast pitch-sweep has settled -- the
    early attack is exactly where a fast-moving fundamental confuses a
    single-window pitch estimate (and confused the multi-frame morph
    technique the same way)."""
    sr, x = load_mono(path)
    n = len(x)
    seg = x[int(start_frac * n): int(end_frac * n)]
    if len(seg) < 32:
        seg = x
    b, a = butter(4, lp_cutoff / (sr / 2), btype="low")
    xf = filtfilt(b, a, seg)
    xf = xf - xf.mean()
    ac = np.correlate(xf, xf, mode="full")[len(xf) - 1:]
    ac[0] = 0
    min_lag = int(sr / max_hz)
    max_lag = min(int(sr / min_hz), len(ac) - 1)
    if max_lag <= min_lag:
        return NATURAL_FREQ
    peak_lag = min_lag + np.argmax(ac[min_lag:max_lag])
    return sr / peak_lag


def extract_single_table(path, at_frac=0.5, table_fade=48, lp_cutoff=150):
    """One zero-crossing-aligned 512-sample table from a specific point in
    the recording -- used as the kick's captured timbre/waveshape, held
    static and swept in pitch by the oscillator rather than re-captured
    per-frame."""
    sr, x = load_mono(path)
    n = len(x)
    usable = n - TABLE_LEN - table_fade
    target_start = int(np.clip(at_frac * n, 0, usable))
    b, a = butter(4, lp_cutoff / (sr / 2), btype="low")
    x_lp = filtfilt(b, a, x)
    start = nearest_rising_zero_crossing(x_lp, target_start)
    start = int(np.clip(start, 0, usable))
    head = x[start: start + TABLE_LEN].copy()
    extra_tail = x[start + TABLE_LEN: start + TABLE_LEN + table_fade]
    alpha = np.linspace(0, 1, table_fade)
    head[:table_fade] = head[:table_fade] * alpha + extra_tail * (1 - alpha)
    return head


def capture_kick_pitch_envelope(path, total_duration=0.35, sweep_ratio=3.0, sweep_ms=35,
                                 decay_floor=0.001, settled_frac=(0.35, 0.75),
                                 sub_mix=1.3, click_decay_ms=90, sub_hp_cutoff=110,
                                 texture_n_frames=24, texture_span_frac=(0.0, 0.35)):
    """Kick-specific capture technique: a real captured texture layer (the
    click/attack) plus a synthesized pitch-enveloped sine (the sustained
    low thump) -- the same two-layer click+body structure every procedural
    kick in this project uses, just with real captured grit for the click
    instead of a synthesized one.

    A kick's real texture/grit is not a single instant -- it's a complex,
    evolving transient spread across ~50-100ms (verified: Kick_Beef's own
    energy envelope stays strong and structured through ~90ms, not
    concentrated at t=0), so a single static 512-sample table snapshot
    (whichever point it's taken from) inherently can't reproduce it -- it
    can only ever sound like one frozen instant, not the real evolving
    character. Use the multi-frame continuous-morph technique (proven on
    snares/hats/cymbals) for JUST this early texture window instead, and
    let it decay away quickly -- while a clean sub-oscillator (a 512-
    sample table structurally can't hold a full cycle of a 40-70Hz wave
    anyway, see below) carries the actual sustained low end."""
    settled_hz = measure_settled_pitch(path, *settled_frac)

    n = int(total_duration * SR)
    t = np.arange(n) / SR
    sweep_n = int(sweep_ms / 1000 * SR)
    freq = np.full(n, settled_hz)
    if sweep_n > 0:
        frac = np.linspace(0, 1, sweep_n)
        freq[:sweep_n] = settled_hz * (sweep_ratio ** (1 - frac))

    texture_tables = extract_capture_frames(
        path, n_frames=texture_n_frames,
        trim_seconds=None, table_fade=48,
    )
    # only the frames within texture_span_frac of the recording -- the
    # early, evolving attack, not the settled tail
    lo = int(texture_span_frac[0] * len(texture_tables))
    hi = max(lo + 2, int(texture_span_frac[1] * len(texture_tables)))
    texture_tables = texture_tables[lo:hi]
    captured = play_wavetable_continuous(texture_tables, NATURAL_FREQ, n, scan_frac=1.0)
    captured = captured / (np.sqrt(np.mean(captured ** 2)) + 1e-9)
    # The captured layer's own low content isn't phase-locked to the clean
    # sub sine (different internal waveform, not literally the same
    # frequency at every instant) -- summing them lets the two beat/smear
    # against each other in the low band, reading as "muddy" rather than a
    # clean thump. High-pass the captured layer so it only contributes
    # click/texture/character, leaving the sub as the sole carrier of the
    # actual low end.
    b_hp, a_hp = butter(2, sub_hp_cutoff / (SR / 2), btype="high")
    captured = filtfilt(b_hp, a_hp, captured)

    phase = np.cumsum(freq) / SR
    sub = np.sin(2 * np.pi * phase)  # unit amplitude already -- no normalization needed

    # captured and sub are essentially uncorrelated (measured correlation
    # ~0.002), so a (1-x)*A + x*B BLEND actively throws away energy --
    # weighting down two independent signals that sum to 1 gives LESS
    # combined RMS than either alone (0.4*0.997 + correlated cross term ~=
    # 0.59, measured), not more. ADD the sub on top instead of blending it
    # in, which for uncorrelated signals genuinely increases the total
    # low-band energy (measured 1.23 vs 0.997/0.707 individually).
    #
    # Give the click/texture layer and the sub SEPARATE decay envelopes --
    # a real kick's low end outlasts its click, not the other way round.
    # A single shared envelope was cutting the sub off just as fast as the
    # click, which reads as "doesn't sustain" even when the raw energy
    # balance looks fine.
    click_decay_rate = -np.log(decay_floor) / (click_decay_ms / 1000)
    click_env = np.exp(-click_decay_rate * t)
    sub_decay_rate = -np.log(decay_floor) / total_duration
    sub_env = np.exp(-sub_decay_rate * t)

    sig = captured * click_env + sub_mix * sub * sub_env
    return sig


def capture_snare(freq=NATURAL_FREQ, n_frames=12, total_duration=0.24):
    return generic_simple_capture(
        f"{MIC_KIT_DIR}/DelugeMicKit_Snare.wav", freq=freq, n_frames=n_frames, total_duration=total_duration,
    )


def capture_hihat(freq=500, cycles_per_table=6):
    tables = extract_capture_frames(f"{MIC_KIT_DIR}/DelugeMicKit_HiHat.wav", n_frames=6)
    return play_wavetable_sequence(tables, freq=freq, cycles_per_table=cycles_per_table)


def capture_crash(freq=150, total_duration=1.3, scan_frac=0.7):
    # Even with continuous morphing and a gentle playback pitch, broadband
    # noise read through ANY looping oscillator retains some periodicity:
    # consecutive captured frames are close together in time (9ms apart)
    # and therefore highly correlated, so within a single ~7ms oscillator
    # cycle the content barely changes -- a structural mismatch between
    # "noise" (needs fresh randomness every cycle) and "wavetable" (inherently
    # repeating), not something crossfading or morph-speed can fully fix.
    # So: use the captured wavetable ONLY for the transient (played once,
    # never looped -- no repetition, no periodicity issue), and cross-fade
    # into genuinely aperiodic synthesized noise for the sustained decay,
    # where looped-buffer periodicity would otherwise be most audible.
    raise NotImplementedError  # see capture_crash_hybrid below


def measure_spectral_envelope(path, skip_seconds=0.015, nperseg=2048):
    """Long-term average magnitude spectrum of the source recording (after
    the initial transient), used to color the synthesized tail so it matches
    the real cymbal's spectral tilt instead of a guessed filter shape."""
    from scipy.signal import stft
    sr, x = load_mono(path)
    x = x[int(skip_seconds * sr):]
    f, t, Z = stft(x, fs=sr, nperseg=nperseg, noverlap=nperseg // 2)
    avg_mag = np.abs(Z).mean(axis=1)
    return f, avg_mag


def spectral_match_fir(target_freqs, target_mag, n_taps=513, sr=SR):
    """Build a linear-phase FIR whose magnitude response matches target_mag
    (measured on an arbitrary freq grid) via IFFT of the interpolated,
    mirrored magnitude spectrum."""
    n_fft = n_taps * 4
    freqs_full = np.fft.rfftfreq(n_fft, 1 / sr)
    mag_interp = np.interp(freqs_full, target_freqs, target_mag, left=target_mag[0], right=target_mag[-1])
    mag_interp = mag_interp / mag_interp.max()
    impulse = np.fft.irfft(mag_interp, n=n_fft)
    impulse = np.fft.fftshift(impulse)
    center = n_fft // 2
    half = n_taps // 2
    kernel = impulse[center - half: center + half + 1]
    window = np.hanning(n_taps)
    return kernel * window


def biquad_bandpass_signal(x, freq, q, sr=SR):
    w0 = 2 * np.pi * freq / sr
    alpha = np.sin(w0) / (2 * q)
    b0, b2 = alpha, -alpha
    a0 = 1 + alpha
    a1 = -2 * np.cos(w0)
    a2 = 1 - alpha
    from scipy.signal import lfilter
    return lfilter([b0 / a0, 0, b2 / a0], [1, a1 / a0, a2 / a0], x)


# A single time-averaged spectral shape + one global decay rate can't work:
# averaging over the WHOLE decay drags high-frequency bands down more than
# low ones (highs fade fastest, so they spend proportionally more of the
# recording being quiet), which systematically understates how loud the
# highs actually are right at the start. Fit level(t=0) and decay rate
# PER BAND directly from the target recording instead (measured via linear
# regression on each band's dB-vs-time curve, 50-500ms post-transient).
CRASH_BAND_FITS = [  # (lo_hz, hi_hz, level_db_at_t0, decay_db_per_sec)
    (0, 250, -36.3, -24.4),
    (250, 500, -39.3, -11.2),
    (500, 1000, -42.8, -10.6),
    (1000, 2000, -44.8, -8.3),
    (2000, 4000, -42.3, -20.7),
    (4000, 8000, -41.4, -27.0),
    (8000, 12000, -42.3, -33.9),
    (12000, 16000, -46.4, -38.9),
    (16000, 20000, -52.5, -31.5),
]

# Real, persistent resonant modes measured directly in the recording's tail
# (narrow spectral peaks that stick up over the smoothed local envelope,
# still prominent 300ms+ in -- genuine ringing, not transient content). A
# physically struck cymbal is modal at its core, so beyond the broadband
# per-band levels above we layer a few narrow ringing resonances on top,
# each riding on its local band's own level/decay curve plus its measured
# prominence.
CRASH_MODES = [  # (freq_hz, prominence_db_over_local_envelope)
    (193.8, 14.9),
    (398.4, 7.0),
    (882.9, 4.4),
    (2228.7, 4.3),
    (4091.3, 4.6),
    (5361.8, 5.1),
]


def _measure_band_db(x, band, sr=SR, nperseg=2048):
    """Same STFT mean-magnitude-per-band measurement used to derive
    CRASH_BAND_FITS/CRASH_MODES, applied to our OWN synthesized signal so we
    calibrate against identical units instead of assuming dB-in equals
    dB-out across different band widths."""
    from scipy.signal import stft
    f, t, Z = stft(x, fs=sr, nperseg=nperseg, noverlap=nperseg // 2)
    mask = (f >= band[0]) & (f < band[1])
    band_db = 20 * np.log10(np.abs(Z[mask]).mean(axis=0) + 1e-9)
    return np.median(band_db)


def _band_centers_levels_rates():
    centers = np.array([(lo + hi) / 2 for lo, hi, _, _ in CRASH_BAND_FITS])
    levels = np.array([lvl for _, _, lvl, _ in CRASH_BAND_FITS])
    rates = np.array([r for _, _, _, r in CRASH_BAND_FITS])
    return centers, levels, rates


def capture_crash_hybrid(freq=150, transient_ms=45, xfade_ms=25):
    crash_path = f"{MIC_KIT_DIR}/DelugeMicKit_Crash.wav"
    tables = extract_capture_frames(crash_path, n_frames=16, trim_seconds=0.15, table_fade=48)

    # Match the ACTUAL source recording's duration (0.817s), not a
    # theoretical extrapolation of its measured decay rate out to -60dB
    # (1.7s) -- the source itself ends well before that point, so ringing
    # on past it doesn't match "the original" at all, regardless of how
    # well the per-band level/rate numbers fit in isolation.
    _, source_for_duration = load_mono(crash_path)
    total_duration = len(source_for_duration) / SR * 1.1
    n_total = int(total_duration * SR)
    n_transient = int((transient_ms + xfade_ms) / 1000 * SR)
    t = np.arange(n_total) / SR

    transient = play_wavetable_continuous(tables, freq, n_transient, scan_frac=1.0)

    # Target levels/rates were measured via STFT mean-magnitude-per-band,
    # which is NOT the same scale as time-domain RMS -- wider bands read
    # much quieter than narrow ones for the exact same unit-RMS noise
    # (measured: -13.3dB for a 250Hz-wide band vs -25.5dB for a 4000Hz-wide
    # band, purely an analysis artifact). So calibrate each band's actual
    # linear scale factor against its OWN measured baseline through the
    # identical analysis, rather than treating the target dB as a literal
    # linear amplitude.
    from scipy.signal import butter, sosfilt
    multiband = np.zeros(n_total)
    for i, (lo, hi, level_db, rate_db) in enumerate(CRASH_BAND_FITS):
        rng = np.random.default_rng(200 + i)
        band_noise = rng.uniform(-1, 1, n_total)
        nyq = SR / 2
        if lo <= 20:
            sos = butter(4, min(hi, nyq * 0.99) / nyq, btype="low", output="sos")
        else:
            sos = butter(4, [lo / nyq, min(hi, nyq * 0.99) / nyq], btype="band", output="sos")
        banded = sosfilt(sos, band_noise)
        banded = banded / (np.sqrt(np.mean(banded ** 2)) + 1e-9)
        baseline_db = _measure_band_db(banded, (lo, hi))
        correction_db = level_db - baseline_db
        env_db = correction_db + rate_db * t
        multiband += banded * (10 ** (env_db / 20))

    centers, levels, rates = _band_centers_levels_rates()
    modes = np.zeros(n_total)
    for i, (mf, prom_db) in enumerate(CRASH_MODES):
        local_level = np.interp(mf, centers, levels)
        local_rate = np.interp(mf, centers, rates)
        mode_noise = np.random.default_rng(300 + i).uniform(-1, 1, n_total)
        ring = biquad_bandpass_signal(mode_noise, mf, q=25)
        ring = ring / (np.sqrt(np.mean(ring ** 2)) + 1e-9)
        mode_band = (mf * 0.9, mf * 1.1)
        baseline_db = _measure_band_db(ring, mode_band)
        correction_db = (local_level + prom_db) - baseline_db
        env_db = correction_db + local_rate * t
        modes += ring * (10 ** (env_db / 20))

    # see generic_hybrid_capture: band_fits/modes were measured against a
    # peak-normalized copy of the source, so rescale back up by the source's
    # actual peak to match the transient's (un-normalized) absolute scale.
    _, source_x = load_mono(crash_path)
    source_peak = np.max(np.abs(source_x)) + 1e-9
    tail = (multiband + modes) * source_peak

    xfade_n = int(xfade_ms / 1000 * SR)
    out = tail.copy()
    t_start = int(transient_ms / 1000 * SR)
    out[:t_start] = transient[:t_start]
    ramp = np.linspace(0, 1, xfade_n)
    seg = transient[t_start : t_start + xfade_n] * (1 - ramp) + tail[t_start : t_start + xfade_n] * ramp
    out[t_start : t_start + xfade_n] = seg
    return out


### ---------------------------------------------------------------------
### Generalized versions of the two proven capture techniques, so they can
### be pointed at any source recording rather than being hand-written per
### voice.
### ---------------------------------------------------------------------

def generic_simple_capture(path, freq, n_frames, total_duration, trim_seconds=None,
                            scan_frac=1.0, align_zero_crossings=False, lp_cutoff=150,
                            decay_floor=0.001):
    """The technique proven on snare/hihat/kick: continuous morph across
    captured frames (never holding a static table -> no fixed-loop
    periodicity), decaying to silence with a real envelope."""
    tables = extract_capture_frames(
        path, n_frames=n_frames, trim_seconds=trim_seconds, table_fade=32,
        align_to_lowpassed_zero_crossings=align_zero_crossings, lp_cutoff=lp_cutoff,
    )
    n = int(total_duration * SR)
    sig = play_wavetable_continuous(tables, freq, n, scan_frac=scan_frac)
    t = np.arange(n) / SR
    decay_rate = -np.log(decay_floor) / total_duration
    return sig * np.exp(-decay_rate * t)


def fit_band_decays(path, bands, fit_start=0.05, fit_end=None, nperseg=2048):
    """Measure real per-band level(t=0) + decay rate directly from a source
    recording -- the technique that fixed the crash tail's spectral balance."""
    from scipy.signal import stft
    sr, x = load_mono(path)
    duration = len(x) / sr
    if fit_end is None:
        # Was hard-capped at 0.5s regardless of recording length -- fine for
        # the ~0.6-0.8s crash/crash2 clips this was tuned on, but on a much
        # longer recording (crash_shiny, ~3.9s) it meant the rate was fit
        # from only the first 500ms, before the real decay has necessarily
        # kicked in (many cymbals show a near-plateau before the steep part
        # of the decay) -- badly underestimating the true long-term rate
        # (measured -6.8dB/sec that way vs the recording's actual ~-15 to
        # -20dB/sec over its full length). Scale the window with duration.
        fit_end = duration * 0.85
    x = x / (np.max(np.abs(x)) + 1e-9)
    f, t, Z = stft(x, fs=sr, nperseg=nperseg, noverlap=nperseg // 2)
    out = []
    for lo, hi in bands:
        mask = (f >= lo) & (f < hi)
        band_db = 20 * np.log10(np.abs(Z[mask]).mean(axis=0) + 1e-9)
        fit_mask = (t >= fit_start) & (t <= fit_end)
        if fit_mask.sum() < 3:
            fit_mask = t <= fit_end
        A = np.vstack([t[fit_mask], np.ones(fit_mask.sum())]).T
        slope, intercept = np.linalg.lstsq(A, band_db[fit_mask], rcond=None)[0]
        # A near-flat fitted slope (e.g. crash2's low band measured -1.05
        # dB/sec) is almost always mic self-noise/room tone stabilizing,
        # not a real cymbal sustaining near-indefinitely -- letting that
        # into the synthesis adds an audible, unnaturally sustained rumble
        # that doesn't match the source's real decay shape. Floor it to a
        # minimum plausible decay speed.
        slope = min(slope, -8.0)
        out.append((lo, hi, float(intercept), float(slope)))
    return out


def find_resonant_modes(path, tail_start_sec, n_modes=6, nperseg=4096, min_height=3):
    """Detect genuine persistent resonances (narrow spectral peaks that
    stick up over the smoothed local envelope, late in the decay -- real
    ringing, not transient content)."""
    from scipy.signal import stft, find_peaks
    from scipy.ndimage import uniform_filter1d
    sr, x = load_mono(path)
    tail = x[int(tail_start_sec * sr):]
    if len(tail) < nperseg:
        nperseg = max(256, len(tail) // 2)
    f, t, Z = stft(tail, fs=sr, nperseg=nperseg, noverlap=nperseg // 2)
    avg_db = 20 * np.log10(np.abs(Z).mean(axis=1) + 1e-12)
    smooth_db = uniform_filter1d(avg_db, size=41)
    prominence = avg_db - smooth_db
    peaks, props = find_peaks(prominence, height=min_height, distance=10)
    order = np.argsort(props["peak_heights"])[::-1]
    return [(float(f[peaks[i]]), float(props["peak_heights"][i])) for i in order[:n_modes]]


STANDARD_BANDS = [(0, 250), (250, 500), (500, 1000), (1000, 2000), (2000, 4000),
                  (4000, 8000), (8000, 12000), (12000, 16000), (16000, 20000)]


def generic_hybrid_capture(path, freq, band_fits, modes, transient_ms=45, xfade_ms=25,
                            total_duration=None, n_frames=16, trim_seconds=None,
                            mode_prominence_scale=0.3):
    """The full technique proven on crash: one-shot wavetable transient (no
    looping -> no periodicity) cross-faded into calibrated multi-band noise
    (matched level+decay per band, since a single global rate/shape can't
    work) plus a handful of measured resonant modes riding on top."""
    tables = extract_capture_frames(path, n_frames=n_frames, trim_seconds=trim_seconds, table_fade=48)

    if total_duration is None:
        # median rather than min/max -- a single near-flat band (room rumble
        # or mic noise floor rather than real decay, e.g. crash2 measured
        # -1.05 dB/sec in its lowest band) shouldn't be allowed to blow the
        # whole duration estimate up to tens of seconds.
        median_rate = np.median([r for _, _, _, r in band_fits])
        total_duration = 60 / abs(median_rate) if median_rate != 0 else 1.5
        total_duration = min(max(total_duration, 0.5), 2.0)
    n_total = int(total_duration * SR)
    n_transient = int((transient_ms + xfade_ms) / 1000 * SR)
    t = np.arange(n_total) / SR

    transient = play_wavetable_continuous(tables, freq, n_transient, scan_frac=1.0)

    from scipy.signal import butter, sosfilt
    multiband = np.zeros(n_total)
    for i, (lo, hi, level_db, rate_db) in enumerate(band_fits):
        rng = np.random.default_rng(200 + i)
        band_noise = rng.uniform(-1, 1, n_total)
        nyq = SR / 2
        if lo <= 20:
            sos = butter(4, min(hi, nyq * 0.99) / nyq, btype="low", output="sos")
        else:
            sos = butter(4, [lo / nyq, min(hi, nyq * 0.99) / nyq], btype="band", output="sos")
        banded = sosfilt(sos, band_noise)
        banded = banded / (np.sqrt(np.mean(banded ** 2)) + 1e-9)
        baseline_db = _measure_band_db(banded, (lo, hi))
        correction_db = level_db - baseline_db
        env_db = correction_db + rate_db * t
        multiband += banded * (10 ** (env_db / 20))

    centers = np.array([(lo + hi) / 2 for lo, hi, _, _ in band_fits])
    levels = np.array([lvl for _, _, lvl, _ in band_fits])
    rates = np.array([r for _, _, _, r in band_fits])
    # Auto-detected modes (unlike the hand-verified original crash) tend to
    # over-represent resonance -- every recording has SOME narrow peaks
    # sticking up over its smoothed envelope, but not all of them read as
    # perceptually significant "ringing" the way the original crash's
    # manually-confirmed modes did. Damping the prominence keeps the
    # automated pipeline from making every capture sound more metallic/
    # comb-like than its target actually is.
    modes_sig = np.zeros(n_total)
    for i, (mf, prom_db) in enumerate(modes):
        prom_db = prom_db * mode_prominence_scale
        local_level = np.interp(mf, centers, levels)
        local_rate = np.interp(mf, centers, rates)
        mode_noise = np.random.default_rng(300 + i).uniform(-1, 1, n_total)
        ring = biquad_bandpass_signal(mode_noise, mf, q=14)
        ring = ring / (np.sqrt(np.mean(ring ** 2)) + 1e-9)
        mode_band = (mf * 0.9, mf * 1.1)
        baseline_db = _measure_band_db(ring, mode_band)
        correction_db = (local_level + prom_db) - baseline_db
        env_db = correction_db + local_rate * t
        modes_sig += ring * (10 ** (env_db / 20))

    # fit_band_decays/find_resonant_modes measure levels against a
    # PEAK-NORMALIZED (to 1.0) copy of the source, but the transient comes
    # from extract_capture_frames/load_mono, which preserves the recording's
    # actual (un-normalized) absolute amplitude. If the source wasn't
    # already at full scale, the tail ends up calibrated far quieter than
    # the transient (measured: an ~80dB mismatch on a source recorded at
    # roughly -20dBFS) -- rescale the tail back up by the source's own peak
    # so both halves share the same absolute scale.
    _, source_x = load_mono(path)
    source_peak = np.max(np.abs(source_x)) + 1e-9
    tail = (multiband + modes_sig) * source_peak

    xfade_n = int(xfade_ms / 1000 * SR)
    out = tail.copy()
    t_start = int(transient_ms / 1000 * SR)
    out[:t_start] = transient[:t_start]
    ramp = np.linspace(0, 1, xfade_n)
    seg = transient[t_start : t_start + xfade_n] * (1 - ramp) + tail[t_start : t_start + xfade_n] * ramp
    out[t_start : t_start + xfade_n] = seg
    return out


def capture_clap():
    path = f"{MIC_KIT_DIR}/DelugeMicKit_Clap.wav"
    return generic_simple_capture(path, freq=180, n_frames=10, total_duration=0.25)


def capture_rim():
    path = f"{MIC_KIT_DIR}/DelugeMicKit_Rim.wav"
    return generic_simple_capture(path, freq=300, n_frames=6, total_duration=0.1)


def capture_shaker():
    path = f"{MIC_KIT_DIR}/DelugeMicKit_Shaker.wav"
    return generic_simple_capture(path, freq=150, n_frames=14, total_duration=0.2)


def capture_crash2_hybrid():
    path = f"{MIC_KIT_DIR}/DelugeMicKit_Crash2.wav"
    band_fits = fit_band_decays(path, STANDARD_BANDS)
    modes = find_resonant_modes(path, tail_start_sec=0.25, n_modes=4)
    _, src = load_mono(path)
    duration = len(src) / SR * 1.1
    return generic_hybrid_capture(path, freq=150, band_fits=band_fits, modes=modes,
                                   trim_seconds=0.12, total_duration=duration)


ACOUSTIC_DIR = "/Users/chrisnicholls/deluge/SAMPLES/Acoustic"


def capture_snare_real():
    path = f"{ACOUSTIC_DIR}/Snares/OS_SC_Snare_Real.wav"
    return generic_simple_capture(path, freq=NATURAL_FREQ, n_frames=10, total_duration=0.19)


def capture_snare_live():
    path = f"{ACOUSTIC_DIR}/Snares/OS_SC_Snare_Live.wav"
    return generic_simple_capture(path, freq=NATURAL_FREQ, n_frames=10, total_duration=0.16)


def capture_snare_roomy():
    path = f"{ACOUSTIC_DIR}/Snares/OS_SC_Snare_Roomy.wav"
    return generic_simple_capture(path, freq=NATURAL_FREQ, n_frames=10, total_duration=0.19)


def capture_crash_shiny_hybrid():
    path = f"{ACOUSTIC_DIR}/Cymbals/Cymbals/OS_SC_Crash_Shiny.wav"
    band_fits = fit_band_decays(path, STANDARD_BANDS)
    modes = find_resonant_modes(path, tail_start_sec=0.25, n_modes=4)
    _, src = load_mono(path)
    duration = len(src) / SR * 1.05  # match the real ~3.9s ring -- render time is trivial, no need to cap it
    return generic_hybrid_capture(path, freq=150, band_fits=band_fits, modes=modes,
                                   trim_seconds=0.12, total_duration=duration)


def capture_kick_beef():
    path = f"{ACOUSTIC_DIR}/Kicks/OS_SC_Kick_Beef.wav"
    return capture_kick_pitch_envelope(path, total_duration=0.7, click_decay_ms=70)


def capture_kick_live():
    path = f"{ACOUSTIC_DIR}/Kicks/OS_SC_Kick_Live.wav"
    return capture_kick_pitch_envelope(path, total_duration=0.65, click_decay_ms=70)


def capture_kick_low():
    path = f"{ACOUSTIC_DIR}/Kicks/OS_SC_Kick_Low.wav"
    return capture_kick_pitch_envelope(path, total_duration=0.6, click_decay_ms=60)


def capture_kick_tonal():
    path = f"{ACOUSTIC_DIR}/Kicks/OS_SC_Kick_Tonal.wav"
    return capture_kick_pitch_envelope(path, total_duration=0.65, click_decay_ms=70)


VOICES = {
    "wt_scan_kick": synth_scan_kick,
    "wt_scan_snare": synth_scan_snare,
    "wt_scan_hat": synth_scan_hat,
    "wt_capture_kick": capture_kick,
    "wt_capture_snare": capture_snare,
    "wt_capture_hihat": capture_hihat,
    "wt_capture_crash": capture_crash_hybrid,
    "wt_capture_clap": capture_clap,
    "wt_capture_rim": capture_rim,
    "wt_capture_shaker": capture_shaker,
    "wt_capture_crash2": capture_crash2_hybrid,
    "wt_capture_snare_real": capture_snare_real,
    "wt_capture_snare_live": capture_snare_live,
    "wt_capture_snare_roomy": capture_snare_roomy,
    "wt_capture_crash_shiny": capture_crash_shiny_hybrid,
    "wt_capture_kick_beef": capture_kick_beef,
    "wt_capture_kick_live": capture_kick_live,
    "wt_capture_kick_low": capture_kick_low,
    "wt_capture_kick_tonal": capture_kick_tonal,
}

# Source recording for every capture_* voice (used to append "original after
# a 1s pause" so the capture can be A/B'd against its target in one file).
# scan_* voices have no source recording, so they're absent here.
VOICE_SOURCES = {
    "wt_capture_kick": f"{MIC_KIT_DIR}/DelugeMicKit_Kick.wav",
    "wt_capture_snare": f"{MIC_KIT_DIR}/DelugeMicKit_Snare.wav",
    "wt_capture_hihat": f"{MIC_KIT_DIR}/DelugeMicKit_HiHat.wav",
    "wt_capture_crash": f"{MIC_KIT_DIR}/DelugeMicKit_Crash.wav",
    "wt_capture_clap": f"{MIC_KIT_DIR}/DelugeMicKit_Clap.wav",
    "wt_capture_rim": f"{MIC_KIT_DIR}/DelugeMicKit_Rim.wav",
    "wt_capture_shaker": f"{MIC_KIT_DIR}/DelugeMicKit_Shaker.wav",
    "wt_capture_crash2": f"{MIC_KIT_DIR}/DelugeMicKit_Crash2.wav",
    "wt_capture_snare_real": f"{ACOUSTIC_DIR}/Snares/OS_SC_Snare_Real.wav",
    "wt_capture_snare_live": f"{ACOUSTIC_DIR}/Snares/OS_SC_Snare_Live.wav",
    "wt_capture_snare_roomy": f"{ACOUSTIC_DIR}/Snares/OS_SC_Snare_Roomy.wav",
    "wt_capture_crash_shiny": f"{ACOUSTIC_DIR}/Cymbals/Cymbals/OS_SC_Crash_Shiny.wav",
    "wt_capture_kick_beef": f"{ACOUSTIC_DIR}/Kicks/OS_SC_Kick_Beef.wav",
    "wt_capture_kick_live": f"{ACOUSTIC_DIR}/Kicks/OS_SC_Kick_Live.wav",
    "wt_capture_kick_low": f"{ACOUSTIC_DIR}/Kicks/OS_SC_Kick_Low.wav",
    "wt_capture_kick_tonal": f"{ACOUSTIC_DIR}/Kicks/OS_SC_Kick_Tonal.wav",
}


def append_original_after_pause(sig, source_path, pause_seconds=1.0, sr=SR):
    """capture, then a silent gap, then the real source recording -- lets
    both be judged from a single file without hunting down the original."""
    _, source = load_mono(source_path)
    source = source / (np.max(np.abs(source)) + 1e-9) * 0.9
    sig = sig / (np.max(np.abs(sig)) + 1e-9) * 0.9
    pause = np.zeros(int(pause_seconds * sr))
    return np.concatenate([sig, pause, source])


if __name__ == "__main__":
    import os
    outdir = os.path.dirname(os.path.abspath(__file__))
    for name, fn in VOICES.items():
        sig = fn()
        if name in VOICE_SOURCES:
            sig = append_original_after_pause(sig, VOICE_SOURCES[name])
        write(os.path.join(outdir, f"py_{name}.wav"), sig)
