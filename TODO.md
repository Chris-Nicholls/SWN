TODOs:
Do not guess, but ask for clarification if a task is underspecified at a feature level. This is a living document. 

- [x] figure out what to do with CV ins
  - Each channel's CV jack now has a mode, cycled by pressing LFOMODE (or every channel's, in global edit mode): trigger (unchanged), density-mod (feeds that channel's k/density the same way the slider does), or filter-mod (adds to the manual filter knob position). One-shot LED-ring flash on cycling, colour-coded per mode.
- [x] crash sounds should be on the open hi hat too
  - Crash voices (mpump/roller) retagged DRUM_CAT_OPEN_HAT instead of a dedicated crash category, which no longer exists. Channels E and F are both DRUM_CAT_OTHER now (explicit kChannelCategory table, since channel index no longer maps 1:1 onto category).

- [x] How can we add ghost notes?
  - New per-channel ghost-note amount: a probability, rolled once per evaluated step, of an extra quiet hit on a step that wasn't otherwise firing -- works in both engines. Euclid channels also picked up Grids' binary accent (the pattern's downbeat fires loud, everything else normal) so both engines are consistent. (Originally on Latitude push+turn; now on OCT push+turn -- see below.)
  - Also borrowed Grids' chaos for Euclid: moved off FINE (now dedicated to automation, see below) onto a plain turn of the otherwise-dead OCT encoder. (Later split further -- see "chaos should be per channel" below.)

- [ ] Clearer UI for which notes are selected in grids mode. It's quite unclear right now
  - The new X/Y/chaos bar-graph feedback helps locate the controls, but doesn't show which of the 32 steps are actually active -- still open. 


- [x] There's no visual feedback when changing humanize settings
  - Traced the whole path (sec_OSC_SPREAD read -> apply_humanize_delta -> start_ongoing_display_drum_param(DRUM_PARAM_DISP_HUMANIZE) -> display_drum_param()'s DRUM_PARAM_DISP_HUMANIZE case) and it's wired identically to filter/decay/other, which do show feedback -- no zombie queue consumer, no blocking ongoing_display guard, nothing found. Needs a repro on hardware: is the push+turn gesture (hold TRANSPOSE, then turn) actually landing on the secondary encoder queue at all, or is it possible the turn is only being tried without holding the button down firmly enough?
  -> Yes, and there is UI feedback that it happens, just no indication of how much humanization is applied. Same for ghost notes UI too.
  -> Re-checked against current code (post chaos/ghost/randomization work): DRUM_PARAM_DISP_HUMANIZE and DRUM_PARAM_DISP_GHOST both go through the exact same display_drum_param() bar-graph as filter/decay/other -- `value`/`color` set from d->humanize / d->ghost_amount, rendered as floor(value*18) outer-ring LEDs same as every other param. No special-casing that would show change-happened without magnitude. This note likely predates that bar-graph being wired up for these two; treating as resolved unless it reproduces again on current firmware.

- [x] Feature: Add randomness to the three parameters (depth/lat/long). Push and turn selects the level or randomisation. Randomisation is an offset from the current param value, set to a new random value every time the channel fires. Bipolar. 
  - Each of Depth/Latitude/Longitude's own push+turn now edits that param's own random amount (0..1); plain turn still edits its level, unchanged. A fresh bipolar offset (-amount..+amount) is rerolled and pushed to the voice right at trigger time, every hit -- the base knob value itself is never touched, so hits keep varying around the same center rather than drifting. Latitude's push+turn used to be ghost -- moved that to OCT push+turn to make room for all three params to get this symmetrically (see above).

- [x] Bug: module crashed on boot again after the chaos/ghost/randomization work above (on both a fresh flash and a plain power cycle, not preset-related)
  - Root cause: a GCC 14.2.1 miscompilation under `-Ofast -flto -fuse-linker-plugin -fwhole-program` together, localized to led_cont.c's display_drum_param() once its inputs (a struct layout + an enum size) shifted -- not a logic bug in our code, confirmed by bisecting on hardware down to "every file at full -Ofast except led_cont.c, which alone dropped to -O0, boots clean" (9 rounds of hardware tests). Ruled out jump-table codegen as the specific mechanism (an equivalent if-else chain still crashed). Fixed by building led_cont.c at -O0 (see the newly-uncommented Makefile line and its comment) -- LED-ring refresh has no real-time budget pressure, so this costs nothing. The exact GCC-internals root cause wasn't identified; this line already existed commented-out in the Makefile, suggesting someone hit adjacent territory before.

- [x] the ghost notes should be more stable between bars, not completely rerolled every bar. Perhaps every 4 bars, or each ghost note has a probability to move each bar? 
  - Ghost hits are now a `ghost_pattern` bitmask rerolled once per bar (Euclid, at each resync) or once per lap (Grids, when step wraps to 0) rather than per-step -- see the randomization commit above. Reads as a stable pattern layer of its own rather than flickering every hit.
- [x] Ghost not density should be driven by the channel density. Same for chaos. If there are no active steps, then ther should be no ghost notes or chaos. 
  - New `channel_density_frac()` in drum_ui.c: k/n for Euclid, density/255 for Grids. Both ghost_amount and chaos_amount (Euclid) / pattern_chaos (Grids) are scaled by this before their per-step rolls, so a sparse pattern gets proportionally fewer ghost/chaos hits and a fully silent one (k==0 or density==0) gets exactly none -- not just "very unlikely". For Grids, pattern_chaos's effect on grids_step_active() was already implicitly zero at density==0 (the threshold math can't be crossed), but scaling it explicitly makes the falloff linear rather than only correct at the extreme.

- [x] When settings are changed manually, we should auto-save them after a short delay. At startup, load the last used settings (not necessarily a preset) 
  - `update_drum_autosave()` in drum_preset.c: debounced dirty-tracking (2s, `DRUM_AUTOSAVE_DEBOUNCE_MS`) writes the live kit state to a dedicated flash sector once edits settle; `init_drum_preset()` loads it at boot before anything else, falling back to init_drum_ui()'s hard defaults on first boot (empty sector).

- [x] bug: Resetting a channel that is clock divided seems to also reset it 4 beats later? 
  - This is the "phantom resync" bug already fixed in 8f424f4: reset_all_patterns() reset step position but not a divided channel's bars_until_resync countdown, so the stale countdown fired a second forced resync on top of the manual one some bars later. Fixed by recomputing bars_until_resync the same way the normal per-bar resync does.

- [ ] Bug: Grids mode no longer functions at all
  - Not yet reproduced -- read through the Grids branch of update_drum_triggers(), pattern_chaos/pec_OCT wiring, and grids_step_active() and nothing obviously wrong stood out. The build the user tested this on may have been a mid-bisection variant with features disabled rather than the final committed HEAD. Built a fresh, from-clean-tree wav straight off dfbe67d (main.bin byte-identical to the confirmed-working boot-crash-fix build, 221244 bytes) to retest with -- if Grids is still broken on this build, that rules out "leftover bisection stub" and narrows it back to actual logic in 23476fa.