TODOs:
- [x] figure out what to do with CV ins
  - Each channel's CV jack now has a mode, cycled by pressing LFOMODE (or every channel's, in global edit mode): trigger (unchanged), density-mod (feeds that channel's k/density the same way the slider does), or filter-mod (adds to the manual filter knob position). One-shot LED-ring flash on cycling, colour-coded per mode.
- [x] crash sounds should be on the open hi hat too
  - Crash voices (mpump/roller) retagged DRUM_CAT_OPEN_HAT instead of a dedicated crash category, which no longer exists. Channels E and F are both DRUM_CAT_OTHER now (explicit kChannelCategory table, since channel index no longer maps 1:1 onto category).

- [x] How can we add ghost notes?
  - New per-channel ghost-note amount: a probability, rolled once per evaluated step, of an extra quiet hit on a step that wasn't otherwise firing -- works in both engines. Euclid channels also picked up Grids' binary accent (the pattern's downbeat fires loud, everything else normal) so both engines are consistent. (Originally on Latitude push+turn; now on OCT push+turn -- see below.)
  - Also borrowed Grids' chaos for Euclid: moved off FINE (now dedicated to automation, see below) onto a plain turn of the otherwise-dead OCT encoder. (Later split further -- see "chaos should be per channel" below.)

- [ ] Clearer UI for which notes are selected in grids mode. It's quite unclear right now
  - The new X/Y/chaos bar-graph feedback helps locate the controls, but doesn't show which of the 32 steps are actually active -- still open. 
- Bug: hihats not making sound in euclidean mode - possibly due to loading an old preset? 
  - Traced choke-group logic, the preset voice_index/category resolution (category isn't even stored in a preset, it's recomputed live from the fixed per-channel table, so an old preset can't desync it), and the Plaits-hihat state-size init guard -- nothing found that would silence a hi-hat specifically in Euclid mode. Need a more specific repro to go further: does it happen from a fresh boot with no preset loaded, or only after loading a particular saved preset? Which voice is selected on the affected channel?

- There's no visual feedback when changing humanize settings
  - Traced the whole path (sec_OSC_SPREAD read -> apply_humanize_delta -> start_ongoing_display_drum_param(DRUM_PARAM_DISP_HUMANIZE) -> display_drum_param()'s DRUM_PARAM_DISP_HUMANIZE case) and it's wired identically to filter/decay/other, which do show feedback -- no zombie queue consumer, no blocking ongoing_display guard, nothing found. Needs a repro on hardware: is the push+turn gesture (hold TRANSPOSE, then turn) actually landing on the secondary encoder queue at all, or is it possible the turn is only being tried without holding the button down firmly enough?

- [x] Feature: Add randomness to the three parameters (depth/lat/long). Push and turn selects the level or randomisation. Randomisation is an offset from the current param value, set to a new random value every time the channel fires. Bipolar. 
  - Each of Depth/Latitude/Longitude's own push+turn now edits that param's own random amount (0..1); plain turn still edits its level, unchanged. A fresh bipolar offset (-amount..+amount) is rerolled and pushed to the voice right at trigger time, every hit -- the base knob value itself is never touched, so hits keep varying around the same center rather than drifting. Latitude's push+turn used to be ghost -- moved that to OCT push+turn to make room for all three params to get this symmetrically (see above).

- [x] Bug: module crashed on boot again after the chaos/ghost/randomization work above (on both a fresh flash and a plain power cycle, not preset-related)
  - Root cause: a GCC 14.2.1 miscompilation under `-Ofast -flto -fuse-linker-plugin -fwhole-program` together, localized to led_cont.c's display_drum_param() once its inputs (a struct layout + an enum size) shifted -- not a logic bug in our code, confirmed by bisecting on hardware down to "every file at full -Ofast except led_cont.c, which alone dropped to -O0, boots clean" (9 rounds of hardware tests). Ruled out jump-table codegen as the specific mechanism (an equivalent if-else chain still crashed). Fixed by building led_cont.c at -O0 (see the newly-uncommented Makefile line and its comment) -- LED-ring refresh has no real-time budget pressure, so this costs nothing. The exact GCC-internals root cause wasn't identified; this line already existed commented-out in the Makefile, suggesting someone hit adjacent territory before.
