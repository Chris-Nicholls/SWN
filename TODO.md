TODOs:
- [x] figure out what to do with CV ins
  - Each channel's CV jack now has a mode, cycled by pressing LFOMODE (or every channel's, in global edit mode): trigger (unchanged), density-mod (feeds that channel's k/density the same way the slider does), or filter-mod (adds to the manual filter knob position). One-shot LED-ring flash on cycling, colour-coded per mode.
- [x] crash sounds should be on the open hi hat too
  - Crash voices (mpump/roller) retagged DRUM_CAT_OPEN_HAT instead of a dedicated crash category, which no longer exists. Channels E and F are both DRUM_CAT_OTHER now (explicit kChannelCategory table, since channel index no longer maps 1:1 onto category).

- [x] How can we add ghost notes?
  - New per-channel ghost-note amount (Latitude push+turn): a probability, rolled once per evaluated step, of an extra quiet hit on a step that wasn't otherwise firing -- works in both engines. Euclid channels also picked up Grids' binary accent (the pattern's downbeat fires loud, everything else normal) so both engines are consistent.
  - Also borrowed Grids' chaos for Euclid: moved off FINE (now dedicated to automation, see below) onto a plain turn of the otherwise-dead OCT encoder, and it's the same one shared value/control in either pattern engine. In Euclid mode it symmetrically flips a step's fire decision, same idea as Grids' own threshold perturbation.

- [ ] Clearer UI for which notes are selected in grids mode. It's quite unclear right now
  - The new X/Y/chaos bar-graph feedback helps locate the controls, but doesn't show which of the 32 steps are actually active -- still open. 
- Bug: hihats not making sound in euclidean mode - possibly due to loading an old preset? 
  - Traced choke-group logic, the preset voice_index/category resolution (category isn't even stored in a preset, it's recomputed live from the fixed per-channel table, so an old preset can't desync it), and the Plaits-hihat state-size init guard -- nothing found that would silence a hi-hat specifically in Euclid mode. Need a more specific repro to go further: does it happen from a fresh boot with no preset loaded, or only after loading a particular saved preset? Which voice is selected on the affected channel?

- There's no visual feedback when changing humanize settings
  - Traced the whole path (sec_OSC_SPREAD read -> apply_humanize_delta -> start_ongoing_display_drum_param(DRUM_PARAM_DISP_HUMANIZE) -> display_drum_param()'s DRUM_PARAM_DISP_HUMANIZE case) and it's wired identically to filter/decay/other, which do show feedback -- no zombie queue consumer, no blocking ongoing_display guard, nothing found. Needs a repro on hardware: is the push+turn gesture (hold TRANSPOSE, then turn) actually landing on the secondary encoder queue at all, or is it possible the turn is only being tried without holding the button down firmly enough?

- Feature: Add randomness to the three parameters (depth/lat/long). Push and turn selects the level or randomisation. Randomisation is an offset from the current param value, set to a new random value every time the channel fires. Bipolar. 

- in euclidean mode, chaos should be per channel 

- the ghost notes should be more stable between bars, not completely rerolled every bar. Perhaps every 4 bars, or each ghost note has a probability to move each bar? 

- [x] Bug: module crashed on boot after adding automation
  - Root cause: FINE's GPIO reads as briefly "pressed" during power-on (a normal settling glitch), which armed automation RECORD, and the instant it cleared -- still mid-boot, nobody near the panel -- it committed straight to PLAY. From then on every main-loop tick pushed a bogus (all-zero) value into that channel's voice, forever, which crashed. Fixed by requiring FINE to be held at least 200ms before a recording can commit to PLAY; anything shorter (including the boot glitch) just reverts to normal manual control. Bisected and confirmed on hardware.