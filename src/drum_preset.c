/*
 * drum_preset.c - kit save/load slots on the PRESET encoder
 *
 * -----------------------------------------------------------------------------
 */

#include <string.h>

#include "drum_preset.h"
#include "drum_ui.h"
#include "drum_voice.h"
#include "euclid_pattern.h"
#include "led_cont.h"
#include "UI_conditioning.h"
#include "hardware_controls.h"
#include "params_lfo_period.h"
#include "math_util.h"
#include "drivers/flashram_spidma.h"
#include "flash_S25FL127.h"

/* Sector 15 is the last of the SPI flash's sixteen 4kB sectors and is
 * otherwise unused (STARTUP_PRESET_SETTING_SECTOR=14 and
 * WT_SECTOR_START=16 bracket it -- see external_flash_layout.h).
 * Independent of the old preset_manager's PRESET_SECTOR_START region,
 * which serializes an unrelated, pre-drum-station struct. */
#define DRUM_PRESET_SECTOR_ADDR	0x0000F000u
#define DRUM_PRESET_SLOT_SIZE	256u	/* 16 slots x 256B = one 4kB sector, exactly */
#define DRUM_PRESET_MAGIC		0x444B3031u	/* "DK01" */

/* Autosave lives in sector 16 (WT_SECTOR_START), otherwise unused now
 * that wavetable capture/editing is gone. Unlike the 16 rotating slots
 * above, nothing else shares this sector, so it's erased and rewritten
 * whole rather than read-modify-written. */
#define DRUM_AUTOSAVE_SECTOR_ADDR	0x00010000u
#define DRUM_AUTOSAVE_DEBOUNCE_MS	2000u	/* "a short delay" after the last manual edit */

typedef struct __attribute__((packed)) {
	uint8_t	voice_index;	/* 0xFF = silent channel (ops == NULL) */
	uint8_t	euclid_n;
	uint8_t	euclid_k;
	uint8_t	euclid_rotation;
	float	level;
	float	pitch;
	float	filter;
	float	decay;
	float	other;
	float	clock_divmult_id;
	uint8_t	density;		/* Grids-mode pattern density, unused by euclidean channels */
} DrumChanPreset;

typedef struct __attribute__((packed)) {
	uint32_t		magic;
	DrumChanPreset	chan[NUM_CHANNELS];

	/* Kit-wide pattern engine and, for Grids, its shared map position
	 * and chaos amount. */
	uint8_t			pattern_engine;
	uint8_t			grids_x;
	uint8_t			grids_y;
	uint8_t			grids_chaos;
	float			grids_clock_divmult_id;
} DrumKitPreset;

static uint8_t			selected_slot = 0;
static uint8_t			slot_filled[DRUM_PRESET_NUM_SLOTS];
static enum PressTypes	prev_press_level = RELEASED;

static uint32_t slot_addr(uint8_t slot)
{
	return DRUM_PRESET_SECTOR_ADDR + (uint32_t)slot * DRUM_PRESET_SLOT_SIZE;
}

/* Shared by slot save/load and autosave below -- everything here is
 * "the whole kit" independent of *where* it ends up in flash. */
static void kit_from_live_state(DrumKitPreset *kit)
{
	memset(kit, 0, sizeof(*kit));
	kit->magic = DRUM_PRESET_MAGIC;

	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		const o_drum_chan *d = &drum_chan[c];
		DrumChanPreset *p = &kit->chan[c];
		int8_t vi = drum_voice_registry_index(d->ops);

		p->voice_index      = (vi < 0) ? 0xFF : (uint8_t)vi;
		p->euclid_n         = (uint8_t)d->euclid.n;
		p->euclid_k         = (uint8_t)d->euclid.k;
		p->euclid_rotation  = (uint8_t)d->euclid.rotation;
		p->level            = d->level;
		p->pitch            = d->pitch;
		p->filter           = d->filter;
		p->decay            = d->decay;
		p->other            = d->other;
		p->clock_divmult_id = d->clock_divmult_id;
		p->density          = d->density;
	}

	kit->pattern_engine = (uint8_t)drum_pattern_engine;
	kit->grids_x        = grids_x;
	kit->grids_y        = grids_y;
	kit->grids_chaos    = grids_chaos;
	kit->grids_clock_divmult_id = grids_clock_divmult_id;
}

/* Rebinds each channel's voice/pattern/params from a loaded kit. Runtime
 * DSP state (envelope phase, oscillator phase, ...) is never
 * serialized -- ops->init() plus a fresh push of filter/decay/other
 * gives every loaded voice a clean start rather than resuming
 * mid-envelope. */
static void kit_to_live_state(const DrumKitPreset *kit)
{
	for (uint8_t c = 0; c < NUM_CHANNELS; c++) {
		o_drum_chan *d = &drum_chan[c];
		const DrumChanPreset *p = &kit->chan[c];

		d->ops = (p->voice_index == 0xFF) ? NULL : drum_voice_registry_lookup(p->voice_index);

		/* n/k/rotation/pattern are read from OSC_TIM -- hold it off
		 * while rewriting them as a group, same idiom as drum_ui.c's
		 * own pattern-encoder handling. */
		__disable_irq();
		euclid_set_n(&d->euclid, p->euclid_n);
		euclid_set_k(&d->euclid, p->euclid_k);
		euclid_set_rotation(&d->euclid, p->euclid_rotation);
		__enable_irq();

		d->level            = p->level;
		d->pitch            = p->pitch;
		d->filter           = p->filter;
		d->decay            = p->decay;
		d->other            = p->other;
		d->clock_divmult_id = p->clock_divmult_id;
		d->clock_rate       = calc_divmult_amount(d->clock_divmult_id);
		d->step_phase       = 0.0f;
		d->density          = p->density;

		if (d->ops) {
			d->ops->init(d->state);
			d->ops->set_filter(d->state, d->filter);
			d->ops->set_decay(d->state, d->decay);
			d->ops->set_other(d->state, d->other);
		}
	}

	drum_pattern_engine = (kit->pattern_engine == PATTERN_ENGINE_GRIDS)
	                    ? PATTERN_ENGINE_GRIDS : PATTERN_ENGINE_EUCLID;
	grids_x     = kit->grids_x;
	grids_y     = kit->grids_y;
	grids_chaos = kit->grids_chaos;
	grids_clock_divmult_id = kit->grids_clock_divmult_id;
	grids_clock_rate       = calc_divmult_amount(grids_clock_divmult_id);

	/* Without this, the very next read_channel_sliders() call would
	 * instantly overwrite every channel's just-loaded k/density with
	 * whatever its physical slider happens to be sitting at -- see the
	 * comment on drum_ui_request_slider_pickup() for the full story. */
	drum_ui_request_slider_pickup();
}

static void save_slot(uint8_t slot)
{
	/* A single slot write shares a sector with 15 others, so saving
	 * has to read-modify-write the whole sector rather than erasing
	 * just this slot -- the flash chip only erases at sector
	 * granularity. */
	static uint8_t sector_buf[sFLASH_SPI_4K_SECTOR_SIZE];
	DrumKitPreset kit;

	kit_from_live_state(&kit);

	sFLASH_read_buffer(sector_buf, DRUM_PRESET_SECTOR_ADDR, sizeof(sector_buf));
	memcpy(sector_buf + (uint32_t)slot * DRUM_PRESET_SLOT_SIZE, &kit, sizeof(kit));
	sFLASH_erase_sector(DRUM_PRESET_SECTOR_ADDR);
	sFLASH_write_buffer(sector_buf, DRUM_PRESET_SECTOR_ADDR, sizeof(sector_buf));

	slot_filled[slot] = 1;
}

static uint8_t load_slot(uint8_t slot)
{
	DrumKitPreset kit;

	sFLASH_read_buffer((uint8_t *)&kit, slot_addr(slot), sizeof(kit));
	if (kit.magic != DRUM_PRESET_MAGIC)
		return 0;

	kit_to_live_state(&kit);
	return 1;
}

static void save_autosave(void)
{
	DrumKitPreset kit;

	kit_from_live_state(&kit);
	sFLASH_erase_sector(DRUM_AUTOSAVE_SECTOR_ADDR);
	sFLASH_write_buffer((uint8_t *)&kit, DRUM_AUTOSAVE_SECTOR_ADDR, sizeof(kit));
}

static uint8_t load_autosave(void)
{
	DrumKitPreset kit;

	sFLASH_read_buffer((uint8_t *)&kit, DRUM_AUTOSAVE_SECTOR_ADDR, sizeof(kit));
	if (kit.magic != DRUM_PRESET_MAGIC)
		return 0;

	kit_to_live_state(&kit);
	return 1;
}

/* Debounced dirty-tracking for the autosave above: rather than
 * instrumenting every single call site that can change drum_chan[]/
 * grids_x/y/chaos/engine (sliders, encoders, preset loads, ...), just
 * compare a fresh snapshot against the last-seen one each tick -- cheap
 * relative to the ~100ms+ flash erase it's guarding, and every field it
 * covers is already quantized/hysteretic at its source (see
 * read_channel_sliders()), so it settles to a stable snapshot rather
 * than chattering on ADC noise. */
static DrumKitPreset	autosave_last_seen;
static uint8_t			autosave_dirty = 0;
static uint32_t			autosave_last_change_ms = 0;

void update_drum_autosave(void)
{
	DrumKitPreset now_kit;
	uint32_t now_ms = HAL_GetTick() / TICKS_PER_MS;

	kit_from_live_state(&now_kit);
	if (memcmp(&now_kit, &autosave_last_seen, sizeof(now_kit)) != 0) {
		autosave_last_seen = now_kit;
		autosave_last_change_ms = now_ms;
		autosave_dirty = 1;
		return;
	}

	if (autosave_dirty && (now_ms - autosave_last_change_ms) >= DRUM_AUTOSAVE_DEBOUNCE_MS) {
		save_autosave();
		autosave_dirty = 0;
	}
}

void init_drum_preset(void)
{
	for (uint8_t s = 0; s < DRUM_PRESET_NUM_SLOTS; s++) {
		uint32_t magic = 0;
		sFLASH_read_buffer((uint8_t *)&magic, slot_addr(s), sizeof(magic));
		slot_filled[s] = (magic == DRUM_PRESET_MAGIC);
	}
	prev_press_level = rotary_pressed(rotm_PRESET);

	/* Restore whatever was live when the module was last powered off,
	 * independent of the 16 numbered slots above. If there's no
	 * autosave yet (first boot), leave init_drum_ui()'s hard defaults
	 * in place. Either way, seed the dirty-tracking snapshot to match
	 * so update_drum_autosave() doesn't immediately re-save on the
	 * very next tick. */
	load_autosave();
	kit_from_live_state(&autosave_last_seen);
	autosave_dirty = 0;
}

void read_drum_preset_ui(void)
{
	int16_t enc = pop_encoder_q(pec_LOADPRESET);
	if (enc) {
		selected_slot = (uint8_t)_WRAP_I16((int16_t)selected_slot + enc, 0, DRUM_PRESET_NUM_SLOTS - 1);
		start_ongoing_display_drum_preset(DRUM_PRESET_DISP_BROWSE);
	}

	/* rotary_pressed() reports how long the press has *currently*
	 * lasted, not a discrete tap classification, so the load/save
	 * decision is made on the release edge by looking at whichever
	 * level the press had reached right before it. */
	enum PressTypes level = rotary_pressed(rotm_PRESET);
	if (level == RELEASED && prev_press_level != RELEASED) {
		if (prev_press_level == LONG_PRESSED) {
			save_slot(selected_slot);
			start_ongoing_display_drum_preset(DRUM_PRESET_DISP_SAVED);
		} else if (load_slot(selected_slot)) {
			start_ongoing_display_drum_preset(DRUM_PRESET_DISP_LOADED);
		}
	}
	prev_press_level = level;
}

uint8_t drum_preset_selected_slot(void) { return selected_slot; }

uint8_t drum_preset_slot_filled(uint8_t slot)
{
	return (slot < DRUM_PRESET_NUM_SLOTS) ? slot_filled[slot] : 0;
}
