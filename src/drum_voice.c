/*
 * drum_voice.c - registry of all ported drum-voice algorithms
 *
 * -----------------------------------------------------------------------------
 */

#include <stddef.h>

#include "drum_voice.h"

const DrumVoiceEntry kDrumVoiceRegistry[] = {
	{ &drum_voice_mpump_kick,        "mpump kick",     DRUM_CAT_KICK       },
	{ &drum_voice_mpump_snare,       "mpump snare",    DRUM_CAT_SNARE      },
	{ &drum_voice_deluge_kick,       "deluge kick",    DRUM_CAT_KICK       },
	{ &drum_voice_deluge_snare,      "deluge snare",   DRUM_CAT_SNARE      },
	{ &drum_voice_deluge_closed_hat, "deluge chat",    DRUM_CAT_CLOSED_HAT },
	{ &drum_voice_deluge_open_hat,   "deluge ohat",    DRUM_CAT_OPEN_HAT   },
	{ &drum_voice_deluge_cowbell,    "deluge cowbell", DRUM_CAT_OTHER      },
	{ &drum_voice_mpump_closed_hat,  "mpump chat",     DRUM_CAT_CLOSED_HAT },
	{ &drum_voice_mpump_open_hat,    "mpump ohat",     DRUM_CAT_OPEN_HAT   },
	{ &drum_voice_mpump_crash,       "mpump crash",    DRUM_CAT_CRASH      },
	{ &drum_voice_mpump_ride,        "mpump ride",     DRUM_CAT_OTHER      },
	{ &drum_voice_mpump_rimshot,     "mpump rim",      DRUM_CAT_OTHER      },
	{ &drum_voice_mpump_tom,         "mpump tom",      DRUM_CAT_OTHER      },
	{ &drum_voice_mpump_cowbell,     "mpump cowbell",  DRUM_CAT_OTHER      },
	{ &drum_voice_mpump_clap,        "mpump clap",     DRUM_CAT_OTHER      },
	{ &drum_voice_chip_kick,         "chip kick",      DRUM_CAT_KICK       },
	{ &drum_voice_chip_snare,        "chip snare",     DRUM_CAT_SNARE      },
	{ &drum_voice_chip_closed_hat,   "chip chat",      DRUM_CAT_CLOSED_HAT },
	{ &drum_voice_chip_open_hat,     "chip ohat",      DRUM_CAT_OPEN_HAT   },
	{ &drum_voice_chip_perc,         "chip perc",      DRUM_CAT_OTHER      },
	{ &drum_voice_chip_cowbell,      "chip cowbell",   DRUM_CAT_OTHER      },
	{ &drum_voice_roller_kick,       "roller kick",    DRUM_CAT_KICK       },
	{ &drum_voice_roller_snare,      "roller snare",   DRUM_CAT_SNARE      },
	{ &drum_voice_roller_closed_hat, "roller chat",    DRUM_CAT_CLOSED_HAT },
	{ &drum_voice_roller_open_hat,   "roller ohat",    DRUM_CAT_OPEN_HAT   },
	{ &drum_voice_roller_ride,       "roller ride",    DRUM_CAT_OTHER      },
	{ &drum_voice_roller_crash,      "roller crash",   DRUM_CAT_CRASH      },
	{ &drum_voice_roller_perc,       "roller perc",    DRUM_CAT_OTHER      },
	{ &drum_voice_roller_rimshot,    "roller rim",     DRUM_CAT_OTHER      },
	{ &drum_voice_plaits_kick,       "plaits kick",    DRUM_CAT_KICK       },
	{ &drum_voice_plaits_snare,      "plaits snare",   DRUM_CAT_SNARE      },
	/* Same voice under both hat categories: on the 808 circuit closed
	 * and open hats are one generator at two decay settings, so it is
	 * worth reaching from either channel's shape cycle. */
	{ &drum_voice_plaits_hihat,      "plaits chat",    DRUM_CAT_CLOSED_HAT },
	{ &drum_voice_plaits_hihat,      "plaits ohat",    DRUM_CAT_OPEN_HAT   },
};

const uint8_t kNumDrumVoices = sizeof(kDrumVoiceRegistry) / sizeof(kDrumVoiceRegistry[0]);

int8_t drum_voice_registry_index(const DrumVoiceOps *ops)
{
	if (!ops)
		return -1;

	for (uint8_t i = 0; i < kNumDrumVoices; i++)
		if (kDrumVoiceRegistry[i].ops == ops)
			return (int8_t)i;

	return -1;
}

const DrumVoiceOps *drum_voice_registry_lookup(uint8_t index)
{
	if (index >= kNumDrumVoices)
		return NULL;

	return kDrumVoiceRegistry[index].ops;
}
