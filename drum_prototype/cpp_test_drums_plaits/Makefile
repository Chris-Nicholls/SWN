# Host-side sanity harness for the Plaits drum-voice wrappers
# (plaits kick, snare, hi-hat). Compiles the real firmware sources -- our
# .cpp wrappers plus the untouched vendored Plaits/stmlib sources -- straight
# against inc/drum_voice.h.
#
# There is deliberately no compare_to_python.py here, unlike the other
# families: these ARE the Mutable Instruments engines, not a reimplementation,
# so there is nothing to converge on. The checks are NaN/Inf, non-silence,
# no clipping and that the hit decays.
#
# Usage:
#   make          # build drumtest_plaits
#   make run      # build + run, dumping WAVs into ./out/
#   make arm      # cross-compile-only check with arm-none-eabi-g++
#   make clean

CC      ?= clang
CXX     ?= clang++
CFLAGS  ?= -O2 -g -Wall -Wextra -std=gnu11
CXXFLAGS?= -O2 -g -Wall -Wextra -std=c++14 -fno-exceptions -fno-rtti
# -DTEST swaps stmlib/dsp/dsp.h's Cortex-M inline asm (ssat/usat/vsqrt.f32)
# for portable C -- required to build the vendored sources on the host.
INCLUDES = -I ../../inc -I ../../src -DTEST

SRC_DIR = ../../src
HDRS    = ../../inc/drum_voice.h ../../inc/drum_shared_filter.h

VOICES  = drum_voice_plaits_kick drum_voice_plaits_snare drum_voice_plaits_hihat

# Untouched vendored sources the wrappers pull in.
VENDOR  = plaits/dsp/engine/bass_drum_engine \
          plaits/dsp/engine/snare_drum_engine \
          plaits/resources \
          stmlib/dsp/units \
          stmlib/utils/random

SUPPORT_C = drum_shared_filter

OBJS = $(addprefix build/,$(addsuffix .o,$(VOICES) $(VENDOR))) \
       $(addprefix build/,$(addsuffix .o,$(SUPPORT_C))) \
       build/test_main.o

BIN = drumtest_plaits

ARM_CXX     ?= $(HOME)/arm-gnu-toolchain/bin/arm-none-eabi-g++
# -D_USE_MATH_DEFINES/-DM_PI mirror the firmware Makefile's CXXFLAGS; the
# vendored Plaits headers need M_PI and newlib-nano does not define it
# under -std=c++14.
ARM_CXXFLAGS?= -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
               -O2 -Wall -Wextra -std=c++14 -fno-exceptions -fno-rtti \
               -D_USE_MATH_DEFINES -DM_PI=3.14159265358979323846

all: $(BIN)

$(BIN): $(OBJS)
	$(CXX) $(OBJS) -o $@ -lm

dirs:
	mkdir -p build out build/plaits/dsp/engine build/plaits build/stmlib/dsp build/stmlib/utils

build/%.o: $(SRC_DIR)/%.c $(HDRS) | dirs
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/%.o: $(SRC_DIR)/%.cpp $(HDRS) | dirs
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

build/%.o: $(SRC_DIR)/%.cc | dirs
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

build/test_main.o: test_main.cpp wav_io.hpp $(HDRS) | dirs
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

run: $(BIN)
	./$(BIN) out

# Compile-only sanity check that the wrappers build for the F765 target.
arm: | dirs
	@for v in $(VOICES); do \
		echo "  arm-c++ $$v"; \
		$(ARM_CXX) $(ARM_CXXFLAGS) $(INCLUDES) -c $(SRC_DIR)/$$v.cpp -o build/$$v.arm.o || exit 1; \
	done
	@echo "ARM cross-compile OK"

clean:
	rm -rf build $(BIN) out/*.wav

.PHONY: all dirs run arm clean
