# Host-side test harness for the ROLLER-style drum-voice ports
# (kick, snare, closed/open hat, ride, crash, perc, rimshot). Compiles the
# real firmware sources straight against inc/drum_voice.h -- no stubs,
# since that header is pure standard C.
#
# Usage:
#   make          # build drumtest_roller
#   make run      # build + run, dumping WAVs into ./out/
#   make compare  # build + run + spectral spot check vs py_roller_*.wav
#   make arm      # cross-compile-only check with arm-none-eabi-gcc
#   make clean

CC      ?= clang
CXX     ?= clang++
CFLAGS  ?= -O2 -g -Wall -Wextra -std=gnu11
CXXFLAGS?= -O2 -g -Wall -Wextra -std=c++17
INCLUDES = -I ../../inc

SRC_DIR = ../../src
HDRS    = ../../inc/drum_voice.h ../../inc/drum_shared_filter.h \
          ../../inc/drum_roller_common.h ../../inc/drum_roller_metal.h

VOICES = drum_voice_roller_kick drum_voice_roller_snare \
         drum_voice_roller_closed_hat drum_voice_roller_open_hat \
         drum_voice_roller_ride drum_voice_roller_crash \
         drum_voice_roller_perc drum_voice_roller_rimshot

SUPPORT = drum_roller_metal drum_shared_filter

OBJS = $(addprefix build/,$(addsuffix .o,$(VOICES) $(SUPPORT))) build/test_main.o

BIN = drumtest_roller

ARM_CC     ?= $(HOME)/arm-gnu-toolchain/bin/arm-none-eabi-gcc
ARM_CFLAGS ?= -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard \
              -O2 -Wall -Wextra -std=gnu11

all: $(BIN)

$(BIN): $(OBJS)
	$(CXX) $(OBJS) -o $@ -lm

build:
	mkdir -p build out

build/%.o: $(SRC_DIR)/%.c $(HDRS) | build
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/test_main.o: test_main.cpp wav_io.hpp $(HDRS) | build
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

run: $(BIN)
	./$(BIN) out

# Onset-spectrum spot check against the py_roller_*.wav reference renders.
compare: run
	python3 compare_to_python.py

# Compile-only sanity check that the new voices build for the F765 target.
arm: | build
	@for v in $(VOICES) $(SUPPORT); do \
		echo "  arm-cc $$v"; \
		$(ARM_CC) $(ARM_CFLAGS) $(INCLUDES) -c $(SRC_DIR)/$$v.c -o build/$$v.arm.o || exit 1; \
	done
	@echo "ARM cross-compile OK"

clean:
	rm -rf build $(BIN) out/*.wav

.PHONY: all run compare arm clean
