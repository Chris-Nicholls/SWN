# Host-side test harness for the Deluge-style drum-voice port
# (kick, snare, closed hat, open hat, cowbell). Compiles the real
# firmware sources straight against inc/drum_voice.h -- no stubs, since
# that header is pure standard C.
#
# Usage:
#   make          # build drumtest_deluge
#   make run      # build + run, dumping WAVs into ./out/
#   make arm      # cross-compile-only check with arm-none-eabi-gcc
#   make clean

CC      ?= clang
CXX     ?= clang++
CFLAGS  ?= -O2 -g -Wall -Wextra -std=c11
CXXFLAGS?= -O2 -g -Wall -Wextra -std=c++17
INCLUDES = -I ../../inc

SRC_DIR = ../../src
HDRS    = ../../inc/drum_voice.h ../../inc/drum_shared_filter.h

VOICES = drum_voice_deluge_kick drum_voice_deluge_snare drum_voice_deluge_closed_hat \
         drum_voice_deluge_open_hat drum_voice_deluge_cowbell

OBJS = $(addprefix build/,$(addsuffix .o,$(VOICES))) build/drum_shared_filter.o build/test_main.o

BIN = drumtest_deluge

ARM_CC     ?= $(HOME)/arm-gnu-toolchain/bin/arm-none-eabi-gcc
ARM_CFLAGS ?= -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard \
              -O2 -Wall -Wextra -std=gnu11 -ffast-math

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

# Compile-only sanity check that the new voices build for the F765 target.
arm: | build
	@for v in $(VOICES) drum_shared_filter; do \
		echo "  arm-cc $$v"; \
		$(ARM_CC) $(ARM_CFLAGS) $(INCLUDES) -c $(SRC_DIR)/$$v.c -o build/$$v.arm.o || exit 1; \
	done
	@echo "ARM cross-compile OK"

clean:
	rm -rf build $(BIN) out/*.wav

.PHONY: all run arm clean
