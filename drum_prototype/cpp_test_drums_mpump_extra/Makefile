# Host-side test harness for the remaining mpump-style drum-voice ports
# (closed hat, open hat, crash, ride, rimshot, tom, cowbell, clap).
# Kick and snare are covered by ../cpp_test_drums. Compiles the real
# firmware sources straight against inc/drum_voice.h -- no stubs, since
# that header is pure standard C.
#
# Usage:
#   make          # build drumtest_mpump_extra
#   make run      # build + run, dumping WAVs into ./out/
#   make arm      # cross-compile-only check with arm-none-eabi-gcc
#   make clean

CC      ?= clang
CXX     ?= clang++
CFLAGS  ?= -O2 -g -Wall -Wextra -std=gnu11
CXXFLAGS?= -O2 -g -Wall -Wextra -std=c++17
INCLUDES = -I ../../inc

SRC_DIR = ../../src
HDRS    = ../../inc/drum_voice.h ../../inc/drum_shared_filter.h

VOICES = drum_voice_mpump_closed_hat drum_voice_mpump_open_hat drum_voice_mpump_crash \
         drum_voice_mpump_ride drum_voice_mpump_rimshot drum_voice_mpump_tom \
         drum_voice_mpump_cowbell drum_voice_mpump_clap

OBJS = $(addprefix build/,$(addsuffix .o,$(VOICES))) build/drum_shared_filter.o build/test_main.o

BIN = drumtest_mpump_extra

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

# Onset-spectrum spot check against the py_*.wav reference renders.
compare: run
	python3 compare_to_python.py

# Compile-only sanity check that the new voices build for the F765 target.
arm: | build
	@for v in $(VOICES) drum_shared_filter; do \
		echo "  arm-cc $$v"; \
		$(ARM_CC) $(ARM_CFLAGS) $(INCLUDES) -c $(SRC_DIR)/$$v.c -o build/$$v.arm.o || exit 1; \
	done
	@echo "ARM cross-compile OK"

clean:
	rm -rf build $(BIN) out/*.wav

.PHONY: all run compare arm clean
