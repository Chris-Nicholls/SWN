# Host-side test harness for the mpump-style drum-voice port
# (Phase 1: kick + snare). Compiles the real firmware sources
# (src/drum_voice_mpump_kick.c, src/drum_voice_mpump_snare.c,
# src/drum_shared_filter.c) straight against inc/drum_voice.h --
# no stubs needed since that header is pure standard C.
#
# Usage:
#   make          # build drumtest
#   make run      # build + run, dumping WAVs into ./out/
#   make clean

CC      ?= clang
CXX     ?= clang++
CFLAGS  ?= -O2 -g -Wall -Wextra -std=c11
CXXFLAGS?= -O2 -g -Wall -Wextra -std=c++17
INCLUDES = -I ../../inc

SRC_DIR = ../../src

OBJS = build/drum_voice_mpump_kick.o build/drum_voice_mpump_snare.o build/drum_shared_filter.o build/test_main.o

BIN = drumtest

all: $(BIN)

$(BIN): $(OBJS)
	$(CXX) $(OBJS) -o $@ -lm

build:
	mkdir -p build

build/drum_voice_mpump_kick.o: $(SRC_DIR)/drum_voice_mpump_kick.c ../../inc/drum_voice.h ../../inc/drum_shared_filter.h | build
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/drum_voice_mpump_snare.o: $(SRC_DIR)/drum_voice_mpump_snare.c ../../inc/drum_voice.h ../../inc/drum_shared_filter.h | build
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/drum_shared_filter.o: $(SRC_DIR)/drum_shared_filter.c ../../inc/drum_shared_filter.h ../../inc/drum_voice.h | build
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/test_main.o: test_main.cpp wav_io.hpp ../../inc/drum_voice.h | build
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

run: $(BIN)
	./$(BIN) out

clean:
	rm -rf build $(BIN) out/*.wav

.PHONY: all run clean
