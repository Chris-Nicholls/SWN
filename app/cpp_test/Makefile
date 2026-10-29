# Host-side test harness for the Halo physics + audio loop.
# Drops src/halo.c straight into a tiny C++ wrapper so we can
# exercise the same code that runs on the SWN, but on the dev box.
#
# Usage:
#   make                # build ringtest
#   make run            # build + run, dumping WAVs into ./out/
#   make clean

CC      ?= clang
CXX     ?= clang++
CFLAGS  ?= -O2 -g -Wall -Wextra -Wno-unused-parameter
CXXFLAGS?= -O2 -g -Wall -Wextra -std=c++17
INCLUDES = -I include -I stubs

SRCS_C   = build/halo.c
SRCS_CXX = halo_voice.cpp halo_voice_amortized.cpp test_main.cpp

OBJS = build/halo.o build/halo_voice.o build/halo_voice_amortized.o build/test_main.o

BIN = ringtest

all: $(BIN)

$(BIN): $(OBJS)
	$(CXX) $(OBJS) -o $@

# Mirror the firmware halo.c & halo.h into build/include so
# the host-side stub headers (globals.h, sphere.h, oscillator.h, etc.)
# get picked up first via the include search path.
build/halo.c: ../../src/halo.c | build
	cp $< $@

include/halo.h: ../../inc/halo.h | include
	cp $< $@

build:
	mkdir -p build

include:
	mkdir -p include

build/halo.o: build/halo.c include/halo.h | build
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/halo_voice.o: halo_voice.cpp halo_voice.hpp include/halo.h | build
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

build/halo_voice_amortized.o: halo_voice_amortized.cpp halo_voice_amortized.hpp | build
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

build/test_main.o: test_main.cpp halo_voice.hpp halo_voice_amortized.hpp wav_io.hpp include/halo.h | build
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

run: $(BIN)
	./$(BIN) out

clean:
	rm -rf build include $(BIN) out

.PHONY: all run clean
