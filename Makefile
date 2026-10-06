TARGET = bends
OPT = -O2
CPP_STANDARD = -std=gnu++17

CPP_SOURCES = \
    src/main.cpp \
    src/dsp/sdram_buffer.cpp \
    src/touch/knobs.cpp \
    src/touch/pads.cpp \
    src/touch/switches.cpp

C_INCLUDES = \
    -Isrc \
    -Isrc/core \
    -Isrc/dsp \
    -Isrc/touch \
    -Isrc/engine \
    -Isrc/circuit \
    -Isrc/blocks

LIBDAISY_DIR ?= lib/libDaisy
DAISYSP_DIR ?= lib/DaisySP

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

libs:
	$(MAKE) -C $(LIBDAISY_DIR) -j4
	$(MAKE) -C $(DAISYSP_DIR) -j4

clean-libs:
	$(MAKE) -C $(LIBDAISY_DIR) clean
	$(MAKE) -C $(DAISYSP_DIR) clean

.PHONY: libs clean-libs
