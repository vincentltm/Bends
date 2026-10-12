TARGET = bends
OPT = -Os
CPP_STANDARD = -std=gnu++17

CPP_SOURCES = \
    src/main.cpp \
    src/surface/knobs.cpp \
    src/surface/pads.cpp \
    src/surface/switches.cpp

C_INCLUDES = \
    -Isrc \
    -Isrc/app \
    -Isrc/surface \
    -Isrc/board \
    -Isrc/parts \
    -Isrc/bends \
    -Isrc/dsp

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
