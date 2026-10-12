# Bends Refactor (v3)

Clean, solver-centric embedded audio engine for virtual circuit bending on the Daisy Seed.

## Documentation

- **Full Architecture Specification**: [`../Docs/architecture_refactor.md`](../Docs/architecture_refactor.md)
- **Original Baseline Firmware**: [`../Bends/`](../Bends/) (reference implementation)
- **v0.1 Prototype**: [`../Legacy/`](../Legacy/)

## Core Design

1. **Solver-Centric**: Every pad is a circuit node. A single 13×13 nodal solve per block produces DC pulls, AC injections, and ground retentions.
2. **Pad-Agnostic Parts**: DSP blocks in `src/parts/` expose modulation slots and never query pad numbers directly.
3. **Declarative Bends**: All custom bends and circuit hacks live in `src/bends/bends.h`.
4. **Race-Free Concurrency**: Lock-free triple-buffered control frame handoff between I2C main loop and the 750 Hz audio interrupt.

## Directory Structure

```
src/
├── main.cpp                 # Boot, audio callback, main loop
├── app/                     # Instrument coordinator and telemetry
├── surface/                 # MPR121 pads, knobs, switches, gestures
├── board/                   # Nodes, contacts, solver, rail, board
├── parts/                   # DSP parts (clock, osc, filter, delay, etc.)
├── bends/                   # Slots, verbs, and the bend table
└── dsp/                     # Math, filters, SDRAM circular buffers
```

## Building

```bash
make -j4
```
