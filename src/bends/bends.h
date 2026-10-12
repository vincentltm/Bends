#pragma once

#include "bends/bend.h"

namespace bends {

// The complete declarative bend personality table for Bends
constexpr Bend kBends[] = {
    // -------------------------------------------------------------
    // P00 (CLK) Bends
    // -------------------------------------------------------------
    // CLK <-> AMP: Rhythmic tremolo chopper
    { NodeClk, NodeAmp, Verb::Modulate, Slot::AmpTremolo,    Sig::ClkPulse,  1.00f, When::Always },
    // CLK <-> FLT: Stepped clock-quantized filter formants
    { NodeClk, NodeFlt, Verb::Modulate, Slot::FltClockStep,  Sig::ClkPulse,  0.75f, When::Always },
    // CLK <-> DLY: Stutter (< 35 Hz) or Decimation (>= 35 Hz)
    { NodeClk, NodeDly, Verb::Set,      Slot::DlyStutter,    Sig::One,       1.00f, When::ClkSlow },
    { NodeClk, NodeDly, Verb::Modulate, Slot::DlyDecimate,   Sig::One,       0.45f, When::ClkFast },
    // CLK <-> OSC: Clock divider triggers sub-octave flip-flop
    { NodeClk, NodeOsc, Verb::Set,      Slot::OscSubOctave,  Sig::One,       0.85f, When::ClkSlow },

    // -------------------------------------------------------------
    // P01 (RST) Bends
    // -------------------------------------------------------------
    // RST <-> GND: Hardware memory flush & feedback dump
    { NodeRst, NodeGnd, Verb::Flush,    Slot::None,          Sig::One,       1.00f, When::Always },
    // RST <-> DLY: 90° quadrature reverse pitch cloud
    { NodeRst, NodeDly, Verb::Set,      Slot::DlyReverse,    Sig::One,       1.00f, When::Always },
    // RST <-> OSC: Hard phase sync
    { NodeRst, NodeOsc, Verb::Sync,     Slot::OscPhase,      Sig::ClkPulse,  1.00f, When::Always },

    // -------------------------------------------------------------
    // P07 (VCC) Bends
    // -------------------------------------------------------------
    // VCC <-> DLY: Power-of-two address bus jump & CD skip
    { NodeVcc, NodeDly, Verb::Set,      Slot::DlyAddressSlip,Sig::One,       1.00f, When::Always },
    // VCC <-> CLK: Overclock speed boost
    { NodeVcc, NodeClk, Verb::Modulate, Slot::DlyOverclock,  Sig::One,       0.35f, When::Always },
    // VCC <-> FBK: Power bus XOR memory address line scrambling
    { NodeVcc, NodeFbk, Verb::Set,      Slot::DlyXorGlitch,  Sig::One,       1.00f, When::Always },
    // VCC <-> AMP: Hot rail saturation boost
    { NodeVcc, NodeAmp, Verb::Modulate, Slot::PreampDrive,   Sig::One,       1.25f, When::Always },
    // VCC <-> FLT: Overvoltage filter howling & diode drive
    { NodeVcc, NodeFlt, Verb::Modulate, Slot::FltDrive,      Sig::One,       1.50f, When::Always },

    // -------------------------------------------------------------
    // P03 (OSC) Bends
    // -------------------------------------------------------------
    // OSC <-> AMP: Four-quadrant ring modulation
    { NodeOsc, NodeAmp, Verb::Set,      Slot::PreampRingMod, Sig::One,       1.00f, When::Always },
    // OSC <-> GND: Grounded upper-octave engages sub-octave divider
    { NodeOsc, NodeGnd, Verb::Set,      Slot::OscSubOctave,  Sig::One,       1.00f, When::Always },
    // OSC <-> FLT: Cutoff ramp sweep (in Saw) + flyback resonant ping
    { NodeOsc, NodeFlt, Verb::Modulate, Slot::FltCutoffRamp, Sig::OscRamp,   4.50f, When::Saw },
    { NodeOsc, NodeFlt, Verb::Trigger,  Slot::FltPing,       Sig::OscReset,  0.85f, When::Saw },
    // OSC <-> DLY: Quadrature chorus/FM (Sin) or ramp barberpole (Saw)
    { NodeOsc, NodeDly, Verb::Modulate, Slot::DlyTimeMod,    Sig::Osc,       8.00f, When::Sin },
    // OSC <-> CLK: Heterodyne audio-rate FM on delay converter clock
    { NodeOsc, NodeClk, Verb::Modulate, Slot::DlyClockFm,    Sig::Osc,       0.35f, When::Always },
    // OSC <-> FBK: Chaotic delayed self-FM cross-modulation
    { NodeOsc, NodeFbk, Verb::Modulate, Slot::FbkRecircFm,   Sig::Osc,       0.95f, When::Always },

    // -------------------------------------------------------------
    // P04 (FLT) Bends
    // -------------------------------------------------------------
    // FLT <-> FBK: Dub delay loop (routing delay feedback through resonant filter)
    { NodeFlt, NodeFbk, Verb::Set,      Slot::FbkFilterLoop, Sig::One,       1.00f, When::Always },
    // FLT <-> MIX: Master output filter isolator
    { NodeFlt, NodeMix, Verb::Set,      Slot::MixFilterIsol, Sig::One,       1.00f, When::Always },

    // -------------------------------------------------------------
    // P06 / P02 (DLY / FBK) Bends
    // -------------------------------------------------------------
    // DLY <-> FBK: Elevated ping-pong rotation cross-coupling
    { NodeDly, NodeFbk, Verb::Set,      Slot::DlyPingPong,   Sig::One,       0.68f, When::Always },
    // FBK <-> MIX: Diffuser spring reverb wash
    { NodeFbk, NodeMix, Verb::Set,      Slot::MixDiffuser,   Sig::One,       0.85f, When::Always }
};

constexpr size_t kNumBends = sizeof(kBends) / sizeof(kBends[0]);

} // namespace bends
