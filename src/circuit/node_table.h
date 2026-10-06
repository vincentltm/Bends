#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace bends {

enum NodeId : uint8_t {
    NodeClk  = 0,  // P00: Master clock (timing)
    NodeRst  = 1,  // P01: Reset pull-up
    NodeVcc  = 2,  // P02: Power rail
    NodeOsc  = 3,  // P03: Probe oscillator
    NodeFlt  = 4,  // P04: Low-pass filter
    NodeGnd  = 5,  // P05: Ground plane (0V)
    NodeDly  = 6,  // P06: Delay read-head
    NodeFbk  = 7,  // P07: Feedback loop
    NodeAmp  = 8,  // P08: Preamp / Drive
    NodeMix  = 9,  // P09: Master wet/dry
    NodeIn   = 10, // P10: Ext In probe
    NodeOut  = 11, // P11: Ext Out probe
    NodeBody = 12, // Virtual hidden AC node (Hand/Hum)
    kTotalNodes = 13
};

constexpr size_t kPhysicalNodes = 12; // P00 to P11
constexpr size_t kSolverNodes   = 13; // Includes virtual Body node

struct NodeSpec {
    const char* name;
    float rest_dc;        // Normalized DC rest voltage (0.0 to 1.0)
    float z_dc;           // DC source impedance (stiffness = 1/z)
    float z_ac;           // AC source impedance
    float writer_gain;    // Amplitude scaling for outgoing signal s_i
    bool  writes_signal;  // True if node carries an audio/AC generator
};

struct ReaderSpec {
    float audio_inject_gain; // Gain for injecting AC into block audio input
    float pull_gain;         // Gain for DC pull modulation [-1, +1]
    float lf_mod_gain;       // Gain for slow AC (crossover < 20 Hz)
    float hf_mod_gain;       // Gain for audio AC (crossover >= 20 Hz, FM/AM)
    float load_pull_gain;    // Capacitive load frequency drag
};

// Compile-time configuration table for node specs and default gains
struct NodeTable {
    static constexpr std::array<NodeSpec, kSolverNodes> Specs = {{
        // Name,       Rest,   Z_dc,  Z_ac,  WriterGain, WritesSignal
        { "CLK",       0.50f,  0.20f, 0.20f, 0.50f,      true  }, // P00
        { "RST",       1.00f,  3.00f, 3.00f, 0.00f,      false }, // P01: Weak pull-up
        { "VCC",       1.00f,  0.05f, 0.02f, 0.00f,      false }, // P02: Controlled by rail
        { "OSC",       0.50f,  0.40f, 0.40f, 0.60f,      true  }, // P03: Probe oscillator
        { "FLT",       0.50f,  0.50f, 0.50f, 0.70f,      true  }, // P04: Filter out
        { "GND",       0.00f,  0.02f, 0.02f, 0.00f,      false }, // P05: Stiff 0V ground
        { "DLY",       0.50f,  0.50f, 0.50f, 0.70f,      true  }, // P06: Delay wet out
        { "FBK",       0.50f,  0.50f, 0.50f, 0.70f,      true  }, // P07: Feedback tap
        { "AMP",       0.50f,  0.40f, 0.40f, 0.70f,      true  }, // P08: Preamp out
        { "MIX",       0.50f,  0.35f, 0.35f, 0.70f,      true  }, // P09: Master out
        { "IN",        0.50f,  0.20f, 0.20f, 0.70f,      true  }, // P10: Ext in
        { "OUT",       0.50f,  0.20f, 0.20f, 0.70f,      true  }, // P11: Ext out
        { "BODY",      0.50f,  10.0f, 1.50f, 1.00f,      true  }  // Virtual AC hum/air
    }};

    static constexpr std::array<ReaderSpec, kSolverNodes> Readers = {{
        // AudioInj, Pull,   LFMod,  HFMod,  LoadPull
        { 0.00f,     1.00f,  0.30f,  0.20f,  0.25f }, // CLK
        { 0.00f,     0.00f,  0.00f,  0.00f,  0.00f }, // RST: Evaluated via Schmitt trigger
        { 0.00f,     1.00f,  0.30f,  0.30f,  0.00f }, // VCC: Direct rail pull
        { 0.00f,     1.00f,  0.40f,  0.15f,  0.25f }, // OSC: Detune & injection lock
        { 0.70f,     1.00f,  1.00f,  0.20f,  0.00f }, // FLT: Cutoff sweep/FM
        { 0.00f,     0.00f,  0.00f,  0.00f,  0.00f }, // GND: Shunts everything
        { 0.30f,     1.00f,  1.00f,  0.25f,  0.00f }, // DLY: Clock / readhead mod
        { 0.70f,     1.00f,  0.80f,  0.20f,  0.00f }, // FBK: Loop injection
        { 1.00f,     1.00f,  0.60f,  0.30f,  0.00f }, // AMP: Audio input summing & bias
        { 0.50f,     0.50f,  0.20f,  0.10f,  0.00f }, // MIX: Output bleed
        { 0.00f,     0.00f,  0.00f,  0.00f,  0.00f }, // IN
        { 0.00f,     0.00f,  0.00f,  0.00f,  0.00f }, // OUT
        { 0.00f,     0.00f,  0.00f,  0.00f,  0.00f }  // BODY
    }};

    // Checks physical pad adjacency on Touch 2 PCB
    static constexpr bool IsAdjacent(size_t a, size_t b) {
        if (a > b) {
            size_t tmp = a; a = b; b = tmp;
        }
        return (a == 0 && (b == 1 || b == 4)) ||
               (a == 1 && (b == 2 || b == 4 || b == 5 || b == 6)) ||
               (a == 2 && b == 6) ||
               (a == 3 && (b == 4 || b == 8)) ||
               (a == 4 && (b == 5 || b == 8)) ||
               (a == 5 && (b == 6 || b == 8 || b == 9)) ||
               (a == 6 && (b == 7 || b == 9)) ||
               (a == 7 && b == 9) ||
               (a == 8 && b == 9);
    }

    static constexpr float PhysicalWeight(size_t a, size_t b) {
        if (a == b) return 0.0f;
        if (a >= kPhysicalNodes || b >= kPhysicalNodes) return 0.72f;
        return IsAdjacent(a, b) ? 1.0f : 0.72f;
    }
};

} // namespace bends
