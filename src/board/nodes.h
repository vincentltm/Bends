#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

namespace bends {

enum NodeId : uint8_t {
    NodeClk  = 0,  // P00: Master clock (timing) - Upper-Left Ridge
    NodeRst  = 1,  // P01: Reset pull-up - Top Center Axis
    NodeFbk  = 2,  // P02: Feedback loop - Upper-Right Ridge
    NodeOsc  = 3,  // P03: Probe oscillator - Far-Left Wing
    NodeFlt  = 4,  // P04: Low-pass filter - Mid-Center Left
    NodeGnd  = 5,  // P05: Ground plane (0V) - Bottom Center Axis
    NodeDly  = 6,  // P06: Delay read-head - Mid-Center Right
    NodeVcc  = 7,  // P07: Power rail - Far-Right Wing
    NodeAmp  = 8,  // P08: Preamp / Drive - Bottom-Left Corner
    NodeMix  = 9,  // P09: Master wet/dry - Bottom-Right Corner
    NodeIn   = 10, // P10: Ext In probe / Chassis contact mic Left
    NodeOut  = 11, // P11: Ext Out probe / Chassis contact mic Right
    NodeBody = 12, // Virtual hidden AC node (Hand/Body/Hum)
    kTotalNodes = 13
};

constexpr size_t kPhysicalNodes = 12; // P00 to P11
constexpr size_t kSolverNodes   = 13; // Includes virtual Body node

struct NodeSpec {
    const char* name;
    float rest_dc;        // Normalized DC rest voltage (0.0 to 1.0)
    float z_dc;           // DC source impedance (conductance y = 1/z)
    float z_ac;           // AC source impedance
    float writer_gain;    // Signal amplitude scaling
    bool  is_audio_node;  // True if biased to virtual ground (0.5V)
};

struct NodeTable {
    static constexpr std::array<NodeSpec, kSolverNodes> Specs = {{
        // Name,       Rest,   Z_dc,  Z_ac,  WriterGain, IsAudio
        { "CLK",       0.50f,  0.20f, 0.20f, 0.50f,      false }, // P00: Master clock
        { "RST",       1.00f,  3.00f, 3.00f, 0.00f,      false }, // P01: Weak pull-up
        { "FBK",       0.50f,  0.50f, 0.50f, 0.70f,      true  }, // P02: Feedback tap
        { "OSC",       0.50f,  0.40f, 0.40f, 0.60f,      true  }, // P03: Probe oscillator
        { "FLT",       0.50f,  0.50f, 0.50f, 0.70f,      true  }, // P04: Filter out
        { "GND",       0.00f,  0.02f, 0.02f, 0.00f,      false }, // P05: Stiff 0V ground
        { "DLY",       0.50f,  0.50f, 0.50f, 0.70f,      true  }, // P06: Delay wet out
        { "VCC",       1.00f,  0.05f, 0.02f, 0.00f,      false }, // P07: Rail fader reference
        { "AMP",       0.50f,  0.40f, 0.40f, 0.70f,      true  }, // P08: Preamp out
        { "MIX",       0.50f,  0.35f, 0.35f, 0.70f,      true  }, // P09: Master out
        { "IN",        0.50f,  0.20f, 0.20f, 0.70f,      true  }, // P10: Ext in / Mic L
        { "OUT",       0.50f,  0.20f, 0.20f, 0.70f,      true  }, // P11: Ext out / Mic R
        { "BODY",      0.50f,  10.0f, 1.50f, 1.00f,      false }  // Virtual Body/Hum
    }};

    // Physical pad adjacency on Synthux Touch 2 PCB (18 direct copper borders)
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
