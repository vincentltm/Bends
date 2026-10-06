#pragma once

#include "circuit/node_table.h"
#include <cmath>
#include <array>

namespace bends {

// AuditionMode provides test tones for each node so the resistor network
// can be tested, heard, and verified on hardware before connecting the real DSP blocks.
class AuditionGenerator {
public:
    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        // Tuned to D minor / D pentatonic intervals for pleasant listening
        constexpr float freqs[kSolverNodes] = {
            110.0f,  // 0: CLK (A2)
            146.83f, // 1: RST (D3)
            164.81f, // 2: VCC (E3)
            220.0f,  // 3: OSC (A3)
            293.66f, // 4: FLT (D4)
            0.0f,    // 5: GND (Silence)
            329.63f, // 6: DLY (E4)
            392.00f, // 7: FBK (G4)
            440.0f,  // 8: AMP (A4)
            587.33f, // 9: MIX (D5)
            659.25f, // 10: IN (E5)
            880.0f,  // 11: OUT (A5)
            50.0f    // 12: BODY (50Hz mains hum)
        };

        for (size_t i = 0; i < kSolverNodes; ++i) {
            _phase[i] = 0.0f;
            _phase_inc[i] = (freqs[i] * 6.2831853f) / sample_rate;
        }
    }

    // Generates test tone vector s[13] for Network::Tick()
    inline void Generate(float s_out[kSolverNodes]) {
        for (size_t i = 0; i < kSolverNodes; ++i) {
            if (_phase_inc[i] > 0.0f) {
                _phase[i] += _phase_inc[i];
                if (_phase[i] >= 6.2831853f) _phase[i] -= 6.2831853f;
                // Soft sine tone
                s_out[i] = std::sin(_phase[i]) * 0.22f;
            } else {
                s_out[i] = 0.0f;
            }
        }
    }

private:
    float _sample_rate{48000.0f};
    float _phase[kSolverNodes]{0.0f};
    float _phase_inc[kSolverNodes]{0.0f};
};

} // namespace bends
