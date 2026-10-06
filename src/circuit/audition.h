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

        // Initialize 256-entry sine wavetable
        for (size_t i = 0; i < 256; ++i) {
            float rad = (2.0f * 3.14159265f * static_cast<float>(i)) / 256.0f;
            _sine_lut[i] = std::sin(rad) * 0.25f;
        }

        // Tuned to D pentatonic intervals for clear harmonic audition
        constexpr float freqs[kSolverNodes] = {
            110.00f, // 0: CLK (A2)
            146.83f, // 1: RST (D3)
            164.81f, // 2: VCC (E3)
            220.00f, // 3: OSC (A3)
            293.66f, // 4: FLT (D4)
            0.00f,   // 5: GND (Silence)
            329.63f, // 6: DLY (E4)
            392.00f, // 7: FBK (G4)
            440.00f, // 8: AMP (A4)
            587.33f, // 9: MIX (D5)
            659.25f, // 10: IN (E5)
            880.00f, // 11: OUT (A5)
            50.00f   // 12: BODY (50Hz mains hum)
        };

        for (size_t i = 0; i < kSolverNodes; ++i) {
            _phase[i] = 0;
            if (freqs[i] > 0.0f) {
                // 32-bit phase increment: inc = (freq / sample_rate) * 2^32
                double inc = (static_cast<double>(freqs[i]) / sample_rate) * 4294967296.0;
                _phase_inc[i] = static_cast<uint32_t>(inc);
            } else {
                _phase_inc[i] = 0;
            }
        }
    }

    // Ultra-fast wavetable generator: ~2 cycles per node, zero transcendental math!
    inline void Generate(float s_out[kSolverNodes]) {
        for (size_t i = 0; i < kSolverNodes; ++i) {
            uint32_t inc = _phase_inc[i];
            if (inc > 0) {
                _phase[i] += inc;
                uint8_t lut_idx = static_cast<uint8_t>(_phase[i] >> 24);
                s_out[i] = _sine_lut[lut_idx];
            } else {
                s_out[i] = 0.0f;
            }
        }
    }

private:
    float _sample_rate{48000.0f};
    float _sine_lut[256]{0.0f};
    uint32_t _phase[kSolverNodes]{0};
    uint32_t _phase_inc[kSolverNodes]{0};
};

} // namespace bends
