#pragma once

#include "pads.h"
#include <cmath>
#include <array>
#include <algorithm>

namespace bends {

// Tracks pad pressure, pad-to-pad bridging, and current draw on the touch surface.
class TouchBender {
public:
    static constexpr size_t kNumPads = 12;

    void Init() {
        for (size_t i = 0; i < kNumPads; ++i) {
            _pressures[i] = 0.0f;
            for (size_t j = 0; j < kNumPads; ++j) {
                _conductance[i][j] = 0.0f;
                _weights[i][j] = CalculateWeight(i, j);
            }
        }
        _current_load = 0.0f;
    }

    void Process(const synthux::Pads& pads) {
        float sum_p = 0.0f;
        float sum_g = 0.0f;

        for (size_t i = 0; i < kNumPads; ++i) {
            _pressures[i] = pads.Pressure(i);
            sum_p += _pressures[i];
        }

        // Bridge conductance between two pads
        for (size_t i = 0; i < kNumPads; ++i) {
            for (size_t j = i + 1; j < kNumPads; ++j) {
                float pi = _pressures[i];
                float pj = _pressures[j];

                if (pi > 0.02f && pj > 0.02f) {
                    float g = std::sqrt(pi * pj) * _weights[i][j];
                    _conductance[i][j] = g;
                    _conductance[j][i] = g;
                    sum_g += g;
                } else {
                    _conductance[i][j] = 0.0f;
                    _conductance[j][i] = 0.0f;
                }
            }
        }

        // Total current load from touching pads
        _current_load = sum_p * 0.08f + sum_g * 0.20f;
    }

    float Pressure(size_t pad) const {
        return (pad < kNumPads) ? _pressures[pad] : 0.0f;
    }

    float Conductance(size_t a, size_t b) const {
        return (a < kNumPads && b < kNumPads) ? _conductance[a][b] : 0.0f;
    }

    float CurrentLoad() const {
        return _current_load;
    }

    static constexpr bool IsAdjacent(size_t a, size_t b) {
        if (a > b) {
            size_t tmp = a; a = b; b = tmp;
        }
        // Neighbor pads that share a copper border
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

private:
    static float CalculateWeight(size_t a, size_t b) {
        if (a == b) return 0.0f;
        return IsAdjacent(a, b) ? 1.0f : 0.72f;
    }

    std::array<float, kNumPads> _pressures{};
    float _conductance[kNumPads][kNumPads]{{0.0f}};
    float _weights[kNumPads][kNumPads]{{0.0f}};
    float _current_load{0.0f};
};

} // namespace bends
