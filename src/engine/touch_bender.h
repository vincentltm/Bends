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

    enum PadId : size_t {
        PadDrone       = 0,  // Drone VCA and primary pitch trigger
        PadFilterPing  = 1,  // Filter ping and bandpass excitation
        PadClick       = 2,  // Click trigger and percussion strike
        PadHumAntenna  = 3,  // Body hum / audio antenna
        PadStarve      = 4,  // Power rail starvation and sag
        PadReverse     = 5,  // Dedicated delay reverse
        PadClockBend   = 6,  // Master delay clock bend
        PadRadioWhine  = 7,  // High-Z parasitic RF whine and feedback bridge
        PadDiffusion   = 8,  // Reverb diffusion and chassis ground reference
        PadChassisGnd  = 8,  // Backward compatibility alias
        PadStutter     = 9,  // Granular stutter slice and glitch
        PadDroneRibbon = 10, // Drone pitch bend ribbon
        PadStarveUtil  = 11  // Starve utility bridge
    };

    void Init() {
        _is_latched = false;
        for (size_t i = 0; i < kNumPads; ++i) {
            _pressures[i] = 0.0f;
            _latched_pressures[i] = 0.0f;
            _latch_released[i] = true;
            for (size_t j = 0; j < kNumPads; ++j) {
                _conductance[i][j] = 0.0f;
                _latched_conductance[i][j] = 0.0f;
                _weights[i][j] = CalculateWeight(i, j);
            }
        }
        _current_load = 0.0f;
    }

    void Latch() {
        _is_latched = true;
        for (size_t i = 0; i < kNumPads; ++i) {
            _latched_pressures[i] = _pressures[i];
            _latch_released[i] = (_pressures[i] <= 0.02f);
            for (size_t j = 0; j < kNumPads; ++j) {
                _latched_conductance[i][j] = _conductance[i][j];
            }
        }
    }

    void Unlatch() {
        _is_latched = false;
        for (size_t i = 0; i < kNumPads; ++i) {
            _latched_pressures[i] = 0.0f;
            _latch_released[i] = true;
            for (size_t j = 0; j < kNumPads; ++j) {
                _latched_conductance[i][j] = 0.0f;
            }
        }
    }

    bool IsLatched() const { return _is_latched; }

    void Process(const synthux::Pads& pads) {
        float sum_p = 0.0f;
        float sum_g = 0.0f;

        if (_is_latched) {
            for (size_t i = 0; i < kNumPads; ++i) {
                float live_p = pads.Pressure(i);
                if (live_p <= 0.02f) {
                    _latch_released[i] = true;
                }

                float add_p = 0.0f;
                if (_latch_released[i]) {
                    add_p = live_p;
                } else if (live_p > _latched_pressures[i]) {
                    add_p = live_p - _latched_pressures[i];
                }

                float total_p = _latched_pressures[i] + add_p;
                _pressures[i] = (total_p > 1.0f) ? 1.0f : total_p;
                sum_p += _pressures[i];
            }

            // Bridge conductance between two pads: latched bridges held, live bridges add on top
            for (size_t i = 0; i < kNumPads; ++i) {
                for (size_t j = i + 1; j < kNumPads; ++j) {
                    float live_pi = pads.Pressure(i);
                    float live_pj = pads.Pressure(j);

                    float live_g = 0.0f;
                    if (live_pi > 0.02f && live_pj > 0.02f) {
                        live_g = std::sqrt(live_pi * live_pj) * _weights[i][j];
                    }

                    float total_g = _latched_conductance[i][j] + live_g;
                    if (total_g > 1.0f) total_g = 1.0f;
                    _conductance[i][j] = total_g;
                    _conductance[j][i] = total_g;
                    sum_g += total_g;
                }
            }
        } else {
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

    uint8_t ActiveBridgeCount() const {
        uint8_t count = 0;
        for (size_t i = 0; i < kNumPads; ++i) {
            for (size_t j = i + 1; j < kNumPads; ++j) {
                if (_conductance[i][j] > 0.02f) count++;
            }
        }
        return count;
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

    bool _is_latched{false};
    std::array<float, kNumPads> _pressures{};
    std::array<float, kNumPads> _latched_pressures{};
    std::array<bool, kNumPads> _latch_released{};
    float _conductance[kNumPads][kNumPads]{{0.0f}};
    float _latched_conductance[kNumPads][kNumPads]{{0.0f}};
    float _weights[kNumPads][kNumPads]{{0.0f}};
    float _current_load{0.0f};
};

} // namespace bends
