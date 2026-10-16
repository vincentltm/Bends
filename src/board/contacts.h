#pragma once

#include "board/nodes.h"
#include "surface/frame.h"
#include <cmath>
#include <algorithm>

namespace bends {

struct ActiveBridge {
    uint8_t a;
    uint8_t b;
    float   g;
};

class Contacts {
public:
    static constexpr size_t kMaxActiveBridges = 24;

    Contacts() {
        Reset();
    }

    void Reset() {
        _prev_is_latched = false;
        _has_latched_snapshot = false;
        _was_touching_first_bend = false;
        _unlatch_timer = 0;
        _num_active_bridges = 0;
        _total_current_load = 0.0f;

        for (size_t i = 0; i < kSolverNodes; ++i) {
            _pressures[i] = 0.0f;
            _latched_pressures[i] = 0.0f;
            for (size_t j = 0; j < kSolverNodes; ++j) {
                _conductance[i][j] = 0.0f;
                _latched_conductance[i][j] = 0.0f;
                _weights[i][j] = NodeTable::PhysicalWeight(i, j);
            }
        }
    }

    // Process touch surface contacts at block rate (750 Hz)
    void Process(const ControlFrame& frame, float bend_depth = 1.0f) {
        float sum_p = 0.0f;
        float sum_g = 0.0f;

        bool is_latched = frame.is_latched;
        _prev_is_latched = is_latched;

        if (!is_latched) {
            ClearLatched();
        }

        // 1. Calculate live physical bridges between pairs touched simultaneously
        float live_g[kPhysicalNodes][kPhysicalNodes] = {{0.0f}};
        bool any_live_touch = false;
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float pi = frame.pressures[i];
            if (pi > 0.02f) any_live_touch = true;
            for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                float pj = frame.pressures[j];
                if (pi > 0.02f && pj > 0.02f) {
                    float series_p = (2.0f * pi * pj) / (pi + pj);
                    float g = series_p * _weights[i][j] * bend_depth;
                    live_g[i][j] = std::min(1.0f, g);
                    live_g[j][i] = live_g[i][j];
                }
            }
        }

        // 2. Latch snapshot management:
        // While latched and snapshot is not yet locked:
        // Capture peak pressures and conductances continuously as long as user is touching pads.
        // Once user lifts their fingers, snapshot locks permanently so subsequent touches are purely momentary!
        if (is_latched && !_has_latched_snapshot) {
            if (any_live_touch) {
                for (size_t i = 0; i < kPhysicalNodes; ++i) {
                    if (frame.pressures[i] > 0.02f) {
                        _latched_pressures[i] = std::max(_latched_pressures[i], frame.pressures[i]);
                    }
                    for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                        if (live_g[i][j] > 0.015f) {
                            _latched_conductance[i][j] = std::max(_latched_conductance[i][j], live_g[i][j]);
                            _latched_conductance[j][i] = _latched_conductance[i][j];
                        }
                    }
                }
                _was_touching_first_bend = true;
            } else if (_was_touching_first_bend) {
                _has_latched_snapshot = true;
                _was_touching_first_bend = false;
            }
        }

        // 3. Assemble total conductance matrix and active bridge list
        _num_active_bridges = 0;
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            _conductance[i][i] = 0.0f;
            for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                float total_g = live_g[i][j];
                if (is_latched) {
                    total_g += _latched_conductance[i][j];
                }
                total_g = std::min(1.0f, total_g);
                _conductance[i][j] = total_g;
                _conductance[j][i] = total_g;

                if (total_g > 0.015f) {
                    if (_num_active_bridges < kMaxActiveBridges) {
                        _active_bridges[_num_active_bridges++] = {
                            static_cast<uint8_t>(i),
                            static_cast<uint8_t>(j),
                            total_g
                        };
                    }
                    sum_g += total_g;
                }
            }
        }

        // 4. Update effective pad pressures
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float live_p = frame.pressures[i];
            float eff_p = live_p;
            if (is_latched && _latched_pressures[i] > 0.02f) {
                eff_p = std::max(live_p, _latched_pressures[i]);
            }
            _pressures[i] = eff_p;
            sum_p += eff_p;
        }

        // 5. Capacitive Body Coupling for NodeBody (Node 12)
        float body_p = 0.0f;
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float p = _pressures[i];
            if (p > 0.02f) {
                float g_body = p * 0.15f * bend_depth;
                _conductance[i][NodeBody] = g_body;
                _conductance[NodeBody][i] = g_body;
                body_p += p;
            } else {
                _conductance[i][NodeBody] = 0.0f;
                _conductance[NodeBody][i] = 0.0f;
            }
        }
        _pressures[NodeBody] = std::min(1.0f, body_p);

        // Current draw onto power supply
        _total_current_load = sum_p * 0.06f + sum_g * 0.22f;
    }

    void ClearLatched() {
        _has_latched_snapshot = false;
        _was_touching_first_bend = false;
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _latched_pressures[i] = 0.0f;
            for (size_t j = 0; j < kSolverNodes; ++j) {
                _latched_conductance[i][j] = 0.0f;
            }
        }
    }

    float Conductance(size_t i, size_t j) const {
        return (i < kSolverNodes && j < kSolverNodes) ? _conductance[i][j] : 0.0f;
    }

    float Pressure(size_t i) const {
        return (i < kSolverNodes) ? _pressures[i] : 0.0f;
    }

    float TotalCurrentLoad() const { return _total_current_load; }

    uint8_t NumActiveBridges() const { return _num_active_bridges; }
    const ActiveBridge* ActiveBridges() const { return _active_bridges; }

    bool HasLatched() const {
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            if (_latched_pressures[i] > 0.02f) return true;
            for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                if (_latched_conductance[i][j] > 0.015f) return true;
            }
        }
        return false;
    }

    bool HasLatchedBridges() const {
        return HasLatched();
    }

private:
    float _pressures[kSolverNodes]{0.0f};
    float _latched_pressures[kSolverNodes]{0.0f};
    float _conductance[kSolverNodes][kSolverNodes]{{0.0f}};
    float _latched_conductance[kSolverNodes][kSolverNodes]{{0.0f}};
    float _weights[kSolverNodes][kSolverNodes]{{0.0f}};

    ActiveBridge _active_bridges[kMaxActiveBridges];
    uint8_t      _num_active_bridges{0};
    float        _total_current_load{0.0f};

    bool     _prev_is_latched{false};
    bool     _has_latched_snapshot{false};
    bool     _was_touching_first_bend{false};
    uint32_t _unlatch_timer{0};
};

} // namespace bends
