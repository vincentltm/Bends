#pragma once

#include "circuit/node_table.h"
#include "touch/pads.h"
#include "touch/air.h"
#include <cmath>
#include <algorithm>
#include <array>

namespace bends {

// NetFrame carries the resolved circuit state from control rate (750 Hz)
struct NetFrame {
    // DC pull: normalized deviation from rest voltage in [-1.0, +1.0]
    float pull[kSolverNodes]{0.0f};
    // Total load: sum of all conductances attached to node i
    float load[kSolverNodes]{0.0f};

    float total_current_load{0.0f};
    uint8_t active_bridge_count{0};
};

class Network {
public:
    Network() {
        Reset();
    }

    void Reset() {
        _prev_is_latched = false;
        _unlatch_timer = 0;
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _pressures[i] = 0.0f;
            _latched_pressures[i] = 0.0f;
            _latch_released[i] = false;
            _smooth_pull[i] = 0.0f;
            for (size_t j = 0; j < kSolverNodes; ++j) {
                _conductance[i][j] = 0.0f;
                _latched_conductance[i][j] = 0.0f;
                _weights[i][j] = NodeTable::PhysicalWeight(i, j);
            }
        }
        _frame = NetFrame{};
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _frame.pull[i] = 0.0f;
            _frame.load[i] = 0.0f;
        }
    }

    // Runs once per audio block (~750 Hz)
    void Process(const synthux::Pads& pads,
                 const synthux::AirTracker& air,
                 float rail_voltage,
                 float bend_depth,
                 bool is_latched) {
        float sum_p = 0.0f;
        float sum_g = 0.0f;

        // Detect latch transitions
        bool latch_just_engaged = (is_latched && !_prev_is_latched);
        bool latch_just_disengaged = (!is_latched && _prev_is_latched);
        _prev_is_latched = is_latched;

        if (latch_just_disengaged) {
            _unlatch_timer = 260; // ~350ms grace period at 750Hz control rate
        } else if (!is_latched) {
            if (_unlatch_timer > 0) {
                if (--_unlatch_timer == 0) {
                    ClearLatchedBridges();
                }
            }
        }

        // 1. Calculate live physical bridges between pairs currently touched by fingers
        float live_g[kPhysicalNodes][kPhysicalNodes] = {{0.0f}};
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float live_pi = pads.Pressure(i);
            for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                float live_pj = pads.Pressure(j);
                if (live_pi > 0.02f && live_pj > 0.02f) {
                    float series_p = (2.0f * live_pi * live_pj) / (live_pi + live_pj);
                    float g = series_p * _weights[i][j] * bend_depth;
                    live_g[i][j] = std::min(1.0f, g);
                    live_g[j][i] = live_g[i][j];
                }
            }
        }

        // 2. If latch was just engaged, snapshot active bridges into latched conductance
        if (latch_just_engaged) {
            _unlatch_timer = 0;
            for (size_t i = 0; i < kPhysicalNodes; ++i) {
                if (pads.Pressure(i) > 0.02f) {
                    _latched_pressures[i] = pads.Pressure(i);
                }
                for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                    if (live_g[i][j] > 0.015f) {
                        // Additive latching: merge newly held bridge into latched set
                        _latched_conductance[i][j] = std::max(_latched_conductance[i][j], live_g[i][j]);
                        _latched_conductance[j][i] = _latched_conductance[i][j];
                    }
                }
            }
        }

        // 3. Assemble total conductance matrix:
        // Latched bridges remain held without cross-talk; live bridges add on top.
        uint8_t bridge_count = 0;
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
                    bridge_count++;
                    sum_g += total_g;
                }
            }
        }

        // 4. Update effective pad pressures
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float live_p = pads.Pressure(i);
            float eff_p = live_p;
            if (is_latched && _latched_pressures[i] > 0.02f) {
                eff_p = std::max(live_p, _latched_pressures[i]);
            }
            _pressures[i] = eff_p;
            sum_p += eff_p;
        }

        // 3. Capacitive Body Coupling & Air Proximity for NodeBody (NodeId 12)
        // Body coupling scales with total touch pressure
        float body_p = 0.0f;
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float p = _pressures[i];
            if (p > 0.02f) {
                // Capacitive touch bleed into virtual body node
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

        _frame.active_bridge_count = bridge_count;
        _frame.total_current_load  = sum_p * 0.06f + sum_g * 0.22f;

        // 4. Calculate total load per node
        for (size_t i = 0; i < kSolverNodes; ++i) {
            float node_load = 0.0f;
            for (size_t j = 0; j < kSolverNodes; ++j) {
                if (i != j) node_load += _conductance[i][j];
            }
            _frame.load[i] = node_load;
        }

        // 5. If no bridges or body coupling are active, network is identity
        if (bridge_count == 0 && body_p <= 0.02f) {
            _frame.active_bridge_count = 0;
            for (size_t i = 0; i < kSolverNodes; ++i) {
                _frame.pull[i] = 0.0f;
            }
            return;
        }

        // 6. Assemble DC network and solve: (Y_dc + L) * V_dc = Y_dc * R
        SolveDC(rail_voltage);
    }

    const NetFrame& Frame() const { return _frame; }
    float Pull(NodeId id) const { return _frame.pull[id]; }
    float Load(NodeId id) const { return _frame.load[id]; }
    float Pressure(size_t id) const { return (id < kSolverNodes) ? _pressures[id] : 0.0f; }
    float Conductance(size_t i, size_t j) const {
        return (i < kSolverNodes && j < kSolverNodes) ? _conductance[i][j] : 0.0f;
    }

    bool HasLatchedBridges() const {
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                if (_latched_conductance[i][j] > 0.015f) return true;
            }
        }
        return false;
    }

    void LatchLiveBridges() {
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                if (_conductance[i][j] > 0.015f) {
                    _latched_conductance[i][j] = _conductance[i][j];
                    _latched_conductance[j][i] = _conductance[i][j];
                }
            }
            if (_pressures[i] > 0.02f) {
                _latched_pressures[i] = _pressures[i];
            }
        }
    }

    void ClearLatchedBridges() {
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _latched_pressures[i] = 0.0f;
            for (size_t j = 0; j < kSolverNodes; ++j) {
                _latched_conductance[i][j] = 0.0f;
            }
        }
    }


private:
    void SolveDC(float rail_voltage) {
        float A[kSolverNodes][kSolverNodes] = {{0.0f}};
        float B[kSolverNodes] = {0.0f};

        // Populate rest voltages and impedances
        for (size_t i = 0; i < kSolverNodes; ++i) {
            const auto& spec = NodeTable::Specs[i];
            float rest = spec.rest_dc;
            if (i == NodeVcc) rest = rail_voltage;

            float z = spec.z_dc;
            if (i == NodeVcc) {
                // Internal battery resistance increases as rail voltage drops
                z = 0.05f + 0.95f * (1.0f - rail_voltage) * (1.0f - rail_voltage);
            }
            float y = 1.0f / std::max(0.001f, z);

            float sum_g = 0.0f;
            for (size_t j = 0; j < kSolverNodes; ++j) {
                if (i != j) {
                    sum_g += _conductance[i][j];
                    A[i][j] = -_conductance[i][j];
                }
            }
            A[i][i] = y + sum_g;
            B[i]    = y * rest;
        }

        float V_dc[kSolverNodes] = {0.0f};
        if (CholeskySolveVector(A, B, V_dc)) {
            for (size_t i = 0; i < kSolverNodes; ++i) {
                const auto& spec = NodeTable::Specs[i];
                float rest = (i == NodeVcc) ? rail_voltage : spec.rest_dc;
                float diff = (V_dc[i] - rest) / 0.5f;
                if (std::isnan(diff) || std::isinf(diff)) diff = 0.0f;
                float raw_pull = std::clamp(diff, -1.0f, 1.0f);
                // Low-pass smooth the DC pull across control frames (time constant ~ 8 ms)
                _smooth_pull[i] += 0.15f * (raw_pull - _smooth_pull[i]);
                _frame.pull[i] = _smooth_pull[i];
            }
        }
    }

    // Cholesky decomposition solving A * x = b for a single vector b
    static bool CholeskySolveVector(const float A[kSolverNodes][kSolverNodes],
                                    const float b[kSolverNodes],
                                    float x[kSolverNodes]) {
        float L[kSolverNodes][kSolverNodes] = {{0.0f}};

        for (size_t i = 0; i < kSolverNodes; ++i) {
            for (size_t j = 0; j <= i; ++j) {
                float sum = 0.0f;
                for (size_t k = 0; k < j; ++k) sum += L[i][k] * L[j][k];
                if (i == j) {
                    float val = A[i][i] - sum;
                    if (val <= 1e-7f) return false;
                    L[i][j] = std::sqrt(val);
                } else {
                    L[i][j] = (A[i][j] - sum) / L[j][j];
                }
            }
        }

        float y[kSolverNodes] = {0.0f};
        for (size_t i = 0; i < kSolverNodes; ++i) {
            float sum = 0.0f;
            for (size_t k = 0; k < i; ++k) sum += L[i][k] * y[k];
            y[i] = (b[i] - sum) / L[i][i];
        }
        for (int i = static_cast<int>(kSolverNodes) - 1; i >= 0; --i) {
            float sum = 0.0f;
            for (size_t k = i + 1; k < kSolverNodes; ++k) sum += L[k][i] * x[k];
            x[i] = (y[i] - sum) / L[i][i];
        }
        return true;
    }

    float _pressures[kSolverNodes]{0.0f};
    float _latched_pressures[kSolverNodes]{0.0f};
    bool  _latch_released[kSolverNodes]{false};

    float _conductance[kSolverNodes][kSolverNodes]{{0.0f}};
    float _latched_conductance[kSolverNodes][kSolverNodes]{{0.0f}};
    float _weights[kSolverNodes][kSolverNodes]{{0.0f}};

    float _smooth_pull[kSolverNodes]{0.0f};
    bool  _is_idle{true};
    bool  _prev_is_latched{false};
    uint32_t _unlatch_timer{0};

    NetFrame _frame{};
};

} // namespace bends
