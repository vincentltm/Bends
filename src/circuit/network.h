#pragma once

#include "circuit/node_table.h"
#include "touch/pads.h"
#include "touch/air.h"
#include <cmath>
#include <algorithm>
#include <array>

namespace bends {

// NetFrame carries the resolved circuit state from control rate (750 Hz) to audio rate (48 kHz)
struct NetFrame {
    // AC transfer matrix: T[i][j] is the fraction of node j's signal coupled into node i
    float T[kSolverNodes][kSolverNodes]{{0.0f}};
    // Retention coefficient: rho[i] = T[i][i] (1.0 = untouched, drops when loaded/shunted)
    float rho[kSolverNodes]{1.0f};
    // DC pull: normalized deviation from rest voltage in [-1.0, +1.0]
    float pull[kSolverNodes]{0.0f};
    // Total load: sum of all conductances attached to node i
    float load[kSolverNodes]{0.0f};

    float total_current_load{0.0f};
    uint8_t active_bridge_count{0};
};

struct MatrixLink {
    uint8_t dst;
    uint8_t src;
    float   gain;
};

class Network {
public:
    Network() {
        Reset();
    }

    void Reset() {
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _pressures[i] = 0.0f;
            _latched_pressures[i] = 0.0f;
            _latch_released[i] = false;
            _smooth_pull[i] = 0.0f;
            _smooth_rho[i] = 1.0f;
            for (size_t j = 0; j < kSolverNodes; ++j) {
                _conductance[i][j] = 0.0f;
                _latched_conductance[i][j] = 0.0f;
                _target_T[i][j] = (i == j) ? 1.0f : 0.0f;
                _current_T[i][j] = (i == j) ? 1.0f : 0.0f;
                _step_T[i][j] = 0.0f;
                _weights[i][j] = NodeTable::PhysicalWeight(i, j);
            }
        }
        _frame = NetFrame{};
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _frame.T[i][i] = 1.0f;
            _frame.rho[i] = 1.0f;
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

        // 1. Update pad pressures & latching
        if (is_latched) {
            for (size_t i = 0; i < kPhysicalNodes; ++i) {
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
        } else {
            for (size_t i = 0; i < kPhysicalNodes; ++i) {
                _pressures[i] = pads.Pressure(i);
                _latched_pressures[i] = _pressures[i];
                _latch_released[i] = false;
                sum_p += _pressures[i];
            }
        }

        // 2. Build pairwise conductances between physical pads
        uint8_t bridge_count = 0;
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            _conductance[i][i] = 0.0f;
            for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                float live_pi = _pressures[i];
                float live_pj = _pressures[j];
                float g = 0.0f;

                if (live_pi > 0.02f && live_pj > 0.02f) {
                    g = std::sqrt(live_pi * live_pj) * _weights[i][j] * bend_depth;
                }

                if (is_latched) {
                    float total_g = _latched_conductance[i][j] + g;
                    g = (total_g > 1.0f) ? 1.0f : total_g;
                } else {
                    _latched_conductance[i][j] = g;
                    _latched_conductance[j][i] = g;
                }

                if (g > 1.0f) g = 1.0f;
                _conductance[i][j] = g;
                _conductance[j][i] = g;

                if (g > 0.02f) {
                    bridge_count++;
                    sum_g += g;
                }
            }
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

        // Air proximity coupling (theremin gesture between OSC P03 and AMP P08)
        if (air.IsEnabled()) {
            float prox_amp = air.PadProximity(NodeAmp);
            if (prox_amp > 0.05f && _pressures[NodeOsc] > 0.02f) {
                float g_air = prox_amp * 0.35f * bend_depth;
                _conductance[NodeOsc][NodeAmp] = std::max(_conductance[NodeOsc][NodeAmp], g_air);
                _conductance[NodeAmp][NodeOsc] = _conductance[NodeOsc][NodeAmp];
                bridge_count++;
            }
        }

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
            _num_links = 0;
            for (size_t i = 0; i < kSolverNodes; ++i) {
                _frame.rho[i]  = 1.0f;
                _frame.pull[i] = 0.0f;
            }
            return;
        }

        // 6. Assemble DC network and solve: (Y_dc + L) * V_dc = Y_dc * R
        SolveDC(rail_voltage);

        // 7. Assemble AC network and solve: (Y_ac + L) * T = Y_ac
        SolveAC();

        // 8. Extract active sparse links for the audio loop
        _num_links = 0;
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _frame.rho[i] = _target_T[i][i];
            for (size_t j = 0; j < kSolverNodes; ++j) {
                if (i != j && _target_T[i][j] > 0.015f && _num_links < kMaxLinks) {
                    float w = NodeTable::Specs[j].writer_gain;
                    _links[_num_links++] = { static_cast<uint8_t>(i), static_cast<uint8_t>(j), _target_T[i][j] * w };
                }
            }
        }
    }

    // Per-sample tick: advances sparse nodal mixing (ultra-low CPU)
    inline void Tick(const float s_in[kSolverNodes],
                     float s_out[kSolverNodes],
                     float inj_out[kSolverNodes]) {
        for (size_t i = 0; i < kSolverNodes; ++i) {
            inj_out[i] = 0.0f;
        }
        for (uint8_t l = 0; l < _num_links; ++l) {
            inj_out[_links[l].dst] += _links[l].gain * s_in[_links[l].src];
        }
        for (size_t i = 0; i < kSolverNodes; ++i) {
            s_out[i] = _frame.rho[i] * s_in[i] + inj_out[i];
        }
    }

    const NetFrame& Frame() const { return _frame; }
    float Pull(NodeId id) const { return _frame.pull[id]; }
    float Rho(NodeId id) const  { return _frame.rho[id]; }
    float Load(NodeId id) const { return _frame.load[id]; }
    float Pressure(size_t id) const { return (id < kSolverNodes) ? _pressures[id] : 0.0f; }


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

    void SolveAC() {
        float A[kSolverNodes][kSolverNodes] = {{0.0f}};
        float B[kSolverNodes][kSolverNodes] = {{0.0f}};

        for (size_t i = 0; i < kSolverNodes; ++i) {
            const auto& spec = NodeTable::Specs[i];
            float y = 1.0f / std::max(0.001f, spec.z_ac);

            float sum_g = 0.0f;
            for (size_t j = 0; j < kSolverNodes; ++j) {
                if (i != j) {
                    sum_g += _conductance[i][j];
                    A[i][j] = -_conductance[i][j];
                }
            }
            A[i][i] = y + sum_g;
            B[i][i] = y; // B = Y_ac
        }

        float solved_T[kSolverNodes][kSolverNodes];
        if (CholeskySolveMatrix(A, B, solved_T)) {
            for (size_t i = 0; i < kSolverNodes; ++i) {
                float row_sum = 0.0f;
                for (size_t j = 0; j < kSolverNodes; ++j) {
                    float v = solved_T[i][j];
                    if (std::isnan(v) || std::isinf(v)) v = (i == j) ? 1.0f : 0.0f;
                    v = std::clamp(v, 0.0f, 1.0f);
                    _target_T[i][j] = v;
                    row_sum += v;
                }
                // Normalize row if needed so sum = 1
                if (row_sum > 1.05f || row_sum < 0.95f) {
                    float inv_sum = 1.0f / std::max(0.001f, row_sum);
                    for (size_t j = 0; j < kSolverNodes; ++j) {
                        _target_T[i][j] *= inv_sum;
                    }
                }
                _frame.rho[i] = _target_T[i][i];
            }
        } else {
            // Safe fallback to identity matrix
            for (size_t i = 0; i < kSolverNodes; ++i) {
                for (size_t j = 0; j < kSolverNodes; ++j) {
                    _target_T[i][j] = (i == j) ? 1.0f : 0.0f;
                }
                _frame.rho[i] = 1.0f;
            }
        }
    }

    void PrepareBlockRamp(uint16_t block_size) {
        float inv_block = 1.0f / static_cast<float>(block_size);
        for (size_t i = 0; i < kSolverNodes; ++i) {
            for (size_t j = 0; j < kSolverNodes; ++j) {
                _step_T[i][j] = (_target_T[i][j] - _current_T[i][j]) * inv_block;
            }
        }
    }

    // Cholesky decomposition of SPD matrix A (size kSolverNodes x kSolverNodes)
    // Solves A * X = B for matrix B
    static bool CholeskySolveMatrix(const float A[kSolverNodes][kSolverNodes],
                                    const float B[kSolverNodes][kSolverNodes],
                                    float X[kSolverNodes][kSolverNodes]) {
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

        for (size_t col = 0; col < kSolverNodes; ++col) {
            float y[kSolverNodes] = {0.0f};
            for (size_t i = 0; i < kSolverNodes; ++i) {
                float sum = 0.0f;
                for (size_t k = 0; k < i; ++k) sum += L[i][k] * y[k];
                y[i] = (B[i][col] - sum) / L[i][i];
            }
            for (int i = static_cast<int>(kSolverNodes) - 1; i >= 0; --i) {
                float sum = 0.0f;
                for (size_t k = i + 1; k < kSolverNodes; ++k) sum += L[k][i] * X[k][col];
                X[i][col] = (y[i] - sum) / L[i][i];
            }
        }
        return true;
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

    float _target_T[kSolverNodes][kSolverNodes]{{0.0f}};
    float _current_T[kSolverNodes][kSolverNodes]{{0.0f}};
    float _step_T[kSolverNodes][kSolverNodes]{{0.0f}};

    float _smooth_pull[kSolverNodes]{0.0f};
    float _smooth_rho[kSolverNodes]{1.0f};
    bool  _is_idle{true};

    static constexpr size_t kMaxLinks = 24;
    MatrixLink _links[kMaxLinks];
    uint8_t    _num_links{0};

    NetFrame _frame{};
};

} // namespace bends
