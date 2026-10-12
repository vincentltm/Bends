#pragma once

#include "board/nodes.h"
#include "board/contacts.h"
#include <cmath>
#include <algorithm>

namespace bends {

struct SolverFrame {
    // Normalized DC deviation from rest in [-1.0, +1.0]
    float pull[kSolverNodes]{0.0f};

    // Node retention (fraction of own signal surviving; 1.0 = open, 0.0 = grounded)
    float retention[kSolverNodes]{1.0f};

    // AC cross-coupling matrix from nodal solve (sparse)
    float T_ac[kSolverNodes][kSolverNodes]{{0.0f}};
};

class Solver {
public:
    Solver() {
        Reset();
    }

    void Reset() {
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _smooth_pull[i] = 0.0f;
            _frame.pull[i] = 0.0f;
            _frame.retention[i] = 1.0f;
            for (size_t j = 0; j < kSolverNodes; ++j) {
                _frame.T_ac[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }

    // Runs once per control block (~750 Hz)
    void Solve(const Contacts& contacts, float rail_voltage) {
        // 1. Retention & physical AC conductance
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _frame.retention[i] = 1.0f - contacts.Conductance(i, NodeGnd) * 0.98f;
        }

        uint8_t num_bridges = contacts.NumActiveBridges();
        const auto* bridges = contacts.ActiveBridges();

        // If no bridges or body coupling are active, rest voltages
        if (num_bridges == 0 && contacts.Pressure(NodeBody) <= 0.02f) {
            for (size_t i = 0; i < kSolverNodes; ++i) {
                _smooth_pull[i] += 0.20f * (0.0f - _smooth_pull[i]);
                _frame.pull[i] = _smooth_pull[i];
            }
            return;
        }

        // Setup nodal resting DC voltages and stiffnesses
        float v_rest[kSolverNodes];
        float g_stiff[kSolverNodes];

        for (size_t i = 0; i < kSolverNodes; ++i) {
            const auto& spec = NodeTable::Specs[i];
            float rest = spec.rest_dc;
            float z = spec.z_dc;
            if (i == NodeVcc) {
                float starve_factor = 1.0f - rail_voltage;
                z = 0.05f + 0.95f * starve_factor * starve_factor;
                rest = rail_voltage;
            }
            v_rest[i] = rest;
            g_stiff[i] = 1.0f / std::max(0.001f, z);
        }

        // Seed target voltages with rest values
        float v_target[kSolverNodes];
        for (size_t i = 0; i < kSolverNodes; ++i) {
            v_target[i] = v_rest[i];
        }

        // 2-pass Jacobi DC relaxation over active bridges
        for (int pass = 0; pass < 2; ++pass) {
            float sum_gv[kSolverNodes];
            float sum_g[kSolverNodes];

            for (size_t i = 0; i < kSolverNodes; ++i) {
                sum_gv[i] = g_stiff[i] * v_rest[i];
                sum_g[i]  = g_stiff[i];
            }

            for (uint8_t k = 0; k < num_bridges; ++k) {
                uint8_t a = bridges[k].a;
                uint8_t b = bridges[k].b;
                float g = bridges[k].g;

                // Functional parameter nodes (OSC, DLY, CLK, MIX, FBK, RST) operate via dedicated DSP
                // modulations and must NOT have their resting DC knob positions pulled by passive bridges
                if (a == NodeOsc || b == NodeOsc ||
                    a == NodeDly || b == NodeDly ||
                    a == NodeClk || b == NodeClk ||
                    a == NodeMix || b == NodeMix ||
                    a == NodeFbk || b == NodeFbk ||
                    a == NodeRst || b == NodeRst) {
                    continue;
                }

                // DC relaxation applies strictly to power rails (VCC, GND) and starve loads
                bool a_is_dc_rail = (a == NodeVcc || a == NodeGnd || b == NodeVcc || b == NodeGnd);
                bool a_is_dc_load = ((a == NodeFlt && b == NodeAmp) || (a == NodeAmp && b == NodeFlt));
                if (!a_is_dc_rail && !a_is_dc_load) {
                    continue;
                }

                if (a < kSolverNodes && b < kSolverNodes) {
                    sum_gv[a] += g * v_target[b];
                    sum_g[a]  += g;
                    sum_gv[b] += g * v_target[a];
                    sum_g[b]  += g;
                }
            }

            for (size_t i = 0; i < kSolverNodes; ++i) {
                v_target[i] = sum_gv[i] / sum_g[i];
            }
        }

        for (size_t i = 0; i < kSolverNodes; ++i) {
            const auto& spec = NodeTable::Specs[i];
            float rest = (i == NodeVcc) ? rail_voltage : spec.rest_dc;
            float diff = (v_target[i] - rest) / 0.5f;
            if (std::isnan(diff) || std::isinf(diff)) diff = 0.0f;
            float raw_pull = std::clamp(diff, -1.0f, 1.0f);
            _smooth_pull[i] += 0.20f * (raw_pull - _smooth_pull[i]);
            _frame.pull[i] = _smooth_pull[i];
        }
    }

    const SolverFrame& Frame() const { return _frame; }
    float Pull(NodeId id) const { return _frame.pull[id]; }
    float Retention(NodeId id) const { return _frame.retention[id]; }
    float AC(size_t i, size_t j) const { return _frame.T_ac[i][j]; }

private:
    float _smooth_pull[kSolverNodes]{0.0f};
    SolverFrame _frame{};
};

} // namespace bends
