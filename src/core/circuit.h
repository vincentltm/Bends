#pragma once

#include "module.h"
#include "connection.h"
#include <cstdint>
#include <cstddef>
#include <algorithm>

namespace bends {

constexpr uint8_t kMaxModules     = 16;
constexpr uint8_t kMaxConnections = 32;
constexpr uint8_t kMaxFeedback    = 8;
constexpr uint8_t kInvalidModule  = 255;

// Audio graph execution engine.
//
// Manages a patch of interconnected DSP modules. Arbitrary bridging on the
// physical surface frequently forms feedback loops (e.g. delay output fed back into
// filter input, or resonance self-modulation). In digital signal processing,
// delay-free loops cannot be computed directly without solving implicit systems.
//
// This engine resolves feedback cycles by inserting a 1-sample unit delay (z^-1)
// on all back-edges identified during topological sort. Processing is interleaved
// sample-by-sample so that feedback paths have true single-sample latency at the
// audio rate (48 kHz), preserving tight feedback characteristics.
//
// References:
// - G. Borin, G. De Poli, D. Rocchesso, "Elimination of delay-free loops in discrete-time models" (2000)
//   https://doi.org/10.1109/89.861380
// - A.B. Kahn, "Topological sorting of large networks" (1962)
//   https://doi.org/10.1145/368996.369025
class Circuit {
public:
    Circuit() {
        for (uint8_t i = 0; i < kMaxModules; ++i) {
            _modules[i] = nullptr;
            _exec_order[i] = i;
        }
        Reset();
    }

    void Init(float sample_rate, uint16_t block_size) {
        _sample_rate = sample_rate;
        _block_size  = block_size;
        for (uint8_t i = 0; i < kMaxModules; ++i) {
            if (_modules[i]) {
                _modules[i]->Init(sample_rate, block_size);
            }
        }
    }

    void Reset() {
        for (uint8_t i = 0; i < kMaxFeedback; ++i) {
            _fb_buffers[i][0] = 0.0f;
            _fb_buffers[i][1] = 0.0f;
        }
        for (uint8_t i = 0; i < kMaxModules; ++i) {
            if (_modules[i]) {
                _modules[i]->Reset();
            }
        }
    }

    void RegisterModule(uint8_t id, Module* module) {
        if (id < kMaxModules) {
            _modules[id] = module;
            if (module && _sample_rate > 0.0f) {
                module->Init(_sample_rate, _block_size);
            }
        }
    }

    Module* GetModule(uint8_t id) const {
        return (id < kMaxModules) ? _modules[id] : nullptr;
    }

    bool Connect(const Connection& conn) {
        if (_num_connections >= kMaxConnections) return false;
        _connections[_num_connections++] = conn;
        return true;
    }

    void ClearConnections() {
        _num_connections = 0;
        _num_feedback = 0;
    }

    void SetParam(uint8_t module_id, uint8_t param_id, float value) {
        if (module_id < kMaxModules && _modules[module_id]) {
            _modules[module_id]->SetParam(param_id, value);
        }
    }

    float GetParam(uint8_t module_id, uint8_t param_id) const {
        if (module_id < kMaxModules && _modules[module_id]) {
            return _modules[module_id]->GetParam(param_id);
        }
        return 0.0f;
    }

    void SetConnectionGain(uint8_t conn_idx, float gain) {
        if (conn_idx < _num_connections) {
            _connections[conn_idx].gain = gain;
        }
    }

    // Resolves execution order using Kahn's topological sorting algorithm.
    //
    // Modules with zero in-degree are scheduled first. When a feedback cycle
    // is detected (edges pointing backward in the execution order, or self-loops),
    // those connections are tagged with `is_feedback = true` and allocated to
    // a unit-delay buffer (_fb_buffers).
    void Compile() {
        _num_feedback = 0;
        _num_exec_modules = 0;

        // Collect registered active modules
        uint8_t active[kMaxModules];
        uint8_t num_active = 0;
        for (uint8_t i = 0; i < kMaxModules; ++i) {
            if (_modules[i]) {
                active[num_active++] = i;
            }
        }

        // Detect feedback loops using a simple adjacency matrix & in-degree Kahn's algorithm
        uint8_t in_degree[kMaxModules] = {0};
        bool adj[kMaxModules][kMaxModules] = {{false}};

        for (uint8_t c = 0; c < _num_connections; ++c) {
            uint8_t u = _connections[c].src_module;
            uint8_t v = _connections[c].dst_module;
            if (u < kMaxModules && v < kMaxModules && _modules[u] && _modules[v] && u != v) {
                if (!adj[u][v]) {
                    adj[u][v] = true;
                    in_degree[v]++;
                }
            }
        }

        // Kahn's algorithm queue
        uint8_t q[kMaxModules];
        uint8_t q_head = 0, q_tail = 0;

        for (uint8_t i = 0; i < num_active; ++i) {
            uint8_t mod = active[i];
            if (in_degree[mod] == 0) {
                q[q_tail++] = mod;
            }
        }

        while (q_head < q_tail) {
            uint8_t u = q[q_head++];
            _exec_order[_num_exec_modules++] = u;

            for (uint8_t i = 0; i < num_active; ++i) {
                uint8_t v = active[i];
                if (adj[u][v]) {
                    if (--in_degree[v] == 0) {
                        q[q_tail++] = v;
                    }
                }
            }
        }

        // If cycles exist, append any remaining active modules and mark their back-edges as feedback
        for (uint8_t i = 0; i < num_active; ++i) {
            uint8_t mod = active[i];
            bool already_added = false;
            for (uint8_t k = 0; k < _num_exec_modules; ++k) {
                if (_exec_order[k] == mod) {
                    already_added = true;
                    break;
                }
            }
            if (!already_added) {
                _exec_order[_num_exec_modules++] = mod;
            }
        }

        // Mark connections that go backwards in execution order as feedback
        for (uint8_t c = 0; c < _num_connections; ++c) {
            uint8_t u = _connections[c].src_module;
            uint8_t v = _connections[c].dst_module;
            
            // Self-loop is always feedback
            if (u == v) {
                _connections[c].is_feedback = true;
            } else {
                int8_t order_u = -1, order_v = -1;
                for (uint8_t k = 0; k < _num_exec_modules; ++k) {
                    if (_exec_order[k] == u) order_u = k;
                    if (_exec_order[k] == v) order_v = k;
                }
                if (order_u >= order_v) {
                    _connections[c].is_feedback = true;
                }
            }

            if (_connections[c].is_feedback && _num_feedback < kMaxFeedback) {
                _fb_wire_indices[_num_feedback++] = c;
            }
        }

        // Root modules have no feedforward incoming connections
        for (uint8_t i = 0; i < kMaxModules; ++i) {
            _is_root[i] = true;
        }
        for (uint8_t c = 0; c < _num_connections; ++c) {
            if (!_connections[c].is_feedback) {
                uint8_t v = _connections[c].dst_module;
                if (v < kMaxModules) {
                    _is_root[v] = false;
                }
            }
        }
    }

    // Runs audio processing over the block with sample-level interleaving.
    //
    // For each sample tick:
    // 1. Module inputs are seeded with feedback from sample (t - 1).
    // 2. Feedforward connections transfer audio within the current sample tick.
    // 3. New feedback state is captured for sample (t + 1).
    void ProcessBlock(const float* in_l, const float* in_r,
                      float* out_l, float* out_r,
                      uint16_t block_size) {
        float mod_out_l[kMaxModules] = {0.0f};
        float mod_out_r[kMaxModules] = {0.0f};

        for (uint16_t s = 0; s < block_size; ++s) {
            float sample_in_l = in_l ? in_l[s] : 0.0f;
            float sample_in_r = in_r ? in_r[s] : 0.0f;

            // Clear module inputs for this sample
            float mod_in_l[kMaxModules] = {0.0f};
            float mod_in_r[kMaxModules] = {0.0f};

            // Read feedback values from previous sample
            float current_fb[kMaxFeedback][2];
            for (uint8_t f = 0; f < _num_feedback; ++f) {
                current_fb[f][0] = _fb_buffers[f][0];
                current_fb[f][1] = _fb_buffers[f][1];
            }

            // Distribute feedback connections
            for (uint8_t f = 0; f < _num_feedback; ++f) {
                uint8_t c = _fb_wire_indices[f];
                uint8_t dst = _connections[c].dst_module;
                float gain = _connections[c].gain;

                if (_connections[c].dst_port == kPortMainL) {
                    mod_in_l[dst] += current_fb[f][0] * gain;
                } else if (_connections[c].dst_port == kPortMainR) {
                    mod_in_r[dst] += current_fb[f][1] * gain;
                } else {
                    if (_modules[dst]) {
                        _modules[dst]->SetAuxInput(_connections[c].dst_port - kPortAux0, current_fb[f][0] * gain);
                    }
                }
            }

            // Route non-feedback connections and process modules in dependency order
            for (uint8_t m = 0; m < _num_exec_modules; ++m) {
                uint8_t u = _exec_order[m];
                Module* mod = _modules[u];
                if (!mod) continue;

                // Inject circuit input into root modules (modules with no incoming connections)
                if (_is_root[u] && mod_in_l[u] == 0.0f && mod_in_r[u] == 0.0f) {
                    mod_in_l[u] = sample_in_l;
                    mod_in_r[u] = sample_in_r;
                }

                // Process module
                mod->Process(mod_in_l[u], mod_in_r[u], mod_out_l[u], mod_out_r[u]);

                // Distribute forward connections from this module
                for (uint8_t c = 0; c < _num_connections; ++c) {
                    if (_connections[c].src_module == u && !_connections[c].is_feedback) {
                        uint8_t dst = _connections[c].dst_module;
                        float gain  = _connections[c].gain;
                        float sig_l = (_connections[c].src_port == kPortMainR) ? mod_out_r[u] : mod_out_l[u];
                        float sig_r = mod_out_r[u];

                        if (_connections[c].src_port >= kPortAux0) {
                            sig_l = mod->GetAuxOutput(_connections[c].src_port - kPortAux0);
                            sig_r = sig_l;
                        }

                        if (_connections[c].dst_port == kPortMainL) {
                            mod_in_l[dst] += sig_l * gain;
                        } else if (_connections[c].dst_port == kPortMainR) {
                            mod_in_r[dst] += sig_r * gain;
                        } else {
                            if (_modules[dst]) {
                                _modules[dst]->SetAuxInput(_connections[c].dst_port - kPortAux0, sig_l * gain);
                            }
                        }
                    }
                }
            }

            // Store new feedback values for next sample
            for (uint8_t f = 0; f < _num_feedback; ++f) {
                uint8_t c = _fb_wire_indices[f];
                uint8_t u = _connections[c].src_module;
                if (_connections[c].src_port >= kPortAux0 && _modules[u]) {
                    float aux = _modules[u]->GetAuxOutput(_connections[c].src_port - kPortAux0);
                    _fb_buffers[f][0] = aux;
                    _fb_buffers[f][1] = aux;
                } else {
                    _fb_buffers[f][0] = mod_out_l[u];
                    _fb_buffers[f][1] = mod_out_r[u];
                }
            }

            // Write circuit output from last module in execution order
            if (_num_exec_modules > 0) {
                uint8_t last = _exec_order[_num_exec_modules - 1];
                out_l[s] = mod_out_l[last];
                out_r[s] = mod_out_r[last];
            } else {
                out_l[s] = sample_in_l;
                out_r[s] = sample_in_r;
            }
        }
    }

private:
    float      _sample_rate = 48000.0f;
    uint16_t   _block_size  = 64;

    Module*    _modules[kMaxModules] = {nullptr};
    Connection _connections[kMaxConnections] = {};
    uint8_t    _num_connections = 0;

    uint8_t    _exec_order[kMaxModules] = {0};
    uint8_t    _num_exec_modules = 0;

    float      _fb_buffers[kMaxFeedback][2] = {{0.0f}};
    uint8_t    _fb_wire_indices[kMaxFeedback] = {0};
    uint8_t    _num_feedback = 0;

    bool       _is_root[kMaxModules] = {true};
};

} // namespace bends
