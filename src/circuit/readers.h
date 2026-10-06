#pragma once

#include "circuit/node_table.h"
#include <cmath>

namespace bends {

// NodeReader processes injected signals for each node:
// splits inj into LF (< 20 Hz) and HF (>= 20 Hz) components via 1-pole crossover
class NodeReader {
public:
    void Init(float sample_rate) {
        // Crossover cutoff at ~20 Hz
        _alpha = 1.0f - std::exp(-6.2831853f * 20.0f / sample_rate);
        Reset();
    }

    void Reset() {
        for (size_t i = 0; i < kSolverNodes; ++i) {
            _lp_state[i] = 0.0f;
            _inj_lf[i]   = 0.0f;
            _inj_hf[i]   = 0.0f;
        }
    }

    // Call once per sample after Network::Tick()
    inline void Process(const float inj_in[kSolverNodes]) {
        for (size_t i = 0; i < kSolverNodes; ++i) {
            float in = inj_in[i];
            _lp_state[i] += _alpha * (in - _lp_state[i]);
            _inj_lf[i]    = _lp_state[i];
            _inj_hf[i]    = in - _lp_state[i];
        }
    }

    inline float InjLF(NodeId id) const { return _inj_lf[id]; }
    inline float InjHF(NodeId id) const { return _inj_hf[id]; }

    // Audio injection into block input
    inline float AudioInject(NodeId id) const {
        return NodeTable::Readers[id].audio_inject_gain * _inj_hf[id];
    }

    // Low-frequency parameter modulation (sweeps, LFO, stepped pulses)
    inline float LFMod(NodeId id) const {
        return NodeTable::Readers[id].lf_mod_gain * _inj_lf[id];
    }

    // Audio-rate parameter modulation (FM, audio sidebands)
    inline float HFMod(NodeId id) const {
        return NodeTable::Readers[id].hf_mod_gain * _inj_hf[id];
    }

private:
    float _alpha{0.002618f};
    float _lp_state[kSolverNodes]{0.0f};
    float _inj_lf[kSolverNodes]{0.0f};
    float _inj_hf[kSolverNodes]{0.0f};
};

} // namespace bends
