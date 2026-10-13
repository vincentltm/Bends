#pragma once

#include "board/nodes.h"
#include "board/contacts.h"
#include "bends/slots.h"
#include "bends/mod_pack.h"
#include <cstdint>
#include <algorithm>

namespace bends {

struct Bend {
    NodeId  a;
    NodeId  b;
    Verb    verb;
    Slot    target;
    Sig     source;
    float   depth;
    uint8_t when;
};

struct DynamicRoute {
    Slot    target;
    Sig     source;
    Verb    verb;
    float   gain;
};

class BendEngine {
public:
    static constexpr size_t kMaxDynamicRoutes = 16;

    // Run once per control block (~750 Hz)
    void Compile(const Bend* table, size_t num_bends,
                 const Contacts& contacts,
                 uint8_t active_when_flags) {
        _base_mods.Reset();
        _num_dynamic_routes = 0;
        _flush_requested = false;
        _write_inhibited = false;

        for (size_t i = 0; i < num_bends; ++i) {
            const auto& bend = table[i];

            // Condition bitmask check
            if (bend.when != When::Always) {
                if ((bend.when & active_when_flags) == 0) {
                    continue;
                }
            }

            float g = contacts.Conductance(bend.a, bend.b);
            if (g <= 0.015f) continue;

            float eff_gain = g * bend.depth;

            if (bend.verb == Verb::Flush) {
                if (eff_gain > 0.10f) _flush_requested = true;
                continue;
            }

            if (bend.verb == Verb::Inhibit) {
                if (eff_gain > 0.15f) _write_inhibited = true;
                continue;
            }

            // Static control-rate modulation (source == Sig::One)
            if (bend.source == Sig::One) {
                _base_mods.Apply(bend.target, bend.verb, eff_gain, 1.0f);
            } else {
                // Dynamic audio-rate route evaluated per-sample
                if (_num_dynamic_routes < kMaxDynamicRoutes) {
                    _dynamic_routes[_num_dynamic_routes++] = {
                        bend.target,
                        bend.source,
                        bend.verb,
                        eff_gain
                    };
                }
            }
        }
    }

    // Run per sample (48 kHz): evaluates only active audio-rate dynamic routes
    inline void AccumulateDynamic(BoardMods& mods,
                                  const float prev_node_sig[kSolverNodes][2],
                                  float osc_ramp, float osc_reset, float osc_sub, float clk_pulse) const {
        for (uint8_t r = 0; r < _num_dynamic_routes; ++r) {
            const auto& rt = _dynamic_routes[r];
            float src_val = 1.0f;
            switch (rt.source) {
                case Sig::One:        src_val = 1.0f; break;
                case Sig::Osc:        src_val = prev_node_sig[NodeOsc][0]; break;
                case Sig::OscRamp:    src_val = osc_ramp; break;
                case Sig::OscReset:   src_val = osc_reset; break;
                case Sig::OscSub:     src_val = osc_sub; break;
                case Sig::ClkPulse:   src_val = clk_pulse; break;
                case Sig::PreampOut:  src_val = prev_node_sig[NodeAmp][0]; break;
                case Sig::FltOut:     src_val = prev_node_sig[NodeFlt][0]; break;
                case Sig::DlyOut:     src_val = prev_node_sig[NodeDly][0]; break;
                case Sig::Hum:        src_val = prev_node_sig[NodeGnd][0]; break;
                case Sig::ContactMic: src_val = prev_node_sig[NodeIn][0]; break;
                case Sig::MasterOut:  src_val = prev_node_sig[NodeOut][0]; break;
            }
            mods.Apply(rt.target, rt.verb, rt.gain, src_val);
        }
    }

    const BoardMods& BaseMods() const { return _base_mods; }
    uint8_t NumDynamicRoutes() const { return _num_dynamic_routes; }
    bool FlushRequested() const { return _flush_requested; }
    bool WriteInhibited() const { return _write_inhibited; }

private:
    BoardMods    _base_mods{};
    DynamicRoute _dynamic_routes[kMaxDynamicRoutes];
    uint8_t      _num_dynamic_routes{0};
    bool         _flush_requested{false};
    bool         _write_inhibited{false};
};

} // namespace bends
