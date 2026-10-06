#pragma once

#include "circuit/node_table.h"
#include "touch/pads.h"
#include <cstdint>

namespace bends {

class Gestures {
public:
    void Reset() {
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            _prev_p[i] = 0.0f;
            _touch_duration[i] = 0;
            _tap_detected[i] = false;
            _tap_velocity[i] = 0.0f;
            _is_held[i] = false;
        }
    }

    // Call once per control frame (750 Hz)
    void Process(const synthux::Pads& pads) {
        size_t active_count = 0;
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            if (pads.Pressure(i) > 0.04f) active_count++;
        }

        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            float p = pads.Pressure(i);
            _tap_detected[i] = false;

            if (p > 0.06f) {
                if (_prev_p[i] <= 0.06f) {
                    // Touch onset: register tap
                    _tap_detected[i] = true;
                    _tap_velocity[i] = (p - _prev_p[i]) * 5.0f;
                    if (_tap_velocity[i] > 1.0f) _tap_velocity[i] = 1.0f;
                    if (_tap_velocity[i] < 0.2f) _tap_velocity[i] = p;
                    _touch_duration[i] = 1;
                } else {
                    _touch_duration[i]++;
                    if (_touch_duration[i] > 180) { // ~240 ms at 750 Hz
                        _is_held[i] = true;
                    }
                }
            } else {
                _touch_duration[i] = 0;
                _is_held[i] = false;
            }

            _prev_p[i] = p;
        }

        _single_pad_active = (active_count == 1);
    }

    inline bool OnTap(NodeId id) const {
        return (id < kPhysicalNodes) ? _tap_detected[id] : false;
    }

    inline float TapVelocity(NodeId id) const {
        return (id < kPhysicalNodes) ? _tap_velocity[id] : 0.0f;
    }

    inline bool IsHeld(NodeId id) const {
        return (id < kPhysicalNodes) ? _is_held[id] : false;
    }

    inline bool IsSolo(NodeId id) const {
        return (id < kPhysicalNodes && _single_pad_active && _touch_duration[id] > 0);
    }

private:
    float   _prev_p[kPhysicalNodes]{0.0f};
    uint16_t _touch_duration[kPhysicalNodes]{0};
    bool    _tap_detected[kPhysicalNodes]{false};
    float   _tap_velocity[kPhysicalNodes]{0.0f};
    bool    _is_held[kPhysicalNodes]{false};
    bool    _single_pad_active{false};
};

} // namespace bends
