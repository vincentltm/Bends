#pragma once

#include <cmath>

namespace synthux {

/**
 * Direct Pickup on Move (Shift knob handler)
 * 
 * When holding a shift button (like Pad 10) to adjust a secondary parameter:
 * - Keeps the parameter at its last stored value so it doesn't jump.
 * - Latches the knob position when shift is pressed.
 * - Only starts tracking the knob once you turn it past the deadband threshold (~2%).
 */
class MValue {
public:
    MValue(float default_val = 0.0f, float threshold = 0.02f) :
        _value(default_val),
        _init_knob(0.0f),
        _threshold(threshold),
        _active(false),
        _tracking(false) {}

    float Process(float knob_val, bool active) {
        // Shift layer activated
        if (active && !_active) {
            _init_knob = knob_val;
            _tracking = false;
        } else if (!active && _active) {
            _tracking = false;
        }
        _active = active;

        if (!_active) return _value;

        // Wait for user to move knob beyond threshold before picking up
        if (!_tracking) {
            if (std::fabs(knob_val - _init_knob) > _threshold) {
                _tracking = true;
                _value = knob_val;
            }
        } else {
            _value = knob_val;
        }
        return _value;
    }

    float Value() const { return _value; }
    void Set(float val) { _value = val; _tracking = false; }
    bool IsTracking() const { return _tracking; }

private:
    float _value;
    float _init_knob;
    float _threshold;
    bool _active;
    bool _tracking;
};

} // namespace synthux
