#pragma once

#include <cmath>
#include <algorithm>

namespace bends {

class Rail {
public:
    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() {
        _fader_vcc = 1.0f;
        _cap_voltage = 1.0f;
        _effective_vcc = 1.0f;
        _is_brownout = false;
        _motorboat_phase = 0.0f;
        _motorboat_freq = 4.0f;
        _current_load = 0.0f;
    }

    // Call at control rate (~750 Hz)
    void ProcessControl(float fader_pos, float external_load, float network_vcc_pull) {
        _fader_vcc = std::clamp(fader_pos, 0.02f, 1.0f);
        _current_load = external_load;

        // Internal battery resistance increases quadratically as fader drops
        float starve_factor = 1.0f - _fader_vcc;
        float r_internal = 0.05f + 0.90f * starve_factor * starve_factor;

        // Target voltage accounts for fader, current draw, and DC pull from network (e.g. VCC-GND short)
        float v_target = _fader_vcc - r_internal * external_load + network_vcc_pull * 0.5f;
        v_target = std::clamp(v_target, 0.02f, 1.0f);

        // Asymmetric reservoir capacitor: fast discharge on heavy draw, slow recovery
        float alpha = (v_target < _cap_voltage) ? 0.06f : 0.008f;
        _cap_voltage += alpha * (v_target - _cap_voltage);

        // Brownout hysteresis comparator
        if (!_is_brownout && _cap_voltage < 0.12f) {
            _is_brownout = true;
        } else if (_is_brownout && _cap_voltage > 0.25f) {
            _is_brownout = false;
        }

        // Motorboat relaxation frequency scales with starve depth below 0.20
        if (_cap_voltage < 0.20f) {
            _motorboat_freq = 2.0f + 12.0f * (0.20f - _cap_voltage);
        }
    }

    // Call per sample (48 kHz) with AC ripple from NodeVcc
    inline float Tick(float ac_ripple) {
        if (_is_brownout) {
            // In deep brownout, motorboating relaxation pulses dominate
            _motorboat_phase += _motorboat_freq / _sample_rate;
            if (_motorboat_phase >= 1.0f) _motorboat_phase -= 1.0f;
            float pulse = (_motorboat_phase < 0.12f) ? 0.25f : 0.02f;
            _effective_vcc = pulse;
            return _effective_vcc;
        }

        // Starvation rail ripple (e.g. OSC bridged to VCC makes the whole circuit breathe)
        float ripple_scale = (1.0f - _cap_voltage) * 0.45f;
        _effective_vcc = std::clamp(_cap_voltage + ac_ripple * ripple_scale, 0.02f, 1.0f);
        return _effective_vcc;
    }

    float EffectiveVcc() const { return _effective_vcc; }
    float CapVoltage() const   { return _cap_voltage; }
    bool  IsBrownout() const   { return _is_brownout; }
    float FaderVcc() const     { return _fader_vcc; }

private:
    float _sample_rate{48000.0f};
    float _fader_vcc{1.0f};
    float _cap_voltage{1.0f};
    float _effective_vcc{1.0f};
    float _current_load{0.0f};
    bool  _is_brownout{false};

    float _motorboat_phase{0.0f};
    float _motorboat_freq{4.0f};
};

} // namespace bends
