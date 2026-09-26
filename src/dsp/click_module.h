#pragma once

#include "module.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

// Click and pulse generator (heartbeat / transient kick).
class ClickModule : public Module {
public:
    ClickModule() = default;

    void Init(float sample_rate, uint16_t /*block_size*/) override {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() override {
        _pulse_phase = 0.0f;
        _poly_phase = 0.0f;
        _impulse_decay = 0.0f;
        _impulse_phase = 0.0f;
        _rng = 424242u;
        _pressure = 0.0f;
        _pitch = 0.5f;
        _starve = 0.0f;
        _drive = 0.2f;
        _poly_mod = 0.0f;
    }

    // Trigger transient click on pad tap
    void Strike(float velocity) {
        _impulse_decay = daisysp::fclamp(velocity * 1.6f, 0.45f, 2.0f);
        _impulse_phase = 0.0f;
    }

    void SetPolyMod(float val) {
        _poly_mod = daisysp::fclamp(val, 0.0f, 1.0f);
    }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
        float out = 0.0f;

        // Transient kick / click: base pitch D1 (36.7 Hz) to D5 (587.3 Hz)
        if (_impulse_decay > 0.001f) {
            float pitch_dive = 1.0f + _impulse_decay * 0.65f;
            float pitch_curve = _pitch * _pitch;
            float base_f = 36.71f * std::pow(2.0f, pitch_curve * 4.0f);
            float eff_freq = base_f * pitch_dive;
            float phase_inc = (eff_freq * 6.2831853f) / _sample_rate;
            _impulse_phase += phase_inc;
            out += std::sin(_impulse_phase) * _impulse_decay * 1.05f;
            float decay_rate = 0.992f + (1.0f - _pitch) * 0.0055f;
            _impulse_decay *= decay_rate;
        } else {
            _impulse_phase = 0.0f;
        }

        // Repeating pulses when pad is held
        if (_pressure > 0.03f) {
            float norm_p = daisysp::fclamp((_pressure - 0.03f) / 0.97f, 0.0f, 1.0f);
            float pitch_curve = _pitch * _pitch;
            float min_rate = 0.06f + pitch_curve * 0.80f;
            float pressure_accel = norm_p * norm_p * norm_p;
            float base_rate = min_rate + pressure_accel * (5.0f + _pitch * 26.0f);

            // Jitter when starved
            _rng = _rng * 1664525u + 1013904223u;
            float jitter = (static_cast<float>(static_cast<int32_t>(_rng)) * 2.3283e-10f) * (_starve * 0.45f);
            float eff_rate = base_rate * (1.0f + jitter);

            _pulse_phase += (eff_rate / _sample_rate);
            if (_pulse_phase >= 1.0f) {
                _pulse_phase -= 1.0f;
                _impulse_decay = std::max(_impulse_decay, 0.60f + norm_p * 0.60f);
                _impulse_phase = 0.0f;
            }

            // 3:2 polyrhythm when bridged to other pads
            if (_poly_mod > 0.05f) {
                _poly_phase += (base_rate * 1.5f / _sample_rate);
                if (_poly_phase >= 1.0f) {
                    _poly_phase -= 1.0f;
                    _impulse_decay = std::max(_impulse_decay, 0.45f * _poly_mod);
                    _impulse_phase = 0.0f;
                }
            } else {
                _poly_phase = 0.0f;
            }
        } else {
            _pulse_phase = 0.0f;
            _poly_phase = 0.0f;
        }

        // Drive scaling
        float scaled = out * daisysp::fclamp(0.25f + _drive * 1.5f, 0.0f, 2.0f);
        out_l = scaled;
        out_r = scaled;
    }

    uint8_t NumParams() const override { return 5; }

    const char* GetParamName(uint8_t id) const override {
        switch (id) {
            case 0: return "Pressure";
            case 1: return "Pitch";
            case 2: return "Starve";
            case 3: return "Drive";
            case 4: return "Poly";
            default: return "";
        }
    }

    void SetParam(uint8_t id, float value) override {
        switch (id) {
            case 0: _pressure = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 1: _pitch    = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 2: _starve   = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 3: _drive    = daisysp::fclamp(value, 0.0f, 2.0f); break;
            case 4: _poly_mod = daisysp::fclamp(value, 0.0f, 1.0f); break;
        }
    }

    float GetParam(uint8_t id) const override {
        switch (id) {
            case 0: return _pressure;
            case 1: return _pitch;
            case 2: return _starve;
            case 3: return _drive;
            case 4: return _poly_mod;
            default: return 0.0f;
        }
    }

    void SetAuxInput(uint8_t channel, float value) override {
        if (channel == 0) _poly_mod = daisysp::fclamp(value, 0.0f, 1.0f);
    }

    const char* Name() const override { return "Click"; }

private:
    float _sample_rate{48000.0f};
    float _pulse_phase{0.0f};
    float _poly_phase{0.0f};
    float _impulse_decay{0.0f};
    float _impulse_phase{0.0f};
    uint32_t _rng{424242u};

    float _pressure{0.0f};
    float _pitch{0.5f};
    float _starve{0.0f};
    float _drive{0.2f};
    float _poly_mod{0.0f};
};

} // namespace bends
