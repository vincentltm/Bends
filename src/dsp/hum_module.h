#pragma once

#include "module.h"
#include "daisysp.h"
#include <cmath>

namespace bends {

// 50Hz mains hum and crackle noise.
class HumModule : public Module {
public:
    HumModule() = default;

    void Init(float sample_rate, uint16_t /*block_size*/) override {
        _sample_rate = sample_rate;
        _phase_50hz = 0.0f;
        _phase_inc_50hz = (2.0f * 3.14159265f * 50.0f) / _sample_rate;
        Reset();
    }

    void Reset() override {
        _rng_state = 22222;
        _crackle_filter = 0.0f;
        _smooth_input = 0.0f;
        _smooth_gnd = 0.0f;
        _buzz = 0.0f;
        _hum = 0.0f;
        _drive = 0.2f;
        _starve = 0.0f;
    }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
        _smooth_input += (_buzz - _smooth_input) * 0.01f;
        _smooth_gnd   += (_hum - _smooth_gnd) * 0.01f;

        float out = 0.0f;
        if (_smooth_input > 0.0001f || _smooth_gnd > 0.0001f) {
            // 50Hz phase accumulator
            _phase_50hz += _phase_inc_50hz;
            if (_phase_50hz >= 2.0f * 3.14159265f) {
                _phase_50hz -= 2.0f * 3.14159265f;
            }

            float sin1 = sinf(_phase_50hz);
            float cos1 = cosf(_phase_50hz);
            float sin2 = 2.0f * sin1 * cos1;
            float cos1_sq = cos1 * cos1;
            float sin3 = sin1 * (4.0f * cos1_sq - 1.0f);
            float sin1_sq = sin1 * sin1;
            float sin5 = sin1 * (sin1_sq * (16.0f * sin1_sq - 20.0f) + 5.0f);

            // Crackle noise bursts
            _rng_state = _rng_state * 1664525u + 1013904223u;
            float white = static_cast<float>(static_cast<int32_t>(_rng_state)) * 4.6566129e-10f;
            float crackle = (white > 0.95f) ? (white - 0.95f) * 12.0f : 0.0f;
            if (white < -0.97f) crackle = (white + 0.97f) * 9.0f;
            _crackle_filter += (crackle - _crackle_filter) * 0.30f;

            // P03: harmonics and crackle
            float p03_buzz_sig = (sin2 * 0.35f + sin3 * 0.30f + sin5 * 0.18f + _crackle_filter * 0.35f) * _smooth_input;

            // P08: 50Hz bass hum
            float p08_gnd_sig = (sin1 * 0.85f + sin2 * 0.15f) * _smooth_gnd;

            out = p03_buzz_sig + p08_gnd_sig;
        }

        out_l = out;
        out_r = out;
    }

    uint8_t NumParams() const override { return 4; }

    const char* GetParamName(uint8_t id) const override {
        switch (id) {
            case 0: return "Buzz";
            case 1: return "Hum";
            case 2: return "Drive";
            case 3: return "Starve";
            default: return "";
        }
    }

    void SetParam(uint8_t id, float value) override {
        switch (id) {
            case 0: _buzz   = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 1: _hum    = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 2: _drive  = daisysp::fclamp(value, 0.0f, 2.0f); break;
            case 3: _starve = daisysp::fclamp(value, 0.0f, 1.0f); break;
        }
    }

    float GetParam(uint8_t id) const override {
        switch (id) {
            case 0: return _buzz;
            case 1: return _hum;
            case 2: return _drive;
            case 3: return _starve;
            default: return 0.0f;
        }
    }

    const char* Name() const override { return "Hum"; }

private:
    float    _sample_rate{48000.0f};
    float    _phase_50hz{0.0f};
    float    _phase_inc_50hz{0.0f};
    uint32_t _rng_state{22222};
    float    _crackle_filter{0.0f};
    float    _smooth_input{0.0f};
    float    _smooth_gnd{0.0f};

    float    _buzz{0.0f};
    float    _hum{0.0f};
    float    _drive{0.2f};
    float    _starve{0.0f};
};

} // namespace bends
