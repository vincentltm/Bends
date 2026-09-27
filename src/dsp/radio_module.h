#pragma once

#include "module.h"
#include "daisysp.h"
#include <cmath>

namespace bends {

// High-frequency radio whine and heterodyne beat.
class RadioModule : public Module {
public:
    RadioModule() = default;

    void Init(float sample_rate, uint16_t /*block_size*/) override {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() override {
        _rf_phase = 0.0f;
        _sub_phase = 0.0f;
        _rf_freq = 1200.0f;
        _rf_phase_inc = _rf_freq * 6.2831853f / _sample_rate;
        _whine_amp = 0.0f;
    }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
        float out = 0.0f;
        if (_whine_amp > 0.0005f) {
            _rf_phase += _rf_phase_inc;
            if (_rf_phase >= 6.2831853f) _rf_phase -= 6.2831853f;
            _sub_phase += _rf_phase_inc * 0.5015f;
            if (_sub_phase >= 6.2831853f) _sub_phase -= 6.2831853f;
            float main_c = sinf(_rf_phase);
            float sub_c  = sinf(_sub_phase);
            float raw_rf = main_c * 0.65f + (main_c * sub_c) * 0.70f;
            out = (raw_rf / (1.0f + 0.4f * std::abs(raw_rf))) * _whine_amp;
        }

        out_l = out;
        out_r = out;
    }

    uint8_t NumParams() const override { return 2; }

    const char* GetParamName(uint8_t id) const override {
        switch (id) {
            case 0: return "Freq";
            case 1: return "Amp";
            default: return "";
        }
    }

    void SetParam(uint8_t id, float value) override {
        switch (id) {
            case 0:
                _rf_freq = daisysp::fclamp(value, 40.0f, 14000.0f);
                _rf_phase_inc = (6.2831853f * _rf_freq) / _sample_rate;
                break;
            case 1:
                _whine_amp = daisysp::fclamp(value, 0.0f, 0.10f);
                break;
        }
    }

    float GetParam(uint8_t id) const override {
        switch (id) {
            case 0: return _rf_freq;
            case 1: return _whine_amp;
            default: return 0.0f;
        }
    }

    const char* Name() const override { return "Radio"; }

private:
    float _sample_rate{48000.0f};
    float _rf_freq{1200.0f};
    float _rf_phase_inc{0.15707963f};
    float _rf_phase{0.0f};
    float _sub_phase{0.0f};
    float _whine_amp{0.0f};
};

} // namespace bends
