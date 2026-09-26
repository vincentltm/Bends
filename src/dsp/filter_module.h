#pragma once

#include "module.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

// Resonant state-variable filter (LP/BP) with ping input.
class FilterModule : public Module {
public:
    FilterModule() = default;

    void Init(float sample_rate, uint16_t /*block_size*/) override {
        _sample_rate = sample_rate;
        _svf.Init(sample_rate);
        _svf.SetFreq(_base_cutoff);
        _svf.SetRes(_res);
        _svf.SetDrive(_drive);
        Reset();
    }

    void Reset() override {
        _aux_in[0] = 0.0f;
        _aux_in[1] = 0.0f;
        _band_out = 0.0f;
        _current_cutoff = _base_cutoff;
        _ping_impulse = 0.0f;
    }

    void Ping(float strength) {
        _ping_impulse += daisysp::fclamp(strength, 0.0f, 1.8f);
    }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
        float oct_mod = daisysp::fclamp(_aux_in[0], -4.0f, 3.5f);
        float target_cutoff = _base_cutoff * std::pow(2.0f, oct_mod);
        
        // Limit cutoff below sample_rate / 3 for stability
        target_cutoff = daisysp::fclamp(target_cutoff, 25.0f, 15500.0f);
        _current_cutoff += 0.05f * (target_cutoff - _current_cutoff);

        float eff_res = daisysp::fclamp(_res + _aux_in[1], 0.0f, 0.95f);

        _svf.SetFreq(_current_cutoff);
        _svf.SetRes(eff_res);
        _svf.SetDrive(_drive);

        // Input soft saturation
        float in_sum = 0.5f * (in_l + in_r) + _ping_impulse;
        _ping_impulse = 0.0f;

        float sat_in = in_sum / (1.0f + 0.35f * std::abs(in_sum));

        _svf.Process(sat_in);

        float low = _svf.Low();
        float band = _svf.Band();
        _band_out = band;

        float main_out = (_mode == 1) ? band : low;

        // Output soft clip
        main_out = daisysp::SoftClip(main_out);

        out_l = main_out;
        out_r = main_out;

        _aux_in[0] = 0.0f;
        _aux_in[1] = 0.0f;
    }

    uint8_t NumParams() const override { return 4; }

    const char* GetParamName(uint8_t id) const override {
        switch (id) {
            case 0: return "Cutoff";
            case 1: return "Res";
            case 2: return "Drive";
            case 3: return "Mode";
            default: return "";
        }
    }

    void SetParam(uint8_t id, float value) override {
        switch (id) {
            case 0:
                _base_cutoff = daisysp::fclamp(value, 30.0f, 15000.0f);
                break;
            case 1:
                _res = daisysp::fclamp(value, 0.0f, 0.95f);
                break;
            case 2:
                _drive = daisysp::fclamp(value, 0.0f, 1.0f);
                break;
            case 3:
                _mode = (value >= 0.5f) ? 1 : 0;
                break;
        }
    }

    float GetParam(uint8_t id) const override {
        switch (id) {
            case 0: return _base_cutoff;
            case 1: return _res;
            case 2: return _drive;
            case 3: return static_cast<float>(_mode);
            default: return 0.0f;
        }
    }

    void SetAuxInput(uint8_t channel, float value) override {
        if (channel < 2) {
            _aux_in[channel] += value;
        }
    }

    float GetAuxOutput(uint8_t channel) const override {
        return (channel == 0) ? _band_out : 0.0f;
    }

    const char* Name() const override { return "Filter"; }

private:
    float _sample_rate{48000.0f};
    daisysp::Svf _svf;
    float _base_cutoff{800.0f};
    float _current_cutoff{800.0f};
    float _res{0.3f};
    float _drive{0.2f};
    uint8_t _mode{0}; // 0 = Lowpass, 1 = Bandpass
    float _aux_in[2]{0.0f, 0.0f};
    float _band_out{0.0f};
    float _ping_impulse{0.0f};
};

} // namespace bends
