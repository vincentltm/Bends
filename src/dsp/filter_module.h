#pragma once

#include "module.h"
#include "fast_math.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

// Resonant state-variable filter (LP/BP) with ping input.
class FilterModule : public Module {
public:
    enum class AuxPort : uint8_t {
        CutoffMod    = 0,
        ResonanceMod = 1
    };

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
        _ping_env = 0.0f;
        _prev_svf_freq = 0.0f;
        _prev_svf_res = -1.0f;
        _prev_svf_drive = -1.0f;
        _decimate_counter = 0;
        _drive_boost = 1.0f + _drive * 3.0f;
        _drive_vol_comp = 1.0f / (1.0f + _drive * 0.65f);
        _res_gain_comp = 1.0f / (1.0f + _res * 1.5f);
        _bass_retain = 1.0f + _res * 0.35f;
        _res_bite = _res * _res * 0.40f;
        _hf_taper = 1.0f;
    }

    void Ping(float strength) {
        float s = daisysp::fclamp(strength, 0.0f, 2.0f);
        _ping_impulse += s * 0.70f;
        _ping_env = std::max(_ping_env, s);
    }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
        // Vactrol decay envelope (~140ms tail) for ping response
        _ping_env *= 0.99935f;

        float oct_mod = daisysp::fclamp(_aux_in[0], -4.0f, 3.5f);
        float ping_oct = _ping_env * 2.4f;
        float total_oct = daisysp::fclamp(oct_mod + ping_oct, -4.0f, 4.0f);
        float target_cutoff = _base_cutoff * FastPow2(total_oct);
        
        // Safe cutoff limits (well below Nyquist/stability limit)
        target_cutoff = daisysp::fclamp(target_cutoff, 25.0f, 13500.0f);
        _current_cutoff += 0.05f * (target_cutoff - _current_cutoff);

        // Dynamic resonance ceiling: prevents instability at high cutoffs while allowing self-oscillation in low/mid register.
        float max_res = 0.92f;
        if (_current_cutoff > 1200.0f) {
            float norm = daisysp::fclamp((_current_cutoff - 1200.0f) / 10000.0f, 0.0f, 1.0f);
            max_res = 0.92f - norm * 0.24f; // smoothly tapers down to ~0.68f at 11kHz+
        }
        float eff_res = daisysp::fclamp(_res + _aux_in[1], 0.0f, max_res);

        // Subsampled coefficient update (every 4 samples = 12 kHz)
        _decimate_counter++;
        if (_decimate_counter >= 4) {
            _decimate_counter = 0;
            if (std::abs(_current_cutoff - _prev_svf_freq) > 0.15f) {
                _prev_svf_freq = _current_cutoff;
                _svf.SetFreq(_current_cutoff);

                if (_current_cutoff > 1800.0f) {
                    float hf_excess = (_current_cutoff - 1800.0f) * 0.000166667f;
                    _hf_taper = 1.0f / (1.0f + hf_excess * 1.6f);
                } else {
                    _hf_taper = 1.0f;
                }
            }
            if (std::abs(eff_res - _prev_svf_res) > 0.001f) {
                _prev_svf_res = eff_res;
                _svf.SetRes(eff_res);
                _res_gain_comp = 1.0f / (1.0f + eff_res * 1.5f);
                _bass_retain = 1.0f + eff_res * 0.35f;
                _res_bite = eff_res * eff_res * 0.40f;
            }
            float target_drv = std::max(0.35f, _drive * 0.95f);
            if (std::abs(target_drv - _prev_svf_drive) > 0.005f) {
                _prev_svf_drive = target_drv;
                _svf.SetDrive(target_drv);
            }
        }

        // Input drive saturation with even-harmonic warmth and noise gate
        float in_sum = 0.5f * (in_l + in_r) + _ping_impulse;
        _ping_impulse = 0.0f;

        // Noise floor squelch gate: dead silence when idle (< -70 dB)
        if (std::abs(in_sum) < 0.00035f && _ping_env < 0.001f) {
            in_sum = 0.0f;
        }

        // 2nd-order saturation with even harmonics
        float driven_in = in_sum * _drive_boost;
        float asym = driven_in + (_drive * 0.15f) * (driven_in * std::abs(driven_in));
        float sat_in = (asym / (1.0f + 0.35f * std::abs(asym))) * _drive_vol_comp;

        _svf.Process(sat_in);

        float low = _svf.Low();
        float band = _svf.Band();
        _band_out = band;

        // Rational soft limiting on the resonant tank
        float band_sat = (band / (1.0f + 0.32f * band * band)) * _hf_taper;
        float low_sat  = low / (1.0f + 0.12f * low * low);

        // Retain bass body when resonance is turned up
        float low_shaped = low_sat * _bass_retain;

        // Vocal formant blend in lowpass mode
        float main_out = (_mode == 1) ? (band_sat * 1.15f) : (low_shaped + band_sat * _res_bite);

        // Resonance gain compensation to balance perceived level
        main_out *= _res_gain_comp;

        // Master stage soft clip
        main_out = daisysp::SoftClip(main_out * 0.95f);

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
                _drive_boost = 1.0f + _drive * 3.0f;
                _drive_vol_comp = 1.0f / (1.0f + _drive * 0.65f);
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
    float _ping_env{0.0f};
    float _prev_svf_freq{0.0f};
    float _prev_svf_res{-1.0f};
    float _prev_svf_drive{-1.0f};
    uint8_t _decimate_counter{0};

    float _drive_boost{1.6f};
    float _drive_vol_comp{1.0f};
    float _res_gain_comp{1.0f};
    float _bass_retain{1.0f};
    float _res_bite{0.0f};
    float _hf_taper{1.0f};
};

} // namespace bends
