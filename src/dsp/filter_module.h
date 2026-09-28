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
        _res_gain_comp = 1.0f / (1.0f + _res * 1.8f);
        _bass_retain = 1.0f + _res * 0.25f;
        _res_bite = _res * _res * 0.32f;
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

        // S32 Filter Character Macro:
        // 0.00 - 0.35: Silky Clean Lowpass (resonance 0.02 -> 0.38)
        // 0.35 - 0.75: Vocal Formant Morph (blends resonant bandpass formant into lowpass body)
        // 0.75 - 1.00: Warm Filter Core Overdrive (asymmetric tube saturation on resonant tank)
        float macro_val = daisysp::fclamp(_res + _aux_in[1], 0.0f, 1.0f);

        float eff_res;
        float formant_morph = 0.0f;
        float core_overdrive = 0.0f;

        if (macro_val <= 0.35f) {
            float t = macro_val / 0.35f;
            eff_res = 0.02f + t * 0.36f;
        } else if (macro_val <= 0.75f) {
            float t = (macro_val - 0.35f) / 0.40f;
            eff_res = 0.38f + t * 0.24f;
            formant_morph = t;
        } else {
            float t = (macro_val - 0.75f) / 0.25f;
            eff_res = 0.62f + t * 0.10f;
            formant_morph = 1.0f;
            core_overdrive = t;
        }

        // High frequency stability ceiling: keeps resonance singing without high-frequency shrillness
        if (_current_cutoff > 1200.0f) {
            float norm = daisysp::fclamp((_current_cutoff - 1200.0f) / 10000.0f, 0.0f, 1.0f);
            eff_res = std::min(eff_res, 0.72f - norm * 0.12f);
        }

        // Subsampled coefficient update (every 4 samples = 12 kHz)
        _decimate_counter++;
        if (_decimate_counter >= 4) {
            _decimate_counter = 0;
            if (std::abs(_current_cutoff - _prev_svf_freq) > 0.15f) {
                _prev_svf_freq = _current_cutoff;
                _svf.SetFreq(_current_cutoff);

                if (_current_cutoff > 1800.0f) {
                    float hf_excess = (_current_cutoff - 1800.0f) * 0.000166667f;
                    _hf_taper = 1.0f / (1.0f + hf_excess * 2.0f);
                } else {
                    _hf_taper = 1.0f;
                }
            }
            if (std::abs(eff_res - _prev_svf_res) > 0.001f) {
                _prev_svf_res = eff_res;
                _svf.SetRes(eff_res);
                _res_gain_comp = 1.0f / (1.0f + eff_res * 1.6f);
                _bass_retain = 1.0f + eff_res * 0.25f;
            }
            float target_drv = std::max(0.35f, _drive * 0.95f);
            if (std::abs(target_drv - _prev_svf_drive) > 0.005f) {
                _prev_svf_drive = target_drv;
                _svf.SetDrive(target_drv);
            }
        }

        // Input drive saturation with even-harmonic warmth (clean linear when drive is low)
        float in_sum = 0.5f * (in_l + in_r) + _ping_impulse;
        _ping_impulse = 0.0f;

        float sat_in = in_sum;
        if (_drive >= 0.05f) {
            float driven_in = in_sum * _drive_boost;
            float asym = driven_in + (_drive * 0.15f) * (driven_in * std::abs(driven_in));
            sat_in = (asym / (1.0f + 0.35f * std::abs(asym))) * _drive_vol_comp;
        }

        _svf.Process(sat_in);

        float low = _svf.Low();
        float band = _svf.Band();
        _band_out = band;

        float main_out;
        if (_drive < 0.05f && macro_val < 0.25f && _ping_env < 0.005f) {
            // Pristine linear response for clean external audio when S32 and Drive are low
            main_out = (_mode == 1) ? band : low;
            main_out *= _res_gain_comp;
        } else {
            // 1. Resonant Bandpass soft limiting
            float band_sat = (band / (1.0f + 0.45f * band * band)) * _hf_taper;

            // 2. Warm Core Overdrive on resonant tank when macro > 0.75
            if (core_overdrive > 0.001f) {
                float drv_in = band_sat * (1.0f + 1.2f * core_overdrive);
                // Asymmetric 2nd-order saturation adds rich octave warmth
                float asym_tank = drv_in + (0.28f * core_overdrive) * (drv_in * std::abs(drv_in));
                band_sat = asym_tank / (1.0f + 0.45f * std::abs(asym_tank));
            }

            // 3. Lowpass shaping with bass retention
            float low_sat  = low / (1.0f + 0.15f * low * low);
            float low_shaped = low_sat * _bass_retain;

            // 4. Formant Morph: smooth blend between pure lowpass and vocal formant peak
            float formant_peak = band_sat * (0.15f + 0.45f * formant_morph);
            float low_weight = 1.0f - formant_morph * 0.20f;
            float combined = (_mode == 1) ? (band_sat * 1.15f) : (low_shaped * low_weight + formant_peak);

            // 5. Equal-loudness gain compensation + overdrive leveling
            float od_comp = 1.0f / (1.0f + core_overdrive * 0.25f);
            main_out = combined * _res_gain_comp * od_comp;
            main_out = daisysp::SoftClip(main_out * 0.90f);
        }

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
