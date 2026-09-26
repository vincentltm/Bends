#pragma once

#include "module.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

// Dying battery / voltage starve effect.
class StarveModule : public Module {
public:
    StarveModule() = default;

    void Init(float sample_rate, uint16_t /*block_size*/) override {
        _sample_rate = sample_rate;
        _motorboat_phase = 0.0f;
        Reset();
    }

    void Reset() override {
        _vcc_smooth = 1.0f;
        _cap_charge = 1.0f;
        _motorboat_phase = 0.0f;
        _dc_x[0] = _dc_x[1] = 0.0f;
        _dc_y[0] = _dc_y[1] = 0.0f;
        _aux_in[0] = 0.0f;
        _gain = 1.0f + _drive * 3.5f;
        _vol_comp = 1.0f / (1.0f + _drive * 0.45f);
        _headroom = 1.0f;
        _inv_headroom = 1.0f;
        _asym_bias = 0.0f;
        _deadband = 0.0f;
        _decimate = 0;
    }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
        // Target voltage pulled down by aux input
        float sag_pull = daisysp::fclamp(_aux_in[0], 0.0f, 1.0f);
        float target_vcc = std::max(0.02f, _nominal_vcc - sag_pull * _sag_depth);
        _vcc_smooth += 0.025f * (target_vcc - _vcc_smooth);

        // Capacitor sag on loud inputs
        float dry_mono = 0.5f * (in_l + in_r);
        float abs_in = std::abs(dry_mono);
        if (abs_in > _cap_charge) {
            _cap_charge -= (abs_in - _cap_charge) * 0.025f * std::max(0.0f, 1.0f - _vcc_smooth);
        } else {
            _cap_charge += (_vcc_smooth - _cap_charge) * 0.005f;
        }
        _cap_charge = daisysp::fclamp(_cap_charge, 0.02f, 1.0f);
        float vcc = _cap_charge;

        // Audio path starve engages in lower half of fader (<= 0.52)
        // Top half keeps audio path clean & uncompressed so delay/chorus bloom cleanly
        float audio_vcc = 1.0f;
        if (vcc < 0.52f) {
            float t = vcc * 1.9230769f; // vcc / 0.52f
            audio_vcc = t * t * (3.0f - 2.0f * t);
        }

        // Subsampled headroom & bias update (every 4 samples = 12 kHz)
        _decimate++;
        if (_decimate >= 4) {
            _decimate = 0;
            _headroom = 0.08f + 0.92f * (audio_vcc * audio_vcc);
            _inv_headroom = 1.0f / _headroom;
            _asym_bias = (1.0f - audio_vcc) * _asym * 0.30f;
            _deadband = (1.0f - audio_vcc) * 0.015f;
        }

        // Sputter / motorboat when voltage is very low (< 0.2)
        float motorboat = 0.0f;
        if (audio_vcc < 0.20f) {
            float starve_depth = (0.20f - audio_vcc) * 5.0f;
            _motorboat_phase += (6.0f + starve_depth * 14.0f) / _sample_rate;
            if (_motorboat_phase >= 1.0f) _motorboat_phase -= 1.0f;
            motorboat = sinf(_motorboat_phase * 6.2831853f) * starve_depth * 0.20f;
        }

        // Process stereo channels
        float channels[2] = {in_l, in_r};
        for (int ch = 0; ch < 2; ++ch) {
            float driven = ((channels[ch] + motorboat) * _gain) * _inv_headroom;

            // Asymmetric rail bias
            float biased = driven + _asym_bias;
            float saturated = (biased / (1.0f + 0.45f * std::abs(biased))) * _headroom;

            // Crossover deadband
            if (_deadband > 0.002f) {
                if (saturated > _deadband) saturated -= _deadband;
                else if (saturated < -_deadband) saturated += _deadband;
                else saturated = 0.0f;
            }

            // DC blocking (~12 Hz)
            _dc_y[ch] = saturated - _dc_x[ch] + 0.9985f * _dc_y[ch];
            _dc_x[ch] = saturated;
            channels[ch] = daisysp::SoftClip(_dc_y[ch] * _vol_comp);
        }

        out_l = channels[0];
        out_r = channels[1];

        _aux_in[0] = 0.0f;
    }

    uint8_t NumParams() const override { return 4; }

    const char* GetParamName(uint8_t id) const override {
        switch (id) {
            case 0: return "Vcc";
            case 1: return "Drive";
            case 2: return "SagDepth";
            case 3: return "Asym";
            default: return "";
        }
    }

    void SetParam(uint8_t id, float value) override {
        switch (id) {
            case 0: _nominal_vcc = daisysp::fclamp(value, 0.02f, 1.0f); break;
            case 1:
                _drive     = daisysp::fclamp(value, 0.0f, 1.0f);
                _gain      = 1.0f + _drive * 3.5f;
                _vol_comp  = 1.0f / (1.0f + _drive * 0.45f);
                break;
            case 2: _sag_depth   = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 3: _asym        = daisysp::fclamp(value, 0.0f, 1.0f); break;
        }
    }

    float GetParam(uint8_t id) const override {
        switch (id) {
            case 0: return _nominal_vcc;
            case 1: return _drive;
            case 2: return _sag_depth;
            case 3: return _asym;
            default: return 0.0f;
        }
    }

    void SetAuxInput(uint8_t channel, float value) override {
        if (channel == 0) _aux_in[0] += value;
    }

    float GetAuxOutput(uint8_t /*channel*/) const override {
        return 1.0f - _vcc_smooth;
    }

    const char* Name() const override { return "Starve"; }

private:
    float _sample_rate{48000.0f};
    float _nominal_vcc{1.0f};
    float _vcc_smooth{1.0f};
    float _cap_charge{1.0f};
    float _motorboat_phase{0.0f};
    float _drive{0.2f};
    float _sag_depth{0.65f};
    float _asym{0.4f};
    float _dc_x[2]{0.0f, 0.0f};
    float _dc_y[2]{0.0f, 0.0f};
    float _aux_in[1]{0.0f};

    float _gain{1.7f};
    float _vol_comp{1.0f};
    float _headroom{1.0f};
    float _inv_headroom{1.0f};
    float _asym_bias{0.0f};
    float _deadband{0.0f};
    uint8_t _decimate{0};
};

} // namespace bends
