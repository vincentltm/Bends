#pragma once

#include "module.h"
#include "fast_math.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

// Drone oscillator with sub-octave and starved sine wave.
class DroneModule : public Module {
public:
    static constexpr size_t kWavetableSize = 256;

    DroneModule() = default;

    void Init(float sample_rate, uint16_t /*block_size*/) override {
        _sample_rate = sample_rate;
        for (size_t i = 0; i < kWavetableSize; ++i) {
            float phase = (6.2831853f * static_cast<float>(i)) / static_cast<float>(kWavetableSize);
            _sine_table[i] = std::sin(phase);
        }
        Reset();
    }

    void Reset() override {
        _aux_in[0] = 0.0f;
        _aux_in[1] = 0.0f;
        _current_freq = _base_freq;
        _phase_main = 0.0f;
        _phase_sub = 0.0f;
        _sub_sample = 0.0f;
        _gate_amp_smooth = 0.0f;
        _lpg_filter = 0.0f;
        _rng_state = 13371;
    }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
        float in_mono = 0.5f * (in_l + in_r);
        float total_drive = daisysp::fclamp(_drive + _aux_in[1], 0.0f, 2.0f);
        // Voltage starve on input audio: full headroom above ~half (0.52), engages smoothly below half
        float input_vcc = 1.0f;
        if (_vcc < 0.52f) {
            float t = _vcc * 1.9230769f; // _vcc / 0.52f
            input_vcc = t * t * (3.0f - 2.0f * t);
        }
        float dry_headroom = 0.15f + 0.85f * input_vcc;
        float preamp_in = in_mono * (1.0f + total_drive * 3.5f);
        float ext_sig = (preamp_in / (1.0f + (0.35f / dry_headroom) * std::abs(preamp_in))) * dry_headroom;

        float starve = daisysp::fclamp((0.52f - _vcc) * 1.9230769f, 0.0f, 1.0f);

        _gate_amp_smooth += 0.006f * (_gate_amp - _gate_amp_smooth);

        float pitch_offset = daisysp::fclamp(_aux_in[0], -2.5f, 1.0f);
        float target_freq = daisysp::fclamp(_base_freq * FastPow2(pitch_offset), 15.0f, 3200.0f);
        _current_freq += 0.05f * (target_freq - _current_freq);

        // Clock jitter when starved
        _rng_state = _rng_state * 1664525u + 1013904223u;
        float jitter = (static_cast<float>(static_cast<int32_t>(_rng_state)) * 2.3283e-10f) * (starve * starve * 0.015f);
        float phase_inc = (_current_freq * (1.0f + jitter)) / _sample_rate;

        _phase_main += phase_inc;
        if (_phase_main >= 1.0f) _phase_main -= 1.0f;

        // Sub-oscillator (-1 octave)
        _phase_sub += phase_inc * 0.5f;
        if (_phase_sub >= 1.0f) _phase_sub -= 1.0f;

        // Starved wavetable
        float main_s = ReadStarvedSine(_phase_main, starve) * 0.22f;
        _sub_sample  = ReadStarvedSine(_phase_sub, starve * 0.75f) * 0.14f;

        float osc_gain = total_drive * 1.6f;
        float raw_osc = (main_s + _sub_sample * _sub_level) * _gate_amp_smooth * osc_gain;

        // Low-pass gate controlled by pad pressure
        float lpg_cutoff = 30.0f + _gate_amp_smooth * daisysp::fclamp(total_drive * 8500.0f, 1000.0f, 8500.0f);
        float lpg_alpha = daisysp::fclamp(lpg_cutoff * (6.2831853f / _sample_rate), 0.001f, 0.95f);
        _lpg_filter += lpg_alpha * (raw_osc - _lpg_filter);
        if (std::abs(_lpg_filter) < 1.0e-7f) _lpg_filter = 0.0f;
        float osc_sig = _lpg_filter;

        float out = ext_sig + osc_sig;
        out_l = out;
        out_r = out;

        _aux_in[0] = 0.0f;
        _aux_in[1] = 0.0f;
    }

    uint8_t NumParams() const override { return 5; }

    const char* GetParamName(uint8_t id) const override {
        switch (id) {
            case 0: return "Freq";
            case 1: return "GateAmp";
            case 2: return "SubLevel";
            case 3: return "Drive";
            case 4: return "Vcc";
            default: return "";
        }
    }

    void SetParam(uint8_t id, float value) override {
        switch (id) {
            case 0: _base_freq = daisysp::fclamp(value, 15.0f, 3200.0f); break;
            case 1: _gate_amp  = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 2: _sub_level = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 3: _drive     = daisysp::fclamp(value, 0.0f, 2.0f); break;
            case 4: _vcc       = daisysp::fclamp(value, 0.0f, 1.0f); break;
        }
    }

    float GetParam(uint8_t id) const override {
        switch (id) {
            case 0: return _base_freq;
            case 1: return _gate_amp;
            case 2: return _sub_level;
            case 3: return _drive;
            case 4: return _vcc;
            default: return 0.0f;
        }
    }

    void SetAuxInput(uint8_t channel, float value) override {
        if (channel < 2) _aux_in[channel] += value;
    }

    float GetAuxOutput(uint8_t channel) const override {
        return (channel == 0) ? _sub_sample : 0.0f;
    }

    const char* Name() const override { return "Drone"; }

private:
    float ReadStarvedSine(float phase, float starve) const {
        float pos = phase * static_cast<float>(kWavetableSize);
        size_t idx0 = static_cast<size_t>(pos);
        size_t idx1 = (idx0 + 1) & (kWavetableSize - 1);
        float frac = pos - static_cast<float>(idx0);

        // Step collapse
        float step_collapse = daisysp::fclamp(starve * 1.4f, 0.0f, 1.0f);
        float interp_frac = frac * (1.0f - step_collapse);
        float s = _sine_table[idx0] * (1.0f - interp_frac) + _sine_table[idx1] * interp_frac;

        // Bit crush
        if (starve > 0.12f) {
            float starve_sq = starve * starve;
            float denom = 1.0f + starve_sq * 48.0f;
            float quant_levels = 512.0f / denom;
            float inv_quant = denom * (1.0f / 512.0f);
            s = roundf(s * quant_levels) * inv_quant;
        }

        // Crossover deadband
        if (starve > 0.08f) {
            float deadband = starve * 0.20f;
            if (std::abs(s) < deadband) {
                s = 0.0f;
            } else if (s > 0.0f) {
                s -= deadband;
            } else {
                s += deadband;
            }
        }

        // Rail clipping
        float rail_pos = 1.0f - starve * 0.40f;
        float rail_neg = -(1.0f - starve * 0.50f);
        if (s > rail_pos) {
            float diff = s - rail_pos;
            s = rail_pos + diff / (1.0f + 2.0f * diff);
        } else if (s < rail_neg) {
            float diff = s - rail_neg;
            s = rail_neg + diff / (1.0f - 2.0f * diff);
        }

        // Wavefolding
        if (starve > 0.65f) {
            float fold_drive = (starve - 0.65f) * 2.5f;
            if (s > rail_pos * 0.75f) {
                s -= fold_drive * (s - rail_pos * 0.75f);
            }
        }

        return s;
    }

    float _sample_rate{48000.0f};
    float _sine_table[kWavetableSize]{};

    float _base_freq{110.0f};
    float _current_freq{110.0f};
    float _phase_main{0.0f};
    float _phase_sub{0.0f};
    uint32_t _rng_state{13371};

    float _sub_level{0.5f};
    float _drive{0.2f};
    float _gate_amp{0.0f};
    float _gate_amp_smooth{0.0f};
    float _lpg_filter{0.0f};
    float _vcc{1.0f};

    float _aux_in[2]{0.0f, 0.0f};
    float _sub_sample{0.0f};
};

} // namespace bends
