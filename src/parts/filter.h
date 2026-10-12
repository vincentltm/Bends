#pragma once

#include "dsp/fast_math.h"
#include "dsp/filters.h"
#include "bends/mod_pack.h"
#include <cmath>
#include <algorithm>

namespace bends {

class Filter {
public:
    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() {
        _flt_low_l = 0.0f; _flt_low_r = 0.0f;
        _flt_s1_l = 0.0f;  _flt_s2_l = 0.0f;
        _flt_s1_r = 0.0f;  _flt_s2_r = 0.0f;
        _ping = 0.0f;
        _ping_env = 0.0f;
        _base_freq = 440.0f;
        _dc_block_l.Reset();
        _dc_block_r.Reset();
    }

    void TriggerPing(float strength) {
        _ping = std::max(_ping, strength);
    }

    inline void PrepareBlock(float knob_flt, float node_pull, float touch_flt, float air_hover) {
        float flt_dc = std::clamp(knob_flt + node_pull * 0.20f + 0.15f * touch_flt + air_hover * 0.12f, 0.0f, 1.0f);
        _base_freq = 35.0f * FastPow2(flt_dc * 8.6f);
    }

    inline void Process(float in_l, float in_r,
                        float touch_flt,
                        const FilterMods& mods,
                        float v_rail_eff, float starve,
                        float shunt_gnd,
                        float& out_l, float& out_r) {
        // Cutoff scaling with rail voltage
        float flt_v_scale;
        if (v_rail_eff < 0.50f) {
            float s = v_rail_eff / 0.50f;
            flt_v_scale = 0.25f + 0.75f * (s * s);
        } else {
            flt_v_scale = 0.88f + 0.12f * ((v_rail_eff - 0.50f) / 0.50f);
        }

        float freq = _base_freq * flt_v_scale;

        // Bends: Saw ramp sweep or clock step
        if (mods.cutoff_ramp > 0.001f) {
            freq *= FastPow2(mods.cutoff_ramp);
        }
        if (std::abs(mods.clock_step) > 0.001f) {
            freq *= FastPow2(mods.clock_step * 1.5f);
        }
        freq = std::clamp(freq, 25.0f, 19000.0f);

        // Ping impulses
        if (mods.ping > 0.0f) {
            _ping = std::max(_ping, mods.ping);
        }

        if (_ping > 0.001f) {
            _flt_s1_l += _ping;
            _flt_s1_r += _ping;
            in_l += _ping * 1.5f;
            in_r += _ping * 1.5f;
            _ping_env = 1.0f;
            _ping = 0.0f;
        } else {
            _ping_env *= 0.9997f; // ~250 ms ring tail
        }

        // Resonance
        float res_raw = 0.62f + touch_flt * 0.35f + _ping_env * 0.32f + mods.resonance;
        res_raw *= shunt_gnd;
        res_raw = std::clamp(res_raw, 0.0f, 0.98f);

        float starve_choke = std::clamp((v_rail_eff - 0.15f) / 0.45f, 0.0f, 1.0f);
        starve_choke = starve_choke + (1.0f - starve_choke) * std::min(1.0f, mods.howl * 1.4f);
        float res_eff = res_raw * starve_choke;

        float R = 0.45f * (1.0f - std::clamp(res_eff, 0.0f, 0.98f)) + 0.02f;
        float starve_damping = (v_rail_eff < 0.50f) ? (0.50f - v_rail_eff) * 1.6f : 0.0f;
        starve_damping *= (1.0f - std::min(1.0f, mods.howl * 1.5f));
        R += starve_damping;

        if (mods.howl > 0.08f && starve > 0.35f) {
            R = std::max(0.015f, R - mods.howl * 0.45f);
            in_l += starve * mods.howl * 0.35f;
            in_r += starve * mods.howl * 0.35f;
        }

        // 2x oversampled TPT SVF trapezoidal integration
        float g = FastTan(kPi * freq / (2.0f * _sample_rate));
        g = std::clamp(g, 0.001f, 1.25f);
        float h = 1.0f / (1.0f + 2.0f * R * g + g * g);

        float flt_headroom = (v_rail_eff > 0.45f) ? (0.80f + 0.20f * ((v_rail_eff - 0.45f) / 0.55f))
                                                  : (0.25f + 0.55f * (v_rail_eff / 0.45f));

        float starve_drive = std::max(0.0f, starve - 0.55f) * 0.70f;
        float drive = 1.0f + mods.drive + starve_drive;
        in_l *= drive;
        in_r *= drive;

        for (int step = 0; step < 2; ++step) {
            // Left channel
            float high_l = (in_l - (2.0f * R + g) * _flt_s1_l - _flt_s2_l) * h;
            high_l = Saturate(high_l, flt_headroom, starve);
            float v1_l = g * high_l;
            float band_l = v1_l + _flt_s1_l;
            _flt_s1_l = 2.0f * v1_l + _flt_s1_l;
            float v2_l = g * band_l;
            _flt_low_l = v2_l + _flt_s2_l;
            _flt_s2_l = 2.0f * v2_l + _flt_s2_l;

            // Right channel
            float high_r = (in_r - (2.0f * R + g) * _flt_s1_r - _flt_s2_r) * h;
            high_r = Saturate(high_r, flt_headroom, starve);
            float v1_r = g * high_r;
            float band_r = v1_r + _flt_s1_r;
            _flt_s1_r = 2.0f * v1_r + _flt_s1_r;
            float v2_r = g * band_r;
            _flt_low_r = v2_r + _flt_s2_r;
            _flt_s2_r = 2.0f * v2_r + _flt_s2_r;
        }

        if (std::isnan(_flt_low_l) || std::isinf(_flt_low_l)) {
            _flt_low_l = 0.0f; _flt_s1_l = 0.0f; _flt_s2_l = 0.0f;
        }
        if (std::isnan(_flt_low_r) || std::isinf(_flt_low_r)) {
            _flt_low_r = 0.0f; _flt_s1_r = 0.0f; _flt_s2_r = 0.0f;
        }

        _flt_low_l = Saturate(_flt_low_l, flt_headroom, starve);
        _flt_low_r = Saturate(_flt_low_r, flt_headroom, starve);

        float flt_makeup = std::min(1.25f, 1.0f / std::sqrt(0.35f + 0.65f * v_rail_eff));
        _flt_low_l = _dc_block_l.Process(_flt_low_l * flt_makeup * shunt_gnd);
        _flt_low_r = _dc_block_r.Process(_flt_low_r * flt_makeup * shunt_gnd);

        out_l = _flt_low_l;
        out_r = _flt_low_r;
    }

private:
    inline float Saturate(float x, float headroom, float starve) const {
        float h_pos = headroom * (1.0f - 0.20f * starve);
        float h_neg = headroom * (1.0f + 0.08f * starve);
        float sat = (x >= 0.0f) ? (h_pos * FastTanh(x / h_pos)) : (h_neg * FastTanh(x / h_neg));

        if (starve > 0.70f) {
            float deadzone = 0.04f * (starve - 0.70f) / 0.30f;
            if (std::abs(sat) < deadzone) {
                float sgn = (sat >= 0.0f) ? 1.0f : -1.0f;
                sat = sgn * (sat * sat) / deadzone;
            }
        }
        return sat;
    }

    float _sample_rate{48000.0f};
    float _flt_low_l{0.0f}; float _flt_low_r{0.0f};
    float _flt_s1_l{0.0f};  float _flt_s2_l{0.0f};
    float _flt_s1_r{0.0f};  float _flt_s2_r{0.0f};
    float _ping{0.0f};
    float _ping_env{0.0f};
    float _base_freq{440.0f};

    DcBlocker _dc_block_l;
    DcBlocker _dc_block_r;
};

} // namespace bends
