#pragma once

#include "dsp/fast_math.h"
#include "dsp/filters.h"
#include "bends/mod_pack.h"
#include <cmath>
#include <algorithm>

namespace bends {

class Filter {
public:
    struct PingExciter {
        bool     active{false};
        uint16_t sample_idx{0};
        uint16_t total_samples{24};
        float    amplitude{0.0f};
        float    phase{0.0f};
        float    phase_inc{0.0f};

        void Trigger(float vel) {
            vel = std::clamp(vel, 0.15f, 1.20f);
            active = true;
            sample_idx = 0;
            total_samples = 24; // 0.5 ms @ 48 kHz
            phase_inc = kPi / 24.0f;
            phase = 0.0f;
            amplitude = vel;
        }

        inline float Tick() {
            if (!active) return 0.0f;
            sample_idx++;
            if (sample_idx >= total_samples) {
                active = false;
                return 0.0f;
            }
            phase += phase_inc;
            return FastSin(phase) * amplitude;
        }

        void Reset() {
            active = false;
            sample_idx = 0;
            phase = 0.0f;
            amplitude = 0.0f;
        }
    };

    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() {
        _flt_low_l = 0.0f; _flt_low_r = 0.0f;
        _flt_s1_l = 0.0f;  _flt_s2_l = 0.0f;
        _flt_s1_r = 0.0f;  _flt_s2_r = 0.0f;
        _ping_env = 0.0f;
        _base_freq = 440.0f;
        _exciter_l.Reset();
        _exciter_r.Reset();
        _dc_block_l.Reset();
        _dc_block_r.Reset();
    }

    void TriggerPing(float strength) {
        _exciter_l.Trigger(strength);
        _exciter_r.Trigger(strength);
        _ping_env = 1.0f;
    }

    inline void PrepareBlock(float knob_flt, float node_pull, float touch_flt, float air_hover) {
        float flt_dc = std::clamp(knob_flt + node_pull * 0.20f + 0.12f * touch_flt + air_hover * 0.12f, 0.0f, 1.0f);
        float target = 28.0f * FastPow2(flt_dc * 9.6f);
        _base_freq += 0.25f * (target - _base_freq);
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
        freq = std::clamp(freq, 25.0f, 21500.0f);

        // Bends: trigger ping from mod pack
        if (mods.ping > 0.001f) {
            TriggerPing(mods.ping);
        }

        // 2x oversampled TPT SVF pre-warped frequency
        float g = FastTan(kPi * freq / (2.0f * _sample_rate));
        g = std::clamp(g, 0.001f, 1.25f);

        // Bandlimited smooth acoustic excitation pulse into filter inputs (zero DC clicks/aliasing)
        float exc_scale = std::min(2.5f, 0.09f / std::sqrt(std::max(0.001f, g)));
        float exc_l = _exciter_l.Tick() * exc_scale;
        float exc_r = _exciter_r.Tick() * exc_scale;
        in_l += exc_l;
        in_r += exc_r;

        // Ping decay envelope
        if (_ping_env > 0.0001f) {
            _ping_env *= 0.99982f; // ~150-250 ms ring decay
        } else {
            _ping_env = 0.0f;
        }

        // Isochronous acoustic ping resonance:
        // Scales damping factor r_ping with frequency so all cutoff frequencies ring with singing acoustic presence
        float r_ping = std::clamp(0.0006f / g, 0.025f, 0.075f);
        float ping_fade = 1.0f - _ping_env;
        float ping_fade3 = ping_fade * ping_fade * ping_fade;
        float r_ping_eff = r_ping + (0.707f - r_ping) * ping_fade3;

        // Pad 04 continuous touch sweeps resonance from Butterworth (0.707) into singing self-oscillation
        // Smooth musical curve bounded between 0.707 (flat) and 0.028 (Q ≈ 18 singing limit-cycle tone)
        float res = std::clamp(touch_flt + mods.resonance, 0.0f, 1.0f);
        float inv_res = 1.0f - res;
        float r_touch = 0.028f + (0.707f - 0.028f) * (inv_res * inv_res);

        // Active damping R (lowest damping factor wins, strictly bounded to prevent harsh digital runaway)
        float R = std::min(r_ping_eff, r_touch);
        R = std::max(0.025f, R);
        // Ground shunt damping (pulling FLT to GND suppresses resonance)
        R = 0.707f + (R - 0.707f) * shunt_gnd;

        float starve_damping = (v_rail_eff < 0.50f) ? (0.50f - v_rail_eff) * 1.6f : 0.0f;
        starve_damping *= (1.0f - std::min(1.0f, mods.howl * 1.5f));
        R += starve_damping;

        if (mods.howl > 0.08f && starve > 0.35f) {
            R = std::max(0.015f, R - mods.howl * 0.45f);
            in_l += starve * mods.howl * 0.35f;
            in_r += starve * mods.howl * 0.35f;
        }

        float h = 1.0f / (1.0f + 2.0f * R * g + g * g);

        float flt_headroom = (v_rail_eff > 0.45f) ? (1.50f + 0.30f * ((v_rail_eff - 0.45f) / 0.55f))
                                                  : (0.35f + 1.15f * (v_rail_eff / 0.45f));

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
        if (starve < 0.02f) {
            float mag = std::abs(x);
            if (mag <= 1.05f) return x;
            float span = 0.35f;
            float sat = 1.05f + span * FastTanh((mag - 1.05f) / span);
            return (x >= 0.0f) ? sat : -sat;
        }

        float h = headroom * (1.0f - 0.40f * starve);
        float h_pos = h * (1.0f - 0.20f * starve);
        float h_neg = h * (1.0f + 0.08f * starve);
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
    float _ping_env{0.0f};
    float _base_freq{440.0f};

    PingExciter _exciter_l;
    PingExciter _exciter_r;

    DcBlocker _dc_block_l;
    DcBlocker _dc_block_r;
};

} // namespace bends
