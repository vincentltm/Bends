#pragma once

#include "dsp/fast_math.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace bends {

class ContactMic {
public:
    struct Resonator {
        float s1{0.0f}, s2{0.0f};
        float g{0.0f}, R{0.0f}, h{0.0f};

        void Set(float freq, float Q, float sample_rate) {
            g = FastTan(kPi * freq / sample_rate);
            g = std::clamp(g, 0.001f, 1.25f);
            R = 1.0f / (2.0f * Q);
            h = 1.0f / (1.0f + 2.0f * R * g + g * g);
        }

        inline float Process(float in) {
            float high = (in - (2.0f * R + g) * s1 - s2) * h;
            float v1 = g * high;
            float band = v1 + s1;
            s1 = 2.0f * v1 + s1;
            float v2 = g * band;
            s2 = 2.0f * v2 + s2;
            return band * (2.0f * R); // Peak gain strictly normalized to 1.0 (0dB) at resonance
        }

        void Reset() { s1 = 0.0f; s2 = 0.0f; }
    };

    struct TapVoice {
        bool     active{false};
        uint16_t sample_idx{0};
        uint16_t total_samples{0};
        float    vel{0.0f};
        float    phase{0.0f};
        float    phase_inc{0.0f};

        void Trigger(float velocity, float sample_rate) {
            vel = std::clamp(velocity, 0.15f, 1.0f);
            active = true;
            sample_idx = 0;
            float dur_sec = 0.0018f - 0.0012f * vel;
            total_samples = static_cast<uint16_t>(dur_sec * sample_rate);
            if (total_samples < 16) total_samples = 16;
            phase_inc = kPi / static_cast<float>(total_samples);
            phase = 0.0f;
        }

        inline float Process(float white) {
            if (!active) return 0.0f;
            sample_idx++;
            if (sample_idx >= total_samples) {
                active = false;
                return 0.0f;
            }
            phase += phase_inc;
            float s = FastSin(phase);
            float strike = s * (1.0f - 0.35f * (static_cast<float>(sample_idx) / total_samples));
            strike += white * 0.15f * s;
            return strike * vel;
        }

        void Reset() {
            active = false;
            sample_idx = 0;
            total_samples = 0;
            vel = 0.0f;
            phase = 0.0f;
        }
    };

    void Init(float sample_rate) {
        _sample_rate = sample_rate;

        // Left Transducer modes (top-left P10 IN)
        _res_chassis_l.Set(278.0f, 3.2f, sample_rate);
        _res_ceramic_l.Set(3720.0f, 6.2f, sample_rate);

        // Right Transducer modes (top-right P11 OUT)
        _res_chassis_r.Set(286.0f, 3.0f, sample_rate);
        _res_ceramic_r.Set(3810.0f, 6.5f, sample_rate);

        Reset();
    }

    void Reset() {
        _res_chassis_l.Reset();
        _res_ceramic_l.Reset();
        _res_chassis_r.Reset();
        _res_ceramic_r.Reset();

        _hp_state_l = 0.0f;
        _hp_state_r = 0.0f;
        _lp_damp_l = 0.0f;
        _lp_damp_r = 0.0f;

        _tap_l.Reset();
        _tap_r.Reset();

        _pink_l_b0 = 0.0f; _pink_l_b1 = 0.0f; _pink_l_b2 = 0.0f;
        _pink_r_b0 = 0.0f; _pink_r_b1 = 0.0f; _pink_r_b2 = 0.0f;
        _lp_pulp_l = 0.0f;
        _lp_pulp_r = 0.0f;

        _smooth_rub_l = 0.0f;
        _smooth_rub_r = 0.0f;

        _rng = 0x93A11234u;
    }

    void TriggerTap(float velocity, bool is_right_pad = false) {
        if (is_right_pad) {
            _tap_r.Trigger(velocity, _sample_rate);
        } else {
            _tap_l.Trigger(velocity, _sample_rate);
        }
    }

    inline void Process(float ext_l, float ext_r,
                        float p10, float v_rub10,
                        float p11, float v_rub11,
                        float& out_l, float& out_r) {
        // Quick idle bypass when no physical excitation or audio input is present
        bool has_touch = (p10 > 0.015f || p11 > 0.015f || _tap_l.active || _tap_r.active || v_rub10 > 0.0004f || v_rub11 > 0.0004f);
        bool has_audio = (std::abs(ext_l) > 1e-4f || std::abs(ext_r) > 1e-4f);
        if (!has_touch && !has_audio && std::abs(_lp_damp_l) < 1e-4f && std::abs(_lp_damp_r) < 1e-4f) {
            out_l = 0.0f;
            out_r = 0.0f;
            return;
        }

        // Friction Voices (Stochastic Physical Modeling)
        _smooth_rub_l += (v_rub10 - _smooth_rub_l) * 0.020f;
        float eff_l = std::max(v_rub10 * 0.75f, _smooth_rub_l);

        _smooth_rub_r += (v_rub11 - _smooth_rub_r) * 0.020f;
        float eff_r = std::max(v_rub11 * 0.75f, _smooth_rub_r);

        _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
        float white_l = static_cast<float>(static_cast<int32_t>(_rng)) * (1.0f / 2147483648.0f);

        // Left Pink Noise Filter (Kellet 3-pole)
        _pink_l_b0 = 0.99765f * _pink_l_b0 + white_l * 0.0555179f;
        _pink_l_b1 = 0.96300f * _pink_l_b1 + white_l * 0.1538520f;
        _pink_l_b2 = 0.57000f * _pink_l_b2 + white_l * 0.5329522f;
        float pink_l = (_pink_l_b0 + _pink_l_b1 + _pink_l_b2 + white_l * 0.5362f) * 0.22f;

        float scratch_sig_l = 0.0f;
        if (eff_l > 0.0004f && p10 > 0.015f) {
            bool asperity = ((_rng & 0x0FFF) < static_cast<uint32_t>(eff_l * 3500.0f));
            float micro_l = asperity ? (white_l * (0.35f + 0.65f * eff_l)) : 0.0f;
            float raw_fric_l = pink_l * 0.75f + micro_l * 0.25f;

            float cutoff_l = (650.0f + eff_l * 3200.0f) * (1.0f - p10 * 0.35f);
            float g_l = std::clamp(cutoff_l * (kPi / _sample_rate), 0.01f, 0.45f);
            _lp_pulp_l += g_l * (raw_fric_l - _lp_pulp_l);

            float cl_eff_l = std::clamp(eff_l, 0.0f, 1.2f);
            float gain_l = (cl_eff_l * 0.65f + cl_eff_l * cl_eff_l * 0.35f) * (0.30f + 0.70f * p10);
            scratch_sig_l = _lp_pulp_l * gain_l * 1.25f;
        } else {
            _pink_l_b0 *= 0.88f; _pink_l_b1 *= 0.88f; _pink_l_b2 *= 0.88f; _lp_pulp_l *= 0.88f;
        }

        _rng ^= _rng << 13; _rng ^= _rng >> 17; _rng ^= _rng << 5;
        float white_r = static_cast<float>(static_cast<int32_t>(_rng)) * (1.0f / 2147483648.0f);

        // Right Pink Noise Filter
        _pink_r_b0 = 0.99765f * _pink_r_b0 + white_r * 0.0555179f;
        _pink_r_b1 = 0.96300f * _pink_r_b1 + white_r * 0.1538520f;
        _pink_r_b2 = 0.57000f * _pink_r_b2 + white_r * 0.5329522f;
        float pink_r = (_pink_r_b0 + _pink_r_b1 + _pink_r_b2 + white_r * 0.5362f) * 0.22f;

        float scratch_sig_r = 0.0f;
        if (eff_r > 0.0004f && p11 > 0.015f) {
            bool asperity = ((_rng & 0x0FFF) < static_cast<uint32_t>(eff_r * 3500.0f));
            float micro_r = asperity ? (white_r * (0.35f + 0.65f * eff_r)) : 0.0f;
            float raw_fric_r = pink_r * 0.75f + micro_r * 0.25f;

            float cutoff_r = (650.0f + eff_r * 3200.0f) * (1.0f - p11 * 0.35f);
            float g_r = std::clamp(cutoff_r * (kPi / _sample_rate), 0.01f, 0.45f);
            _lp_pulp_r += g_r * (raw_fric_r - _lp_pulp_r);

            float cl_eff_r = std::clamp(eff_r, 0.0f, 1.2f);
            float gain_r = (cl_eff_r * 0.65f + cl_eff_r * cl_eff_r * 0.35f) * (0.30f + 0.70f * p11);
            scratch_sig_r = _lp_pulp_r * gain_r * 1.25f;
        } else {
            _pink_r_b0 *= 0.88f; _pink_r_b1 *= 0.88f; _pink_r_b2 *= 0.88f; _lp_pulp_r *= 0.88f;
        }

        // Tap Impulse Voices
        float tap_sig_l = _tap_l.Process(white_l);
        float tap_sig_r = _tap_r.Process(white_r);

        float exc_l = tap_sig_l * 1.20f + scratch_sig_l;
        float exc_r = tap_sig_r * 1.20f + scratch_sig_r;

        float tactile_l = exc_l * 0.70f + exc_r * 0.30f;
        float tactile_r = exc_r * 0.70f + exc_l * 0.30f;

        // Capacitive Highpass & Sub Bass Coupling
        float hp_freq_l = 120.0f * (1.0f - p10 * 0.70f);
        float hp_alpha_l = 1.0f / (1.0f + kTwoPi * hp_freq_l / _sample_rate);
        float sub_boost_l = 1.0f + p10 * 0.80f;

        float hp_freq_r = 120.0f * (1.0f - p11 * 0.70f);
        float hp_alpha_r = 1.0f / (1.0f + kTwoPi * hp_freq_r / _sample_rate);
        float sub_boost_r = 1.0f + p11 * 0.80f;

        float raw_l = ext_l * 0.60f + tactile_l;
        _hp_state_l = hp_alpha_l * (_hp_state_l + raw_l);
        float sig_l = (raw_l - _hp_state_l) * sub_boost_l;

        float raw_r = ext_r * 0.60f + tactile_r;
        _hp_state_r = hp_alpha_r * (_hp_state_r + raw_r);
        float sig_r = (raw_r - _hp_state_r) * sub_boost_r;

        // Resonators
        float chas_l = _res_chassis_l.Process(sig_l);
        float chas_r = _res_chassis_r.Process(sig_r);
        float ceram_l = _res_ceramic_l.Process(sig_l);
        float ceram_r = _res_ceramic_r.Process(sig_r);

        float disc_out_l = sig_l * 0.35f + chas_l * 0.40f + ceram_l * 0.45f;
        float disc_out_r = sig_r * 0.35f + chas_r * 0.40f + ceram_r * 0.45f;

        // Stereo acoustic coupling
        float stereo_l = disc_out_l * 0.60f + disc_out_r * 0.40f;
        float stereo_r = disc_out_r * 0.60f + disc_out_l * 0.40f;

        // Damping
        _lp_damp_l += 0.67f * (stereo_l - _lp_damp_l);
        _lp_damp_r += 0.67f * (stereo_r - _lp_damp_r);

        out_l = SaturatePiezo(_lp_damp_l * 0.90f);
        out_r = SaturatePiezo(_lp_damp_r * 0.90f);
    }

private:
    inline float SaturatePiezo(float x) const {
        float x_clip = std::clamp(x, -2.5f, 2.5f);
        float sat = x_clip + 0.14f * x_clip * x_clip - 0.05f * x_clip * x_clip * x_clip;
        return std::clamp(sat, -1.0f, 1.0f);
    }

    float _sample_rate{48000.0f};

    Resonator _res_chassis_l;
    Resonator _res_ceramic_l;
    Resonator _res_chassis_r;
    Resonator _res_ceramic_r;

    float _hp_state_l{0.0f};
    float _hp_state_r{0.0f};
    float _lp_damp_l{0.0f};
    float _lp_damp_r{0.0f};

    TapVoice _tap_l;
    TapVoice _tap_r;

    float _pink_l_b0{0.0f}, _pink_l_b1{0.0f}, _pink_l_b2{0.0f};
    float _pink_r_b0{0.0f}, _pink_r_b1{0.0f}, _pink_r_b2{0.0f};
    float _lp_pulp_l{0.0f};
    float _lp_pulp_r{0.0f};
    float _smooth_rub_l{0.0f};
    float _smooth_rub_r{0.0f};
    uint32_t _rng{0x93A11234u};
};

} // namespace bends
