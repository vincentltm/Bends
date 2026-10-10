#pragma once

#include "daisysp.h"
#include <cmath>
#include <algorithm>
#include "piezo_tables.inc"

namespace synthux {

/**
 * Dual-Transducer Stereo Chassis Contact Microphone Model
 * 
 * Features:
 * 1. Dual-Transducer Physical Resonator Core:
 *    - Left Disc:  3720 Hz ceramic mode (Q=6.2), 278 Hz chassis mode (Q=3.2)
 *    - Right Disc: 3810 Hz ceramic mode (Q=6.5), 286 Hz chassis mode (Q=3.0)
 *    - Matching acoustic manufacturing tolerance offset of real stereo contact mic pairs.
 * 2. Solid Substrate Acoustic Wave Propagation:
 *    - 7-sample (~0.15ms) inter-transducer structural delay across PCB/chassis.
 *    - High-frequency material damping filter (4.5 kHz) cross-coupling L <-> R.
 * 3. Tactile P10 Surface Excitation:
 *    - Tap strikes trigger real recorded piezo micro-transients (kPiezoTap, kPiezoClick).
 *    - Scratching/rubbing dynamically scrubs through kPiezoScratch proportional to rub velocity.
 *    - Pressure coupling extends sub-bass from 120 Hz down to 35 Hz.
 * 4. External Audio Shaping:
 *    - Stereo audio inputs (ext_l, ext_r) pass through the contact mic transfer function.
 */
class ContactMicModel {
public:
    struct Resonator {
        float s1{0.0f}, s2{0.0f};
        float g{0.0f}, R{0.0f}, h{0.0f};

        void Set(float freq, float Q, float sample_rate) {
            g = std::tan(3.14159265f * freq / sample_rate);
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
            return band;
        }

        void Reset() { s1 = 0.0f; s2 = 0.0f; }
    };

    void Init(float sample_rate) {
        _sample_rate = sample_rate;

        // Left Disc modes (top-left near P10)
        _res_chassis_l.Set(278.0f, 3.2f, sample_rate);
        _res_ceramic_l.Set(3720.0f, 6.2f, sample_rate);

        // Right Disc modes (top-right near P11)
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
        _cross_lp_l = 0.0f;
        _cross_lp_r = 0.0f;

        std::fill(std::begin(_sub_delay_l), std::end(_sub_delay_l), 0.0f);
        std::fill(std::begin(_sub_delay_r), std::end(_sub_delay_r), 0.0f);
        _sub_idx = 0;

        _trans_active = false;
        _trans_pos = 0.0f;
        _trans_vel = 0.0f;
        _trans_type = 0;

        _scratch_pos = 0.0f;
        _smooth_rub_speed = 0.0f;
        _lfsr = 0x93A1u;
    }

    void TriggerTap(float velocity, bool is_sharp = false) {
        _trans_active = true;
        _trans_pos = 0.0f;
        _trans_vel = std::clamp(velocity, 0.05f, 1.0f);
        _trans_type = is_sharp ? 1 : 0;
    }

    /**
     * Process one stereo sample.
     * @param ext_l External audio left input
     * @param ext_r External audio right input
     * @param p10 Current touch pressure on Pad 10 (0.0 .. 1.0)
     * @param v_rub Rubbing/scratching velocity on Pad 10
     * @param out_l Output left (contact mic signal)
     * @param out_r Output right (contact mic signal)
     */
    inline void Process(float ext_l, float ext_r, float p10, float v_rub, float& out_l, float& out_r) {
        // 1. Friction & Scratch Voice (Original Authentic Modeling)
        _smooth_rub_speed += (v_rub - _smooth_rub_speed) * 0.015f;
        float eff_rub = std::max(v_rub * 0.70f, _smooth_rub_speed);

        float scratch_sig = 0.0f;
        if (eff_rub > 0.008f && p10 > 0.03f) {
            // Speed of scrub advances through recorded grain table
            float scrub_rate = 0.20f + eff_rub * 1.8f;
            _scratch_pos += scrub_rate;
            while (_scratch_pos >= static_cast<float>(kPiezoScratch_len - 1)) {
                _scratch_pos -= static_cast<float>(kPiezoScratch_len - 1);
            }

            int idx0 = static_cast<int>(_scratch_pos);
            int idx1 = idx0 + 1;
            if (idx1 >= kPiezoScratch_len) idx1 = 0;
            float frac = _scratch_pos - static_cast<float>(idx0);

            float s0 = static_cast<float>(kPiezoScratch[idx0]) / 128.0f;
            float s1 = static_cast<float>(kPiezoScratch[idx1]) / 128.0f;
            float grain_val = s0 + frac * (s1 - s0);

            // Friction power: normal force (pressure) * rubbing velocity
            float friction_power = std::min(1.0f, (0.15f + p10 * 0.85f) * eff_rub * 2.2f);

            // Stick-slip micro-crackle
            _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
            float noise = (static_cast<float>(_lfsr % 2001) - 1000.0f) / 1000.0f;
            float micro_slip = (std::abs(noise) > 0.88f) ? (noise * 0.35f) : 0.0f;

            scratch_sig = (grain_val * 0.75f + micro_slip * 0.25f) * friction_power * 1.0f;
        }

        // 2. Transient Tap Voice (Authentic Piezo Chassis Tap)
        float tap_sig = 0.0f;
        if (_trans_active) {
            // 24kHz table played at 48kHz sample rate -> advance 0.5 samples/step
            _trans_pos += 0.50f;
            const int16_t table_len = kPiezoTap_len;
            const int8_t* table_data = kPiezoTap;

            if (_trans_pos >= static_cast<float>(table_len - 1)) {
                _trans_active = false;
            } else {
                int idx0 = static_cast<int>(_trans_pos);
                int idx1 = idx0 + 1;
                float frac = _trans_pos - static_cast<float>(idx0);
                float s0 = static_cast<float>(table_data[idx0]) / 128.0f;
                float s1 = static_cast<float>(table_data[idx1]) / 128.0f;
                tap_sig = (s0 + frac * (s1 - s0)) * _trans_vel * 0.85f;
            }
        }

        // Combined tactile impact & scratch on P10 (top-left)
        // Balanced physical chassis excitation:
        // Left Disc is closer to P10 (0.65 direct energy)
        // Right Disc receives chassis plate conduction (0.52 direct energy)
        // Ratio = 1.25 (~1.9dB), creating a natural left-biased spatial image with rich stereo width
        float tactile_in_l = (tap_sig + scratch_sig) * 0.65f;
        float tactile_in_r = (tap_sig + scratch_sig) * 0.52f;

        // 3. Capacitive High-Pass Loading & Surface Pressure Coupling
        // Light touch: 120 Hz HP (thin/tinny piezo).
        // Firm press: couples body mass down to 35 Hz with warm chest thump!
        float hp_freq = 120.0f * (1.0f - p10 * 0.70f);
        float hp_alpha = 1.0f / (1.0f + 6.2831853f * hp_freq / _sample_rate);
        float sub_boost = 1.0f + p10 * 1.5f;

        // Left channel input
        float raw_l = ext_l * 0.85f + tactile_in_l;
        _hp_state_l = hp_alpha * (_hp_state_l + raw_l);
        float sig_l = (raw_l - _hp_state_l) * sub_boost;

        // Right channel input
        float raw_r = ext_r * 0.85f + tactile_in_r;
        _hp_state_r = hp_alpha * (_hp_state_r + raw_r);
        float sig_r = (raw_r - _hp_state_r) * sub_boost;

        // 4. Acoustic Resonator Modes (Full Authentic Resonator Gain)
        // Chassis cavity mode (278 Hz L, 286 Hz R)
        float chas_l = _res_chassis_l.Process(sig_l);
        float chas_r = _res_chassis_r.Process(sig_r);

        // Ceramic disc drumhead mode (3720 Hz L, 3810 Hz R)
        float ceram_l = _res_ceramic_l.Process(sig_l);
        float ceram_r = _res_ceramic_r.Process(sig_r);

        float disc_out_l = sig_l * 0.55f + chas_l * 0.75f + ceram_l * 1.15f;
        float disc_out_r = sig_r * 0.55f + chas_r * 0.75f + ceram_r * 1.15f;

        // 5. Solid Substrate Acoustic Wave Propagation (7-sample structural delay)
        // Cross-coupled through 4.5 kHz material damping filter
        uint8_t read_idx = (_sub_idx + 1) & 7;
        float delayed_from_l = _sub_delay_l[read_idx];
        float delayed_from_r = _sub_delay_r[read_idx];

        _sub_delay_l[_sub_idx] = disc_out_l;
        _sub_delay_r[_sub_idx] = disc_out_r;
        _sub_idx = (_sub_idx + 1) & 7;

        // 4.5 kHz acoustic material damping filter
        _cross_lp_l += 0.44f * (delayed_from_r - _cross_lp_l);
        _cross_lp_r += 0.44f * (delayed_from_l - _cross_lp_r);

        float stereo_l = disc_out_l + _cross_lp_l * 0.28f;
        float stereo_r = disc_out_r + _cross_lp_r * 0.28f;

        // 6. Mechanical High-Frequency Damping (8.5 kHz solid-medium rolloff)
        _lp_damp_l += 0.67f * (stereo_l - _lp_damp_l);
        _lp_damp_r += 0.67f * (stereo_r - _lp_damp_r);

        // 7. Asymmetric Non-Linear Piezoelectric Saturation
        out_l = SaturatePiezo(_lp_damp_l * 0.68f);
        out_r = SaturatePiezo(_lp_damp_r * 0.68f);
    }

private:
    inline float SaturatePiezo(float x) const {
        // Real piezoelectric crystals compress asymmetrically under mechanical stress
        float x_clip = std::clamp(x, -2.5f, 2.5f);
        float sat = x_clip + 0.14f * x_clip * x_clip - 0.05f * x_clip * x_clip * x_clip;
        return std::clamp(sat, -1.0f, 1.0f);
    }

    float _sample_rate{48000.0f};

    // Dual Transducer Resonators
    Resonator _res_chassis_l;
    Resonator _res_ceramic_l;
    Resonator _res_chassis_r;
    Resonator _res_ceramic_r;

    // Filters
    float _hp_state_l{0.0f};
    float _hp_state_r{0.0f};
    float _lp_damp_l{0.0f};
    float _lp_damp_r{0.0f};
    float _cross_lp_l{0.0f};
    float _cross_lp_r{0.0f};

    // Structural delay buffer (8 samples circular buffer, 7-sample propagation delay)
    float _sub_delay_l[8]{0.0f};
    float _sub_delay_r[8]{0.0f};
    uint8_t _sub_idx{0};

    // Transient Player Voice
    bool    _trans_active{false};
    float   _trans_pos{0.0f};
    float   _trans_vel{0.0f};
    uint8_t _trans_type{0}; // 0 = tap, 1 = click

    // Scratch Grain Scrubber Voice
    float    _scratch_pos{0.0f};
    float    _smooth_rub_speed{0.0f};
    uint16_t _lfsr{0x93A1u};
};

} // namespace synthux
