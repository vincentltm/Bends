#pragma once

#include "dsp/fast_math.h"
#include "dsp/filters.h"
#include "bends/mod_pack.h"
#include <cmath>
#include <algorithm>

namespace bends {

class Preamp {
public:
    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() {
        _dc_block_l.Reset();
        _dc_block_r.Reset();
        _pluck = 0.0f;
        _motorboat_phase = 0.0f;
        _vol = 1.0f;
        _target_vol = 1.0f;
        _drive = 1.0f;
        _target_drive = 1.0f;
    }

    void TriggerPluck(float strength) {
        _pluck = std::max(_pluck, strength);
    }

    inline void PrepareBlock(float knob_drive, float node_pull) {
        float u = std::clamp(knob_drive + node_pull * 0.20f, 0.0f, 1.0f);
        if (u < 0.02f) {
            _target_vol = 0.0f;
            _target_drive = 1.0f;
        } else if (u <= 0.50f) {
            // 0.02 to 0.50 (12 o'clock center): Clean volume control from 0.0 to 1.0 unity gain
            float t = (u - 0.02f) / 0.48f; // in [0, 1]
            _target_vol = t * std::sqrt(t); // Natural audio taper reaching exactly 1.0 at center
            _target_drive = 1.0f;          // Pure clean headroom below center
        } else {
            // 0.50 to 1.00 (Past 12 o'clock): Clean unity volume maintained while smoothly boosting drive
            float t = (u - 0.50f) / 0.50f; // in [0, 1]
            _target_vol = 1.0f;
            _target_drive = 1.0f + 5.5f * (t * t); // Drive scales smoothly from 1.0 to 6.5
        }
        // Smooth control block updates
        _vol += 0.20f * (_target_vol - _vol);
    }

    inline void Process(float in_l, float in_r,
                        float touch_amp,
                        const PreampMods& mods,
                        float osc_inject, float hum_inject,
                        float v_rail_eff, float starve,
                        float shunt_gnd,
                        float& out_l, float& out_r) {
        // Pluck impulse decay
        float pluck_sig = 0.0f;
        if (_pluck > 0.001f) {
            pluck_sig = _pluck;
            _pluck *= 0.995f;
        }

        // Dying battery motorboat pulse (< 0.35V)
        float mb_sig = 0.0f;
        if (v_rail_eff < 0.35f) {
            float starve_depth = (0.35f - v_rail_eff) / 0.35f;
            float mb_rate = 2.5f + 5.5f * (1.0f - starve_depth);
            _motorboat_phase += mb_rate * kTwoPi / _sample_rate;
            if (_motorboat_phase >= kTwoPi) _motorboat_phase -= kTwoPi;
            float mb_pulse = (FastSin(_motorboat_phase) > 0.65f) ? 0.8f : -0.15f;
            mb_sig = 0.040f * starve_depth * mb_pulse * (0.30f + 0.70f * _vol);
        }

        float bias_drift = 0.18f * (starve * starve) + mods.bias;
        float deadzone = 0.07f * (starve * starve);
        float sputter_th = (starve > 0.40f) ? ((starve - 0.40f) * 0.15f) : 0.0f;
        float touch_drive = touch_amp * 4.0f;

        float target_drive = (_target_drive + touch_drive + mods.drive) * (1.0f + 1.2f * starve);
        _drive += 0.025f * (target_drive - _drive);
        float drive_eff = _drive * (1.0f + 0.8f * starve);
        float bias_null = (starve > 0.02f || std::abs(bias_drift) > 0.01f) ? FastTanh(bias_drift * drive_eff) : 0.0f;

        // Dynamic loudness compensation: prevents volume explosion when pushed into heavy saturation
        float comp = (_drive > 1.0f) ? (1.0f / (1.0f + 0.18f * (_drive - 1.0f))) : 1.0f;

        // Master preamp volume: S30 is volume control up to 12 o'clock, with clean mute at zero
        float pluck_bypass = (_pluck > 0.001f) ? 0.25f : 0.0f;
        float master_vol = std::clamp(_vol + pluck_bypass, 0.0f, 1.0f) * (1.0f - 0.25f * starve) * shunt_gnd;

        // Tremolo modulation
        if (mods.tremolo > 0.0f) {
            master_vol *= std::clamp(1.0f - mods.tremolo * 0.85f, 0.0f, 1.0f);
        }

        float blend_rm = std::min(1.0f, mods.ring_mod);
        float rm_carrier = (blend_rm > 0.015f) ? FastTanh(osc_inject * 2.5f) : 0.0f;

        // Headroom boundaries for C1 smooth saturation:
        // Full rail: linear up to 0.85, saturating asymptotically to 1.25
        // Under starve: linear headroom collapses down to ~0.20
        float H = 0.85f * (1.0f - 0.75f * starve);
        float M = 1.25f * (1.0f - 0.50f * starve);

        auto process_channel = [&](float ext_in, DcBlocker& dc_block) -> float {
            float dry = ext_in;
            float amp_core = dry;
            if (blend_rm > 0.015f && std::abs(dry) > 0.01f) {
                float rm_sig = dry * rm_carrier * 2.4f;
                amp_core = (1.0f - blend_rm) * dry + blend_rm * rm_sig;
            }

            // Bridged oscillator injection is scaled by _vol so S30 provides volume control over oscillator voice
            float osc_in = osc_inject * _vol * 1.35f;
            float amp_in = amp_core + osc_in + hum_inject * _vol + pluck_sig + mb_sig;
            float biased_in = amp_in + bias_drift;
            float gated_in = biased_in;

            if (sputter_th > 0.001f && std::abs(amp_in) < sputter_th) {
                // Tier 2 Velcro tearing sputter: rapid non-linear attenuation below threshold
                float ratio = std::abs(amp_in) / sputter_th;
                gated_in = biased_in * (ratio * ratio * ratio);
            } else if (deadzone > 0.001f && std::abs(biased_in) < deadzone) {
                float sgn = (biased_in >= 0.0f) ? 1.0f : -1.0f;
                float mag = std::abs(biased_in);
                gated_in = sgn * (mag * mag) / deadzone;
            }

            float scaled_in = gated_in * drive_eff;
            if (scaled_in < 0.0f && starve > 0.02f) {
                scaled_in *= (1.0f + 1.8f * starve);
            }

            float sat = SaturateSmooth(scaled_in, H, M);
            float out = (sat - bias_null) * comp * master_vol;
            return dc_block.Process(out);
        };

        out_l = process_channel(in_l, _dc_block_l);
        out_r = process_channel(in_r, _dc_block_r);
    }

    float Volume() const { return _vol; }
    float InputVolume() const { return _vol; }
    float InputGain() const { return _vol; }

private:
    static inline float SaturateSmooth(float x, float H, float M) {
        float mag = std::abs(x);
        if (mag <= H) return x;
        float span = M - H;
        float sat = H + span * FastTanh((mag - H) / span);
        return (x >= 0.0f) ? sat : -sat;
    }

    float _sample_rate{48000.0f};
    DcBlocker _dc_block_l;
    DcBlocker _dc_block_r;
    float _pluck{0.0f};
    float _motorboat_phase{0.0f};
    float _vol{1.0f};
    float _target_vol{1.0f};
    float _drive{1.0f};
    float _target_drive{1.0f};
};

} // namespace bends
