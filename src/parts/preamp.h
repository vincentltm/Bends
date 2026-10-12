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
        _in_vol = 1.0f;
        _drive = 1.0f;
    }

    void TriggerPluck(float strength) {
        _pluck = std::max(_pluck, strength);
    }

    inline void PrepareBlock(float knob_drive, float node_pull) {
        float u = std::clamp(knob_drive + node_pull * 0.20f, 0.0f, 1.0f);
        float vol = (u < 0.04f) ? 0.0f : std::clamp((u - 0.04f) / 0.96f, 0.0f, 1.0f);
        _in_vol = vol * vol; // Audio taper
        _in_gain = (vol * 0.60f + _in_vol * 0.40f) * 1.6f;
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
            mb_sig = 0.040f * starve_depth * mb_pulse * (0.30f + 0.70f * _in_vol);
        }

        float bias_drift = 0.18f * (starve * starve) + mods.bias;
        float deadzone = 0.07f * (starve * starve);
        float touch_drive = touch_amp * 3.0f;

        float target_drive = (0.35f + 7.2f * _in_vol + touch_drive + mods.drive) * (1.0f + 1.4f * starve);
        _drive += 0.025f * (target_drive - _drive);
        float bias_null = FastTanh(bias_drift * _drive);

        // Musical loudness compensation: prevents volume collapse when pushed or starved
        float comp = 1.0f / (1.0f + 0.10f * std::max(0.0f, _drive - 1.0f));

        // Master preamp volume curve: S30 is the primary volume control for external audio & bridged oscillators!
        // When S30 is at zero, output gain and all bridged inputs attenuate cleanly to zero.
        float amp_level = (0.20f * std::sqrt(_in_vol) + 0.80f * _in_vol);
        float pluck_bypass = (_pluck > 0.001f) ? 0.25f : 0.0f;
        float out_gain = comp * std::clamp(amp_level + pluck_bypass, 0.0f, 1.0f) * (1.0f - 0.25f * starve) * shunt_gnd;

        // Tremolo modulation
        if (mods.tremolo > 0.0f) {
            out_gain *= std::clamp(1.0f - mods.tremolo * 0.85f, 0.0f, 1.0f);
        }

        float blend_rm = std::min(1.0f, mods.ring_mod);
        float rm_carrier = (blend_rm > 0.015f) ? FastTanh(osc_inject * 2.5f) : 0.0f;

        auto process_channel = [&](float ext_in, DcBlocker& dc_block) -> float {
            float dry = ext_in * _in_gain;
            float amp_core = dry;
            if (blend_rm > 0.015f && std::abs(dry) > 0.01f) {
                float rm_sig = dry * rm_carrier * 2.4f;
                amp_core = (1.0f - blend_rm) * dry + blend_rm * rm_sig;
            }
            // Bridged oscillator injection is attenuated by S30 so the player has clean volume control over the oscillator voice
            float osc_in = osc_inject * (0.35f * _in_gain + 0.65f * _in_vol) * 1.35f;
            float amp_in = amp_core + osc_in + hum_inject * _in_vol + pluck_sig + mb_sig;
            float biased_in = amp_in + bias_drift;
            float gated_in = biased_in;
            if (std::abs(biased_in) < deadzone) {
                float sgn = (biased_in >= 0.0f) ? 1.0f : -1.0f;
                float mag = std::abs(biased_in);
                gated_in = sgn * (mag * mag) / std::max(0.001f, deadzone);
            }
            float clipped = FastTanh(gated_in * _drive);
            float out = (clipped - bias_null) * out_gain;
            return dc_block.Process(out);
        };

        out_l = process_channel(in_l, _dc_block_l);
        out_r = process_channel(in_r, _dc_block_r);
    }

    float InputVolume() const { return _in_vol; }
    float InputGain() const { return _in_gain; }

private:
    float _sample_rate{48000.0f};
    DcBlocker _dc_block_l;
    DcBlocker _dc_block_r;
    float _pluck{0.0f};
    float _motorboat_phase{0.0f};
    float _in_vol{1.0f};
    float _in_gain{1.0f};
    float _drive{1.0f};
};

} // namespace bends
