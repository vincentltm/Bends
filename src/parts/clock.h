#pragma once

#include "dsp/fast_math.h"
#include "bends/mod_pack.h"
#include <cmath>
#include <algorithm>

namespace bends {

class Clock {
public:
    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() {
        _phase = 0.0f;
        _rate = 1.0f;
        _base_rate = 1.0f;
        _beat = false;
        _trig_env = 0.0f;
    }

    inline void PrepareBlock(float knob_clk, float node_pull) {
        float clk_dc = std::clamp(knob_clk + node_pull * 0.35f, 0.0f, 1.0f);
        _base_rate = FastPow2(std::clamp(clk_dc * 11.23f, 0.0f, 11.5f));
    }

    inline void Tick(float v_rail_eff, float fm_input, const ClockMods& mods,
                     uint8_t osc_mode, float osc_sig, float osc_ramp,
                     float& out_clk) {
        float clk_vcc_sag = 0.60f + 0.40f * (v_rail_eff * v_rail_eff);
        float rate = _base_rate * clk_vcc_sag;

        if (std::abs(fm_input) > 0.001f) {
            rate *= FastPow2(fm_input * 0.35f);
        }

        // Rhythmic Oscillator Modulation
        if (mods.rate_mod > 0.001f) {
            if (osc_mode == 2) { // ModeRnd: Generative Musical Tempo Subdivisions
                static constexpr float kSubdivs[8] = { 0.5f, 0.75f, 1.0f, 1.3333f, 1.5f, 2.0f, 3.0f, 4.0f };
                int idx = std::clamp(static_cast<int>((osc_sig + 1.0f) * 3.99f), 0, 7);
                float subdiv = kSubdivs[idx];
                rate *= (1.0f + (subdiv - 1.0f) * std::min(1.0f, mods.rate_mod));
            } else if (osc_mode == 1) { // ModeSaw: Accelerando Ratcheting Ramp
                rate *= (1.0f + mods.rate_mod * osc_ramp * 3.0f);
            } else { // ModeSin: Elastic Swing / Rubato / Audio-rate FM
                rate *= FastPow2(mods.rate_mod * osc_sig * 1.8f);
            }
        }

        // Clock Subdivision Bend (Bend 3: P00 CLK <-> P05 GND)
        if (mods.sub_div > 0.01f) {
            float div_factor = (mods.sub_div > 0.65f) ? (1.0f / 3.0f) : 0.5f;
            rate *= (1.0f - mods.sub_div) + mods.sub_div * div_factor;
        }

        // Phase Hard Sync (Saw flyback reset or explicit sync pulse)
        if (mods.sync) {
            _phase = 0.0f;
            _beat = true;
            _trig_env = 1.0f;
        }

        rate = std::clamp(rate, 0.05f, 12000.0f);
        _rate = rate;
        _phase += rate / _sample_rate;
        _beat = false;

        if (_phase >= 1.0f) {
            _phase -= 1.0f;
            _beat = true;
            _trig_env = 1.0f;
        }

        // Punchy Analog Trigger Pulse + Body Thump (~30ms decay)
        _trig_env *= 0.9968f;
        float click = _trig_env * _trig_env * _trig_env;
        float thump = FastSin(_trig_env * kPi * 2.0f) * _trig_env;
        float punchy_pulse = click * 0.85f + thump * 0.40f - 0.05f;
        out_clk = punchy_pulse * std::min(1.0f, v_rail_eff * 1.5f);
    }

    float Rate() const { return _rate; }
    bool Beat() const { return _beat; }
    float Phase() const { return _phase; }

private:
    float _sample_rate{48000.0f};
    float _phase{0.0f};
    float _rate{1.0f};
    float _base_rate{1.0f};
    bool  _beat{false};
    float _trig_env{0.0f};
};

} // namespace bends
