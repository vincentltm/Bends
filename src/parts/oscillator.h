#pragma once

#include "dsp/fast_math.h"
#include "bends/mod_pack.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace bends {

class Oscillator {
public:
    enum Mode : uint8_t {
        ModeSin = 0,
        ModeSaw = 1,
        ModeRnd = 2
    };

    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() {
        _phase = 0.0f;
        _vc = 0.0f;
        _charging = true;
        _saw_ramp = 0.0f;
        _saw_reset_pulse = 0.0f;
        _lfsr = 0xACE1u;
        _rnd_held = 0.0f;
        _sh_step_ratio = 1.0f;
        _sub_state = false;
        _base_freq = 110.0f;
    }

    inline void PrepareBlock(float knob_osc, float air_hover) {
        float u = std::clamp(knob_osc, 0.0f, 1.0f);
        float octaves = u * (22.57f - 10.02f * u);
        _base_freq = std::clamp(0.25f * FastPow2(octaves) * (1.0f + air_hover * 0.08f), 0.1f, 1600.0f);
    }

    inline void PrepareBlock(float knob_osc, float /*node_pull*/, float air_hover) {
        PrepareBlock(knob_osc, air_hover);
    }

    inline void Tick(uint8_t mode, float v_rail_eff, float clk_vcc_sag,
                     float fm_input, const OscillatorMods& mods,
                     float& out_osc, float& out_ramp, float& out_reset_pulse, float& out_sub) {
        float dt = 1.0f / _sample_rate;

        float freq = std::clamp(_base_freq * clk_vcc_sag, 0.1f, 1600.0f);
        float total_fm = fm_input + mods.freq_mod;
        if (std::abs(total_fm) > 0.001f) {
            freq = std::clamp(freq * FastPow2(total_fm * 0.75f), 0.1f, 1600.0f);
        }

        // Hard sync
        if (mods.sync) {
            _phase = 0.0f;
            _vc = 0.0f;
            _charging = true;
        }

        bool cycle_reset = false;
        _saw_reset_pulse = 0.0f;
        float sig = 0.0f;

        if (mode == ModeSaw) {
            // Analog RC relaxation discharge model
            float v_th_hi = 0.65f * v_rail_eff;
            float v_th_lo = 0.30f * v_rail_eff;
            float tau = 1.4236f / std::max(0.1f, freq);
            float k_charge = dt / tau;
            float k_discharge = dt / (0.012f * tau);

            if (v_rail_eff < 0.25f) {
                float choke = (v_rail_eff / 0.25f);
                k_charge *= (choke * choke);
            }

            if (_charging) {
                _vc += (v_rail_eff - _vc) * k_charge;
                if (_vc >= v_th_hi) {
                    _charging = false;
                    _saw_reset_pulse = 1.0f; // Single-sample flyback pulse
                    cycle_reset = true;
                }
            } else {
                _vc -= _vc * k_discharge;
                if (_vc <= v_th_lo) {
                    _charging = true;
                }
            }

            float span = std::max(0.005f, v_th_hi - v_th_lo);
            float norm_vc = std::clamp((_vc - v_th_lo) / span, 0.0f, 1.0f);
            _saw_ramp = norm_vc;
            sig = (2.0f * norm_vc - 1.0f) * std::min(1.0f, v_rail_eff * 1.5f);

            if (cycle_reset) {
                _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
                _rnd_held = (static_cast<float>(_lfsr % 2001) - 1000.0f) / 1000.0f;
                static constexpr float kShSteps[6] = { 1.0f, 1.1892f, 1.3348f, 1.4983f, 2.0f, 0.75f };
                _sh_step_ratio = kShSteps[_lfsr % 6];
            }
        } else {
            _phase += freq * dt;
            if (_phase >= 1.0f) {
                _phase -= 1.0f;
                cycle_reset = true;
                // LFSR step
                _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
                _rnd_held = (static_cast<float>(_lfsr % 2001) - 1000.0f) / 1000.0f;

                // Semitone ratios for S&H arpeggiation (unison, m3, 4th, 5th, oct, 4th down)
                static constexpr float kShSteps[6] = { 1.0f, 1.1892f, 1.3348f, 1.4983f, 2.0f, 0.75f };
                _sh_step_ratio = kShSteps[_lfsr % 6];
            }

            if (mode == ModeSin) {
                sig = FastSin(_phase * kTwoPi) * std::min(1.0f, v_rail_eff * 1.5f);
                _saw_ramp = 0.0f;
            } else {
                sig = _rnd_held * std::min(1.0f, v_rail_eff * 1.5f);
                _saw_ramp = 0.0f;
            }
        }

        if (cycle_reset) {
            _sub_state = !_sub_state;
        }
        float sub_sig = (_sub_state ? 1.0f : -1.0f) * std::min(1.0f, v_rail_eff * 1.5f);

        if (mods.sub_octave > 0.01f) {
            float sub_amt = std::min(1.0f, mods.sub_octave);
            sig = (1.0f - 0.35f * sub_amt) * sig + (0.50f * sub_amt) * sub_sig;
        }

        out_osc = sig;
        out_ramp = _saw_ramp;
        out_reset_pulse = _saw_reset_pulse;
        out_sub = sub_sig;
    }

    float StepRatio() const { return _sh_step_ratio; }

private:
    float _sample_rate{48000.0f};
    float _phase{0.0f};
    float _vc{0.0f};
    bool  _charging{true};
    float _saw_ramp{0.0f};
    float _saw_reset_pulse{0.0f};
    uint16_t _lfsr{0xACE1u};
    float _rnd_held{0.0f};
    float _sh_step_ratio{1.0f};
    bool  _sub_state{false};
    float _base_freq{110.0f};
};

} // namespace bends
