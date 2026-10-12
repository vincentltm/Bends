#pragma once

#include "dsp/fast_math.h"
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
    }

    inline void PrepareBlock(float knob_clk, float node_pull) {
        float clk_dc = std::clamp(knob_clk + node_pull * 0.35f, 0.0f, 1.0f);
        _base_rate = FastPow2(std::clamp(clk_dc * 11.23f, 0.0f, 11.5f));
    }

    inline void Tick(float v_rail_eff, float fm_input, float& out_clk) {
        float clk_vcc_sag = 0.60f + 0.40f * (v_rail_eff * v_rail_eff);
        float rate;
        if (std::abs(fm_input) > 0.001f) {
            float clk_fm = fm_input * 0.35f;
            rate = _base_rate * FastPow2(clk_fm) * clk_vcc_sag;
        } else {
            rate = _base_rate * clk_vcc_sag;
        }
        _rate = rate;
        _phase += rate / _sample_rate;
        _beat = false;

        if (_phase >= 1.0f) {
            _phase -= 1.0f;
            _beat = true;
        }

        // Bipolar clock pulse waveform
        out_clk = (_phase < 0.28f) ? 0.65f : -0.25f;
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
};

} // namespace bends
