#pragma once

#include "dsp/fast_math.h"
#include <cmath>
#include <algorithm>
#include <cstdint>

namespace bends {

class Hum {
public:
    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() {
        _smooth_gnd_hum = 0.0f;
        _phase = 0.0f;
        _crackle_rng = 13579u;
        _crackle_filter = 0.0f;
    }

    inline void Tick(float v_rail_eff, float starve, float standalone_p_gnd, float gnd_prox,
                     float touched_nodes_sum, float& out_hum) {
        float eff_prox = std::max(0.0f, (gnd_prox - 0.10f) / 0.90f);

        float gnd_touch_hum = standalone_p_gnd * 0.045f;
        float hover_hum = (eff_prox * eff_prox) * 0.015f;
        float v_starve_hum = std::clamp((0.72f - v_rail_eff) / 0.55f, 0.0f, 1.0f) * 0.020f;

        float hum_target = gnd_touch_hum + hover_hum + v_starve_hum;
        _smooth_gnd_hum += (hum_target - _smooth_gnd_hum) * 0.008f;

        if (_smooth_gnd_hum < 0.0002f) {
            out_hum = 0.0f;
            return;
        }

        _phase += 50.0f * kTwoPi / _sample_rate;
        if (_phase >= kTwoPi) _phase -= kTwoPi;

        float sin1 = FastSin(_phase);
        float cos1 = FastCos(_phase);
        float sin2 = 2.0f * sin1 * cos1;
        float sin3 = sin1 * (3.0f - 4.0f * sin1 * sin1);

        float harmonic_brightness = 0.20f + 0.40f * (standalone_p_gnd + eff_prox);
        float hum_composite = sin1 * 0.70f + (sin2 * 0.22f + sin3 * 0.08f) * harmonic_brightness;
        float buzz = FastTanh(hum_composite);

        // Electrostatic crackle generator
        _crackle_rng = _crackle_rng * 1664525u + 1013904223u;
        float white = static_cast<float>(static_cast<int32_t>(_crackle_rng)) * 4.6566129e-10f;
        float crackle = (white > 0.96f) ? (white - 0.96f) * 4.0f : ((white < -0.97f) ? (white + 0.97f) * 3.0f : 0.0f);
        _crackle_filter += (crackle - _crackle_filter) * 0.20f;

        float starve_buzz = buzz + (buzz * buzz - 0.5f) * starve * 0.50f;
        float gnd_hum_audio = (starve_buzz * 0.90f + _crackle_filter * 0.10f) * _smooth_gnd_hum;

        float body_antenna_mult = 1.0f + std::min(0.5f, touched_nodes_sum * 0.5f);
        out_hum = gnd_hum_audio * body_antenna_mult;
    }

private:
    float _sample_rate{48000.0f};
    float _smooth_gnd_hum{0.0f};
    float _phase{0.0f};
    uint32_t _crackle_rng{13579u};
    float _crackle_filter{0.0f};
};

} // namespace bends
