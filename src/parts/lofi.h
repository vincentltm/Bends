#pragma once

#include "dsp/fast_math.h"
#include <algorithm>

namespace bends {

// Lo-Fi PT2399 downsampler: clock sample-and-hold (aliasing),
// delta-modulation slew limit, and dual-stage op-amp filter.
struct Lofi {
    float hold_sample{0.0f};
    float clock_phase{0.0f};
    float integrator{0.0f};
    float lp1{0.0f};
    float lp2{0.0f};

    float clock_speed{1.0f};
    float max_slew{0.40f};
    float alpha{0.45f};
    bool  active{false};

    void Init() {
        Reset();
    }

    void Reset() {
        hold_sample = 0.0f;
        clock_phase = 0.0f;
        integrator = 0.0f;
        lp1 = 0.0f;
        lp2 = 0.0f;
        clock_speed = 1.0f;
        max_slew = 0.40f;
        alpha = 0.45f;
        active = true;
    }

    inline void Prepare(float clock_ratio, float sample_rate) {
        clock_speed = std::clamp(clock_ratio, 0.04f, 1.0f);
        active = true;

        if (clock_speed >= 0.96f) {
            max_slew = 10.0f;
            alpha = 0.95f;
        } else {
            max_slew = 1.5f + 8.5f * (clock_speed * clock_speed);
            float cutoff = 8000.0f + 12000.0f * clock_speed;
            alpha = std::clamp(cutoff * (kTwoPi / sample_rate), 0.10f, 0.95f);
        }
    }

    inline float Process(float in) {
        if (!active) {
            integrator = in; lp1 = in; lp2 = in; hold_sample = in;
            return in;
        }

        if (clock_speed >= 0.98f && max_slew > 5.0f) {
            hold_sample = in; integrator = in; lp1 = in; lp2 = in;
            return in;
        }

        // 1. Clock sample-and-hold (downsampling mirror frequencies)
        clock_phase += clock_speed;
        if (clock_phase >= 1.0f) {
            clock_phase -= 1.0f;
            hold_sample = in;
        }

        // 2. Delta modulation slew limit
        float diff = hold_sample - integrator;
        float step = std::clamp(diff, -max_slew, max_slew);
        integrator += step;

        // 3. Dual-stage op-amp lowpass
        lp1 += alpha * (integrator - lp1);
        lp2 += alpha * (lp1 - lp2);
        return lp2;
    }
};

} // namespace bends
