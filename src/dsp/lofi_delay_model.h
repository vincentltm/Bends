#pragma once

#include "daisysp.h"

namespace bends {

// Lo-fi digital delay model: clock sample-and-hold (aliasing),
// delta-modulation slew limit, and dual-stage op-amp filter.
struct LoFiDelayModel {
    float hold_sample{0.0f};
    float clock_phase{0.0f};
    float integrator{0.0f};
    float lp1{0.0f};
    float lp2{0.0f};

    float clock_speed{1.0f};
    float max_slew{0.40f};
    float alpha{0.45f};
    bool active{false};

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
        active = false;
    }

    inline void Prepare(float delay_sec, float voltage, float sample_rate) {
        // PT2399 clock behavior:
        // As delay time increases past 100ms, the internal VCO clock slows down,
        // producing signature metallic aliasing sidebands.
        // Voltage starvation further drags the clock down into deep lo-fi crunch.
        float time_factor = daisysp::fclamp((delay_sec - 0.08f) / 0.32f, 0.0f, 1.0f);
        float base_clock = 1.0f - 0.65f * (time_factor * time_factor); // 1.0 down to 0.35

        float v_norm = daisysp::fclamp(voltage, 0.02f, 1.0f);
        float v_curved = v_norm * v_norm;

        clock_speed = daisysp::fclamp(base_clock * (0.05f + 0.95f * v_curved), 0.04f, 1.0f);
        active = (clock_speed < 0.98f);

        // Delta modulation slew limit: slope overload creates classic PT2399 1-bit compression and bite
        float comb_blend = daisysp::fclamp((0.065f - delay_sec) / 0.020f, 0.0f, 1.0f);
        float base_slew = 0.14f * (1.0f - comb_blend) + 0.30f * comb_blend;
        max_slew = base_slew + 0.40f * (clock_speed * clock_speed);

        // Dual-stage op-amp low-pass reconstruction filter (2.6 kHz to 6.8 kHz)
        // Tracks clock speed to keep metallic aliasing sidebands warm and musical
        float cutoff = 2600.0f + 4200.0f * clock_speed;
        alpha = daisysp::fclamp(cutoff * (6.2831853f / sample_rate), 0.02f, 0.85f);
    }

    inline float Process(float in) {
        if (!active) {
            integrator = in;
            lp1 = in;
            lp2 = in;
            hold_sample = in;
            return in;
        }

        // 1. Clock sample-and-hold (downsampling aliasing)
        clock_phase += clock_speed;
        if (clock_phase >= 1.0f) {
            clock_phase -= 1.0f;
            hold_sample = in;
        }

        // 2. Delta modulation slew limit (soft slope overload compression)
        float diff = hold_sample - integrator;
        float step = daisysp::fclamp(diff, -max_slew, max_slew);
        integrator += step;

        // 3. Dual-stage op-amp low-pass filter
        lp1 += alpha * (integrator - lp1);
        lp2 += alpha * (lp1 - lp2);
        return lp2;
    }
};

} // namespace bends
