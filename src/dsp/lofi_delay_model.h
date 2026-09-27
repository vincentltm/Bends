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
        // Voltage fader directly controls the delay clock frequency / sample rate!
        // At voltage >= 0.98 (5V full rail): full 48 kHz pristine clean delay (bypassed).
        // As voltage drops towards 0V: clock downsampling drops from 48 kHz down to ~3.8 kHz,
        // producing rich, metallic, gritty PT2399-style aliasing without any Doppler pitch wobble!
        if (voltage >= 0.98f) {
            active = false;
            return;
        }

        active = true;

        // Quadratic fader curve gives precise, musical control over the aliasing sweet spots:
        // v = 0.90 -> ~39 kHz (subtle tape warmth)
        // v = 0.60 -> ~18 kHz (vintage 12-bit sampler crunch)
        // v = 0.35 -> ~8.5 kHz (PT2399 metallic ringing chime)
        // v = 0.15 -> ~4.8 kHz (heavy 8-bit grit)
        // v = 0.00 -> ~3.8 kHz (extreme lo-fi aliased repeats)
        float delay_scale = 1.0f + daisysp::fclamp((delay_sec - 0.20f) / 3.0f, 0.0f, 0.4f);
        float v_curved = voltage * voltage;
        float min_clock = 0.08f / delay_scale;
        clock_speed = daisysp::fclamp(min_clock + (1.0f - min_clock) * v_curved, 0.05f, 1.0f);

        // Delta modulation slew limit: slope overload creates classic PT2399 1-bit compression and bite
        // Slew limit scales with clock speed so aliasing stair-steps retain punchy transients
        float comb_blend = daisysp::fclamp((0.065f - delay_sec) / 0.020f, 0.0f, 1.0f);
        float base_slew = 0.14f * (1.0f - comb_blend) + 0.30f * comb_blend;
        max_slew = base_slew + 0.40f * (clock_speed * clock_speed);

        // Dual-stage op-amp low-pass reconstruction filter (2.8 kHz to 7.5 kHz)
        // Tracks clock speed to keep metallic aliasing sidebands bright and audible while removing supersonic noise
        float cutoff = 2800.0f + 4700.0f * clock_speed;
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
