#pragma once

#include "dsp/fast_math.h"
#include <cmath>
#include <algorithm>

namespace bends {

// 1-pole DC blocker (highpass filter ~3 Hz at 48 kHz)
struct DcBlocker {
    float x1{0.0f};
    float y1{0.0f};

    inline void Reset() {
        x1 = 0.0f;
        y1 = 0.0f;
    }

    inline float Process(float in) {
        if (std::isnan(in) || std::isinf(in)) in = 0.0f;
        float out = in - x1 + 0.9996f * y1;
        out = FlushDenormal(out);
        x1 = in;
        y1 = out;
        return out;
    }
};

// Simple 1-pole lowpass filter
struct OnePole {
    float state{0.0f};

    inline void Reset(float initial = 0.0f) {
        state = initial;
    }

    inline float Process(float in, float alpha) {
        state += alpha * (in - state);
        state = FlushDenormal(state);
        return state;
    }
};

// Parameter smoother for control-rate slewing (zipper noise elimination)
struct ParameterSmoother {
    float target{0.0f};
    float current{0.0f};
    float coeff{0.008f};

    inline void Init(float initial = 0.0f, float slew = 0.008f) {
        target = initial;
        current = initial;
        coeff = slew;
    }

    inline void SetTarget(float t) {
        target = t;
    }

    inline float Tick() {
        current += coeff * (target - current);
        current = FlushDenormal(current);
        return current;
    }

    inline float Value() const {
        return current;
    }
};

} // namespace bends
