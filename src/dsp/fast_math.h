#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>
#include "daisysp.h"

namespace bends {

static constexpr float kPi    = 3.14159265358979323846f;
static constexpr float kTwoPi = 6.28318530717958647692f;

inline float FlushDenormal(float val) {
    return (std::abs(val) < 1e-15f) ? 0.0f : val;
}

inline float FastTanh(float x) {
    if (x < -3.0f) return -1.0f;
    if (x > 3.0f)  return 1.0f;
    return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

// Ultra-fast 2^x approximation for ARM Cortex-M7 (~5 cycles, 0 soft-float libm calls)
inline float FastPow2(float x) {
    float clamped = daisysp::fclamp(x, -30.0f, 30.0f);
    int i = static_cast<int>(clamped);
    float f = clamped - static_cast<float>(i);
    if (f < 0.0f) {
        f += 1.0f;
        i -= 1;
    }
    // Minimax polynomial for 2^f on [0, 1]
    float p = 1.0f + f * (0.693019f + f * (0.241404f + f * 0.052032f));
    union {
        uint32_t u;
        float f;
    } pun;
    pun.u = static_cast<uint32_t>((i + 127) << 23);
    return p * pun.f;
}

// Fast parabolic sine approximation on [-pi, pi] (~6 cycles)
inline float FastSin(float rad) {
    float x = rad * (1.0f / kTwoPi);
    x -= static_cast<int>(x + (x >= 0.0f ? 0.5f : -0.5f));
    x *= kTwoPi;

    constexpr float B = 4.0f / kPi;
    constexpr float C = -4.0f / (kPi * kPi);
    float y = B * x + C * x * std::abs(x);
    constexpr float P = 0.225f;
    return P * (y * std::abs(y) - y) + y;
}

inline float FastCos(float rad) {
    return FastSin(rad + (0.5f * kPi));
}

// Fast tan(x) polynomial approximation for filter pre-warping on [0, 1.25]
inline float FastTan(float x) {
    float x2 = x * x;
    return x * (1.0f + x2 * (0.3333333f + x2 * (0.1333333f + x2 * 0.0539683f)));
}

// Fast DAC quantization with hardware float-to-int rounding (no libc std::round)
inline float QuantizeDac(float in, float levels, float inv, float dnl) {
    float scaled = in * levels;
    float rounded = static_cast<float>(static_cast<int32_t>(scaled + (scaled >= 0.0f ? 0.5f : -0.5f)));
    float q = rounded * inv;
    float sgn = (in >= 0.0f) ? 1.0f : -1.0f;
    q += sgn * dnl * inv * (1.0f - std::abs(in));
    return daisysp::fclamp(q, -1.0f, 1.0f);
}

} // namespace bends
