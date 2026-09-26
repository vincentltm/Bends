#pragma once

#include <cmath>
#include "daisysp.h"

namespace bends {

/**
 * Ultra-fast 2^x approximation (~6 CPU cycles on ARM Cortex-M7 vs ~120 cycles for std::pow).
 * Uses integer power of two exponent extraction and a 3rd-order polynomial on [0, 1].
 */
inline float FastPow2(float x) {
    float clamped = daisysp::fclamp(x, -12.0f, 12.0f);
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

} // namespace bends
