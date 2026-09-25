#pragma once

#include <cstdint>

namespace bends {

constexpr uint8_t kPortMainL = 0;
constexpr uint8_t kPortMainR = 1;
constexpr uint8_t kPortAux0  = 2;

// Routing connection between module ports.
//
// Multiple connections targeting the same destination port sum linearly,
// modeling electrical node summing. The gain parameter reflects bridging
// conductance (e.g. skin resistance or conductive object pressure).
// If `is_feedback` is set by Circuit::Compile(), the connection reads from
// a 1-sample unit-delay buffer (z^-1) to resolve cycles.
struct Connection {
    uint8_t src_module  = 0;
    uint8_t src_port    = kPortMainL;
    uint8_t dst_module  = 0;
    uint8_t dst_port    = kPortMainL;
    float   gain        = 1.0f;
    bool    is_feedback = false;
};

} // namespace bends
