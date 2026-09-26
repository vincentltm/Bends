#pragma once

#include <cstdint>

namespace bends {

enum class Port : uint8_t {
    MainL = 0,
    MainR = 1,
    Aux0  = 2,
    Aux1  = 3
};

constexpr uint8_t kPortMainL = 0;
constexpr uint8_t kPortMainR = 1;
constexpr uint8_t kPortAux0  = 2;

// Connection between module ports.
struct Connection {
    uint8_t src_module  = 0;
    uint8_t src_port    = 0;
    uint8_t dst_module  = 0;
    uint8_t dst_port    = 0;
    float   gain        = 1.0f;
    bool    is_feedback = false;

    constexpr Connection() = default;

    constexpr Connection(uint8_t src_m, uint8_t src_p, uint8_t dst_m, uint8_t dst_p, float g = 1.0f)
        : src_module(src_m), src_port(src_p), dst_module(dst_m), dst_port(dst_p), gain(g) {}

    template <typename ModT, typename PortT>
    constexpr Connection(ModT src_m, PortT src_p, ModT dst_m, PortT dst_p, float g = 1.0f)
        : src_module(static_cast<uint8_t>(src_m)),
          src_port(static_cast<uint8_t>(src_p)),
          dst_module(static_cast<uint8_t>(dst_m)),
          dst_port(static_cast<uint8_t>(dst_p)),
          gain(g) {}
};

} // namespace bends
