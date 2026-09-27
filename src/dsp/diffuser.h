#pragma once

#include <cstddef>
#include <cstring>

namespace bends {

// Schroeder/Dattorro all-pass diffuser stage in internal SRAM
template <size_t kDelaySize>
struct DiffuserStage {
    float buf[kDelaySize]{0.0f};
    size_t pos{0};
    float damp{0.0f};

    void Reset() {
        std::memset(buf, 0, sizeof(buf));
        pos = 0;
        damp = 0.0f;
    }

    inline float Process(float in, float coef) {
        float y = buf[pos];
        // 1-pole lowpass filter inside allpass loop (~5.5 kHz cutoff at 48kHz)
        // Absorbs harsh metallic transients on every reflection
        damp += 0.55f * (y - damp);
        float z = coef * damp + in;
        buf[pos] = z;
        pos++;
        if (pos >= kDelaySize) pos = 0;
        return damp - coef * z;
    }
};

} // namespace bends
