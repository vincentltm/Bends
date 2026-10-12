#pragma once

#include <cstddef>
#include <cstring>
#include <algorithm>

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
        // 1-pole lowpass inside allpass loop (~5.5 kHz cutoff at 48kHz)
        damp += 0.55f * (y - damp);
        float z = coef * damp + in;
        buf[pos] = z;
        pos++;
        if (pos >= kDelaySize) pos = 0;
        return damp - coef * z;
    }
};

// 4-stage stereo diffuser tank for ambient spring / plate wash
class DiffuserTank {
public:
    void Init() {
        Reset();
    }

    void Reset() {
        _d1_l.Reset(); _d2_l.Reset(); _d3_l.Reset(); _d4_l.Reset();
        _d1_r.Reset(); _d2_r.Reset(); _d3_r.Reset(); _d4_r.Reset();
    }

    inline void Process(float in_l, float in_r, float& out_l, float& out_r) {
        float s_l = _d1_l.Process(in_l, 0.70f);
        s_l = _d2_l.Process(s_l, 0.62f);
        s_l = _d3_l.Process(s_l, 0.55f);
        out_l = _d4_l.Process(s_l, 0.45f);

        float s_r = _d1_r.Process(in_r, 0.70f);
        s_r = _d2_r.Process(s_r, 0.62f);
        s_r = _d3_r.Process(s_r, 0.55f);
        out_r = _d4_r.Process(s_r, 0.45f);
    }

private:
    // Prime number delay line lengths for decorrelation
    DiffuserStage<149>  _d1_l;
    DiffuserStage<211>  _d2_l;
    DiffuserStage<379>  _d3_l;
    DiffuserStage<563>  _d4_l;

    DiffuserStage<157>  _d1_r;
    DiffuserStage<223>  _d2_r;
    DiffuserStage<397>  _d3_r;
    DiffuserStage<587>  _d4_r;
};

} // namespace bends
