#pragma once

#include <cstdint>

namespace bends {

class Module {
public:
    virtual ~Module() = default;

    virtual void Init(float sample_rate, uint16_t block_size) = 0;
    virtual void Reset() = 0;

    virtual void Process(float in_l, float in_r, float& out_l, float& out_r) = 0;

    virtual uint8_t     NumParams() const = 0;
    virtual const char* GetParamName(uint8_t id) const = 0;
    virtual void        SetParam(uint8_t id, float value) = 0;
    virtual float       GetParam(uint8_t id) const = 0;

    virtual float GetAuxOutput(uint8_t /*channel*/) const { return 0.0f; }
    virtual void  SetAuxInput(uint8_t /*channel*/, float /*value*/) {}

    virtual const char* Name() const = 0;
};

} // namespace bends
