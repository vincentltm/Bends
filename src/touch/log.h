#pragma once

#if DEBUG

#include "daisy_seed.h"

namespace synthux {

class Log {
public:
    static Log& Instance() {
        static Log instance;
        return instance;
    }

    void Init(daisy::DaisySeed* hw) {
        _hw = hw;
        if (_hw) _hw->StartLog();
    }

    template <typename... Args>
    void Print(const char* fmt, Args... args) {
        if (_hw) _hw->PrintLine(fmt, args...);
    }

private:
    Log() : _hw(nullptr) {}
    daisy::DaisySeed* _hw;
};

} // namespace synthux

#define LOG_INIT(hw_ptr) synthux::Log::Instance().Init(hw_ptr)
#define LOG_MSG(msg)     synthux::Log::Instance().Print(msg)
#define LOG_INT(val)     synthux::Log::Instance().Print("%d", static_cast<int>(val))
#define LOG_FLOAT(val)   synthux::Log::Instance().Print("%d", static_cast<int>((val) * 1000))

#else

#define LOG_INIT(hw_ptr) ((void)0)
#define LOG_MSG(msg)     ((void)0)
#define LOG_INT(val)     ((void)0)
#define LOG_FLOAT(val)   ((void)0)

#endif
