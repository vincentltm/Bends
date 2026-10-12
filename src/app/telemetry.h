#pragma once

#include "daisy_seed.h"
#include "surface/touch.h"
#include "app/instrument.h"

namespace bends {

class Telemetry {
public:
    static inline bool enabled{false};

    static void ProcessRx(uint8_t* buff, uint32_t* length) {
        if (buff && length && *length > 0) {
            for (uint32_t i = 0; i < *length; ++i) {
                uint8_t c = buff[i];
                if (c == 'b' || c == 'B' || c == 'd' || c == 'D') {
                    daisy::System::ResetToBootloader(daisy::System::BootloaderMode::STM);
                } else if (c == 't' || c == 'T' || c == '1') {
                    enabled = true;
                } else if (c == 'x' || c == 'X' || c == '0') {
                    enabled = false;
                }
            }
        }
    }

    static void Send(daisy::DaisySeed& hw, synthux::Touch& touch, Instrument& inst, uint32_t frame) {
        uint32_t slot = frame % 4;
        if (slot == 0) {
            hw.PrintLine("P %d %d %d %d %d %d %d %d %d %d %d %d",
                         static_cast<int>(touch.pads().Pressure(0) * 100.0f),
                         static_cast<int>(touch.pads().Pressure(1) * 100.0f),
                         static_cast<int>(touch.pads().Pressure(2) * 100.0f),
                         static_cast<int>(touch.pads().Pressure(3) * 100.0f),
                         static_cast<int>(touch.pads().Pressure(4) * 100.0f),
                         static_cast<int>(touch.pads().Pressure(5) * 100.0f),
                         static_cast<int>(touch.pads().Pressure(6) * 100.0f),
                         static_cast<int>(touch.pads().Pressure(7) * 100.0f),
                         static_cast<int>(touch.pads().Pressure(8) * 100.0f),
                         static_cast<int>(touch.pads().Pressure(9) * 100.0f),
                         static_cast<int>(touch.pads().Pressure(10) * 100.0f),
                         static_cast<int>(touch.pads().Pressure(11) * 100.0f));
        } else if (slot == 1) {
            hw.PrintLine("R %d %d %d %d %d %d %d %d %d %d",
                         static_cast<int>(inst.GetBoard().GetRail().EffectiveVcc() * 100.0f),
                         inst.GetBoard().GetRail().IsBrownout() ? 1 : 0,
                         static_cast<int>(touch.knobs().s30() * 100.0f),
                         static_cast<int>(touch.knobs().s31() * 100.0f),
                         static_cast<int>(touch.knobs().s32() * 100.0f),
                         static_cast<int>(touch.knobs().s33() * 100.0f),
                         static_cast<int>(touch.knobs().s34() * 100.0f),
                         static_cast<int>(touch.knobs().s35() * 100.0f),
                         static_cast<int>(touch.knobs().s36() * 100.0f),
                         static_cast<int>(touch.knobs().s37() * 100.0f));
        } else if (slot == 2) {
            hw.PrintLine("D %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld",
                         static_cast<long>(touch.pads().RawDelta(0)),
                         static_cast<long>(touch.pads().RawDelta(1)),
                         static_cast<long>(touch.pads().RawDelta(2)),
                         static_cast<long>(touch.pads().RawDelta(3)),
                         static_cast<long>(touch.pads().RawDelta(4)),
                         static_cast<long>(touch.pads().RawDelta(5)),
                         static_cast<long>(touch.pads().RawDelta(6)),
                         static_cast<long>(touch.pads().RawDelta(7)),
                         static_cast<long>(touch.pads().RawDelta(8)),
                         static_cast<long>(touch.pads().RawDelta(9)),
                         static_cast<long>(touch.pads().RawDelta(10)),
                         static_cast<long>(touch.pads().RawDelta(11)));
        } else {
            const auto& solver = inst.GetBoard().GetSolver();
            hw.PrintLine("U %d %d %d %d %d %d %d %d %d %d %d %d",
                         static_cast<int>((solver.Pull(NodeClk) * 0.5f + 0.5f) * 100.0f),
                         static_cast<int>((solver.Pull(NodeRst) * 0.5f + 1.0f) * 100.0f),
                         static_cast<int>(inst.GetBoard().GetRail().EffectiveVcc() * 100.0f),
                         static_cast<int>((solver.Pull(NodeOsc) * 0.5f + 0.5f) * 100.0f),
                         static_cast<int>((solver.Pull(NodeFlt) * 0.5f + 0.5f) * 100.0f),
                         static_cast<int>(0), // GND is always 0
                         static_cast<int>((solver.Pull(NodeDly) * 0.5f + 0.5f) * 100.0f),
                         static_cast<int>((solver.Pull(NodeFbk) * 0.5f + 0.5f) * 100.0f),
                         static_cast<int>((solver.Pull(NodeAmp) * 0.5f + 0.5f) * 100.0f),
                         static_cast<int>((solver.Pull(NodeMix) * 0.5f + 0.5f) * 100.0f),
                         static_cast<int>((solver.Pull(NodeIn) * 0.5f + 0.5f) * 100.0f),
                         static_cast<int>((solver.Pull(NodeOut) * 0.5f + 0.5f) * 100.0f));
        }
    }
};

} // namespace bends
