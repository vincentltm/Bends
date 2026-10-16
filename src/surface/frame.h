#pragma once

#include <cstdint>
#include <atomic>
#include <algorithm>

namespace bends {

struct ControlFrame {
    // 12 capacitive touch pad pressures and proximities
    float pressures[12]{0.0f};
    float proximities[12]{0.0f};

    // Knobs & Faders (PCB Silkscreen 1:1)
    float s30_amp{0.35f};  // S30 Pot: Preamp Drive (Pad 08 AMP)
    float s31_flt{0.65f};  // S31 Pot: Filter Cutoff (Pad 04 FLT)
    float s32_clk{0.15f};  // S32 Pot: Master Delay Clock (Pad 00 CLK)
    float s33_dly{0.50f};  // S33 Pot: Delay Time (Pad 06 DLY)
    float s34_fbk{0.40f};  // S34 Pot: Delay Feedback (Pad 02 FBK)
    float s35_mix{0.50f};  // S35 Pot: Master Wet/Dry (Pad 09 MIX)

    float s36_osc{0.25f};  // S36 Left Fader: Probe Oscillator Pitch (Pad 03 OSC)
    float s37_vcc{1.00f};  // S37 Right Fader: Power Rail / Starve (Pad 07 VCC)

    // Hardware Switches
    uint8_t osc_mode{1};   // 0 = Sin, 1 = Saw, 2 = Rnd
    bool    is_frozen{false};
    bool    is_latched{false};

    // Contact Mic Physical Gestures
    float tap_vel_p10{0.0f};
    float tap_vel_p11{0.0f};
    float rub_speed_p10{0.0f};
    float rub_speed_p11{0.0f};

    // Standalone pad taps
    bool tap_amp{false};
    bool tap_flt{false};
    float tap_vel_flt{0.0f};
    bool tap_rst{false};

    uint32_t frame_index{0};
};

// Lock-free triple buffer for race-free communication between main loop (I2C) and audio ISR
class FrameBuffer {
public:
    FrameBuffer() {
        _write_idx = 0;
        _published_idx.store(1, std::memory_order_relaxed);
        _read_idx = 2;
    }

    // Called only by main loop thread
    ControlFrame& BeginWrite() {
        return _frames[_write_idx];
    }

    void CommitWrite() {
        // Swap write index with published index
        uint8_t old_pub = _published_idx.exchange(_write_idx, std::memory_order_release);
        _write_idx = old_pub;
    }

    // Called only by audio ISR (ProcessBlock)
    const ControlFrame& ReadLatest() {
        uint8_t pub = _published_idx.load(std::memory_order_acquire);
        if (pub != _read_idx) {
            _read_idx = pub;
        }
        return _frames[_read_idx];
    }

private:
    ControlFrame _frames[3];
    uint8_t      _write_idx{0};
    uint8_t      _read_idx{2};
    std::atomic<uint8_t> _published_idx{1};
};

} // namespace bends
