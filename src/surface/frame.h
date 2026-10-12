#pragma once

#include <cstdint>
#include <atomic>
#include <algorithm>

namespace bends {

struct ControlFrame {
    // 12 capacitive touch pad pressures and proximities
    float pressures[12]{0.0f};
    float proximities[12]{0.0f};

    // Knobs & Faders
    float s30_amp{0.35f};  // S30 Pot: Preamp Drive
    float s31_osc{0.25f};  // S31 Pot: Oscillator Pitch
    float s32_flt{0.65f};  // S32 Pot: Filter Cutoff
    float s33_dly{0.50f};  // S33 Pot: Delay Time
    float s34_fbk{0.40f};  // S34 Pot: Delay Feedback
    float s35_mix{0.50f};  // S35 Pot: Master Wet/Dry

    float s36_clk{0.15f};  // S36 Left Fader: Master Clock
    float s37_vcc{1.00f};  // S37 Right Fader: Power Rail

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
