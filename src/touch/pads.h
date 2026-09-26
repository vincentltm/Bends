#pragma once

#include "daisy_seed.h"
#include <functional>
#include <array>

namespace synthux {

/**
 * MPR121 12-pad capacitive touch driver (P00–P11) with continuous pressure sensing.
 * Connected via I2C1 (SCL: PB8, SDA: PB9, Address: 0x5A).
 */
class Pads {
public:
    // Calibrated max delta thresholds for 1.0 pressure
    static constexpr std::array<float, 12> kDefaultMaxDeltas = {
        220.0f, // P00: Drone
        220.0f, // P01: Filter Ping
        220.0f, // P02: Click
        220.0f, // P03: Hum
        260.0f, // P04: Starve
        230.0f, // P05: Valley Feedback
        230.0f, // P06: Delay Clock
        220.0f, // P07: Radio Whine
        250.0f, // P08: Ground
        250.0f, // P09: Stutter
        220.0f, // P10: Pitch Ribbon
        220.0f  // P11: Utility Bridge
    };

    Pads() : _state{0}, _oor_state{0}, _exponential{false} {
        for (size_t i = 0; i < 12; i++) {
            _pressure[i] = 0.0f;
            _debounce_cnt[i] = 0;
            _pad_max_delta[i] = kDefaultMaxDeltas[i];
        }
    }

    void Init(daisy::DaisySeed& hw);
    void Process();
    void Recalibrate();

    // Response curve: true = quadratic (natural finger pulp resistance), false = linear
    void SetCurveExponential(bool exp) { _exponential = exp; }
    bool IsCurveExponential() const { return _exponential; }

    // Max delta tuning
    void SetMaxDelta(uint16_t pad, float max_delta) { if (pad < 12) _pad_max_delta[pad] = max_delta; }
    float MaxDelta(uint16_t pad) const { return (pad < 12) ? _pad_max_delta[pad] : 0.0f; }

    // Event callbacks
    void SetOnTouch(std::function<void(uint16_t pad)> cb) { _on_touch = cb; }
    void SetOnRelease(std::function<void(uint16_t pad)> cb) { _on_release = cb; }

    // State queries
    bool IsTouched(uint16_t pad) const { return (_state & (1 << pad)) != 0; }
    bool HasTouch() const { return _state > 0; }
    uint16_t State() const { return _state; }
    uint16_t OutOfRange() const { return _oor_state; }

    // Continuous pressure sensing (0.0 .. 1.0)
    float Pressure(uint16_t pad) const { return (pad < 12) ? _pressure[pad] : 0.0f; }
    float operator[](size_t pad) const { return Pressure(pad); }
    const std::array<float, 12>& Pressures() const { return _pressure; }

    // Raw 10-bit ADC data and deltas for proximity / air tracking
    uint16_t Filtered(uint16_t pad) const { return (pad < 12) ? _filtered[pad] : 0; }
    uint16_t Baseline(uint16_t pad) const { return (pad < 12) ? (_has_sw_baseline ? _sw_baseline[pad] : _baseline[pad]) : 0; }
    int32_t RawDelta(uint16_t pad) const { return (pad < 12) ? _raw_delta[pad] : 0; }

    // Baseline locking (freeze baseline tracking to eliminate hover decay)
    void LockBaseline(bool lock);
    bool IsBaselineLocked() const { return _baseline_locked; }
    void CaptureSoftwareBaseline(size_t samples = 16);

private:
    void WriteRegister(uint8_t reg, uint8_t val);
    uint8_t ReadRegister(uint8_t reg);
    bool ReadBurst(uint8_t start_reg, uint8_t* buffer, uint16_t size);

    static constexpr uint8_t kMpr121Addr = 0x5A;

    daisy::I2CHandle _i2c;
    daisy::I2CHandle::Config _i2c_conf{};
    uint8_t _consecutive_errors{0};
    uint32_t _last_poll_ms{0};
    uint16_t _state;
    uint16_t _oor_state;
    bool _exponential;
    bool _baseline_locked{false};
    bool _has_sw_baseline{false};

    std::array<uint16_t, 12> _filtered{};
    std::array<uint16_t, 12> _baseline{};
    std::array<uint16_t, 12> _sw_baseline{};
    std::array<int32_t, 12>  _raw_delta{};
    std::array<float, 12>    _pad_max_delta{};
    std::array<float, 12>    _pressure{};
    std::array<uint8_t, 12>  _debounce_cnt{};

    std::function<void(uint16_t pad)> _on_touch;
    std::function<void(uint16_t pad)> _on_release;
};

} // namespace synthux
