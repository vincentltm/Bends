#include "knobs.h"

using namespace synthux;
using namespace daisy;

void Knobs::Init(DaisySeed& hw) {
    // S30..S35 are pots on A0..A5; S36, S37 are faders on A6, A7
    Pin pins[8] = {
        seed::A0, seed::A1, seed::A2, seed::A3,
        seed::A4, seed::A5, seed::A6, seed::A7
    };

    AdcChannelConfig cfg[8];
    for (int i = 0; i < 8; i++) {
        cfg[i].InitSingle(pins[i]);
    }
    hw.adc.Init(cfg, 8);

    // Polled in main loop at ~250 Hz (I2C + 1ms delay)
    constexpr float kKnobPollRate = 250.0f;
    for (int i = 0; i < 8; i++) {
        // Use 40ms slew filter (instead of 2ms which clamped filter coeff to 1.0)
        _knobs[i].Init(hw.adc.GetPtr(i), kKnobPollRate, false, false, 0.040f);
        float v = _knobs[i].Process();
        _filtered[i] = v;
        _values[i] = v;
    }
}

void Knobs::Process() {
    constexpr float kDeadband = 0.0035f; // 0.35% deadband window to reject ADC LSB jitter
    for (int i = 0; i < 8; i++) {
        float raw = _knobs[i].Process();
        // 2-pole cascade smoothing
        _filtered[i] += 0.25f * (raw - _filtered[i]);

        // Deadband hysteresis: only update when movement exceeds noise floor
        float diff = _filtered[i] - _values[i];
        if (std::abs(diff) > kDeadband) {
            _values[i] += 0.35f * diff;
        } else if (_filtered[i] < 0.005f) {
            _values[i] = 0.0f; // Clean floor
        } else if (_filtered[i] > 0.995f) {
            _values[i] = 1.0f; // Clean ceiling
        }
    }
}
