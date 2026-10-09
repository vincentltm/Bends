#pragma once

#include "daisy_seed.h"
#include "touch/touch.h"
#include "engine/bends_engine_v2.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

class BendsControllerV2 {
public:
    void Init(BendsEngineV2* engine) {
        _engine = engine;
        _is_latched = false;
        _is_frozen = false;
        _bend_depth = 1.0f;
        _led_brightness = 0.0f;
        _led_pwm_accum = 0.0f;
    }

    // Call once per audio block (750 Hz)
    void Process(daisy::DaisySeed& /*hw*/, synthux::Touch& touch, uint32_t loop_counter) {
        if (!_engine) return;

        // 1. Process Hardware Switches
        // Switch B (Right): Up = Freeze, Down = Latch, Center = Live
        _is_frozen  = touch.switches().IsBUp();
        _is_latched = touch.switches().IsBDown();

        // 2. Read Faders & Knobs
        float s30_amp = touch.knobs().s30(); // S30: AMP Drive
        float s31_osc = touch.knobs().s31(); // S31: OSC Freq
        float s32_flt = touch.knobs().s32(); // S32: FLT Cutoff
        float s33_dly = touch.knobs().s33(); // S33: DLY Time
        float s34_fbk = touch.knobs().s34(); // S34: FBK Regeneration
        float s35_mix = touch.knobs().s35(); // S35: MIX Wet/Dry

        float s36_clk = touch.knobs().s36(); // S36 (Left Fader): CLK Rate
        float s37_vcc = touch.knobs().s37(); // S37 (Right Fader): VCC Rail

        _engine->SetKnobs(s30_amp, s31_osc, s32_flt, s33_dly, s34_fbk, s35_mix, s36_clk);

        // 3. Process Gestures (tap strikes, hold, solo touches)
        _engine->Gest().Process(touch.pads());

        // 4. Update Resistor Node Network
        float current_rail = _engine->GetRail().EffectiveVcc();
        _engine->Net().Process(touch.pads(),
                               touch.air(),
                               current_rail,
                               _bend_depth,
                               _is_latched);

        // 5. Update Power Rail
        float total_load = _engine->Net().Frame().total_current_load;
        float vcc_pull   = _engine->Net().Pull(NodeVcc);
        _engine->GetRail().ProcessControl(s37_vcc, total_load, vcc_pull);

        // 6. Update Master Clock
        float clk_pull = _engine->Net().Pull(NodeClk);
        float clk_load = _engine->Net().Load(NodeClk);
        _engine->Clk().ProcessControl(s36_clk, current_rail, clk_pull, clk_load);

        // 7. Update Onboard LED State
        UpdateLed(loop_counter);
    }

    // Flicker-free 750 Hz block-rate delta-sigma PWM for Daisy Seed onboard LED
    void UpdateLedPwm(daisy::DaisySeed& hw) {
        _led_pwm_accum += _led_brightness;
        if (_led_pwm_accum >= 1.0f) {
            _led_pwm_accum -= 1.0f;
            hw.SetLed(true);
        } else {
            hw.SetLed(false);
        }
    }

    float GetBendDepth() const { return _bend_depth; }
    void  SetBendDepth(float d) { _bend_depth = std::clamp(d, 0.1f, 3.0f); }

private:
    void UpdateLed(uint32_t loop_counter) {
        float vcc = _engine->GetRail().EffectiveVcc();
        bool beat = _engine->Clk().BeatTrigger();

        // Base brightness reflects power rail health
        float target_b = vcc * 0.25f;

        // Flash brighter on clock beat
        if (beat) {
            target_b = std::min(1.0f, target_b + 0.65f);
        }

        // Fast attack, smooth decay
        _led_brightness += 0.25f * (target_b - _led_brightness);

        // Periodic debug logging over USB serial every ~750 frames (approx 1 sec)
        if ((loop_counter % 750) == 0) {
            uint8_t bridges = _engine->Net().Frame().active_bridge_count;
            if (bridges > 0) {
                // When bridges are active, log the touch event count & rail
                // (keeps serial output quiet when idle)
            }
        }
    }

    BendsEngineV2* _engine{nullptr};
    bool  _is_latched{false};
    bool  _is_frozen{false};
    float _bend_depth{1.0f};

    float _led_brightness{0.0f};
    float _led_pwm_accum{0.0f};
};

} // namespace bends
