#pragma once

#include "daisy_seed.h"
#include "touch/touch.h"
#include "engine/bends_engine.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

class BendsController {
public:
    void Init(BendsEngine* engine) {
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
        // Switch A (Left): Up = SIN, Down = RND, Center = SAW
        uint8_t osc_mode = SynthCircuit::OSC_SAW;
        if (touch.switches().IsAUp()) {
            osc_mode = SynthCircuit::OSC_SIN;
        } else if (touch.switches().IsADown()) {
            osc_mode = SynthCircuit::OSC_RND;
        }

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
        _engine->SetSwitches(osc_mode, _is_frozen);

        // Read Pad Pressures and Air Proximities
        float pressures[12];
        float proximities[12];
        for (size_t i = 0; i < 12; ++i) {
            pressures[i]   = touch.pads().Pressure(i);
            proximities[i] = touch.air().PadProximity(i);
        }
        _engine->SetPressures(pressures);
        _engine->SetProximities(proximities);

        // 3. Update Resistor Node Network
        float current_rail = _engine->GetRail().EffectiveVcc();
        _engine->Net().Process(touch.pads(),
                               touch.air(),
                               current_rail,
                               _bend_depth,
                               _is_latched);

        // Tap Pad 1 (RST) while in Latch mode to add currently held live bridges to the latch set!
        float rst_p = touch.pads().Pressure(NodeRst);
        if (_is_latched && rst_p > 0.15f && _prev_rst_p <= 0.05f) {
            _engine->Net().LatchLiveBridges();
            _latch_flash = 120; // ~160ms bright confirmation flash
        }
        _prev_rst_p = rst_p;

        // 4. Update Power Rail
        float total_load = _engine->Net().Frame().total_current_load;
        float vcc_pull   = _engine->Net().Pull(NodeVcc);
        _engine->GetRail().ProcessControl(s37_vcc, total_load, vcc_pull);

        // 5. Update Onboard LED State
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
    void UpdateLed(uint32_t /*loop_counter*/) {
        float vcc = _engine->GetRail().EffectiveVcc();
        bool beat = _engine->Synth().ClkBeat();

        // Base brightness reflects power rail health
        float target_b = vcc * 0.25f;

        // Flash brighter on clock beat
        if (beat) {
            target_b = std::min(1.0f, target_b + 0.65f);
        }

        // Latch visual feedback:
        if (_latch_flash > 0) {
            _latch_flash--;
            target_b = 1.0f; // bright confirmation flash on stacking
        } else if (_is_latched && _engine->Net().HasLatchedBridges()) {
            // Gentle rhythmic breathing pulse while bridges are latched in memory
            _latch_pulse_phase += 0.035f;
            if (_latch_pulse_phase >= 6.2831853f) _latch_pulse_phase -= 6.2831853f;
            float pulse = 0.5f + 0.5f * std::sin(_latch_pulse_phase);
            target_b = std::max(target_b, 0.15f + 0.35f * pulse);
        }

        // Fast attack, smooth decay
        _led_brightness += 0.25f * (target_b - _led_brightness);
    }

    BendsEngine* _engine{nullptr};
    bool  _is_latched{false};
    bool  _is_frozen{false};
    float _bend_depth{1.0f};

    float _prev_rst_p{0.0f};
    uint16_t _latch_flash{0};
    float _latch_pulse_phase{0.0f};

    float _led_brightness{0.0f};
    float _led_pwm_accum{0.0f};
};

} // namespace bends
