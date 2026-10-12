#pragma once

#include "daisy_seed.h"
#include "surface/touch.h"
#include "surface/frame.h"
#include "surface/gestures.h"
#include "board/board.h"
#include <cmath>

namespace bends {

class Instrument {
public:
    void Init(float sample_rate, float* delay_buf_l, float* delay_buf_r, size_t max_delay = Delay::kMaxDelay) {
        _sample_rate = sample_rate;
        _board.Init(sample_rate, delay_buf_l, delay_buf_r, max_delay);

        _tracker_p10.Init();
        _tracker_p11.Init();

        for (size_t i = 0; i < 12; ++i) {
            _rest_scans[i] = 10;
        }

        _led_brightness = 0.0f;
        _led_pwm_accum = 0.0f;
        _latch_flash = 0;
        _latch_pulse_phase = 0.0f;
    }

    // Called from main() while(1) loop (~1 kHz, runs I2C without blocking audio)
    void ProcessSurface(synthux::Touch& touch, bool new_scan) {
        ControlFrame& frame = _frame_buffer.BeginWrite();

        // 1. Read Switches
        // Switch A (Left): Up = SIN, Down = RND, Center = SAW
        if (touch.switches().IsAUp()) {
            frame.osc_mode = Oscillator::ModeSin;
        } else if (touch.switches().IsADown()) {
            frame.osc_mode = Oscillator::ModeRnd;
        } else {
            frame.osc_mode = Oscillator::ModeSaw;
        }

        // Switch B (Right): Up = Freeze, Down = Latch, Center = Live
        frame.is_frozen = touch.switches().IsBUp();
        frame.is_latched = touch.switches().IsBDown();

        // 2. Read Knobs & Faders (Symmetrical Delay Pedal + Lateral Flanking Benders)
        // Knobs S30..S35: The Complete Delay Pedal Signal Chain
        frame.s30_amp = touch.knobs().s30(); // Knob 1: Preamp Drive & Master Voice Volume (Pad 08 AMP)
        frame.s32_flt = touch.knobs().s31(); // Knob 2: Filter Cutoff / Tone (Pad 04 FLT)
        frame.s36_clk = touch.knobs().s32(); // Knob 3: Delay Clock Rate / Tape Speed (Pad 00 CLK)
        frame.s33_dly = touch.knobs().s33(); // Knob 4: Delay Time (Pad 06 DLY)
        frame.s34_fbk = touch.knobs().s34(); // Knob 5: Delay Feedback Regen (Pad 02 FBK)
        frame.s35_mix = touch.knobs().s35(); // Knob 6: Master Wet/Dry Mix (Pad 09 MIX)

        // Faders S36 & S37: The Flanking Benders
        frame.s31_osc = touch.knobs().s36(); // Left Fader: Probe Oscillator Pitch (Pad 03 OSC - Left Wing)
        frame.s37_vcc = touch.knobs().s37(); // Right Fader: System Power Rail / Starve (Pad 07 VCC - Right Wing)

        // 3. Read Touch Pads & Proximities
        for (size_t i = 0; i < 12; ++i) {
            frame.pressures[i]   = touch.pads().Pressure(i);
            frame.proximities[i] = touch.air().PadProximity(i);
        }

        // 4. Contact Mic Physical Gestures
        frame.tap_vel_p10 = 0.0f;
        frame.tap_vel_p11 = 0.0f;
        _tracker_p10.Update(frame.pressures[NodeIn], frame.tap_vel_p10);
        _tracker_p11.Update(frame.pressures[NodeOut], frame.tap_vel_p11);
        frame.rub_speed_p10 = _tracker_p10.rub_speed;
        frame.rub_speed_p11 = _tracker_p11.rub_speed;

        if (frame.tap_vel_p10 > 0.0f) _board.GetContactMic().TriggerTap(frame.tap_vel_p10, false);
        if (frame.tap_vel_p11 > 0.0f) _board.GetContactMic().TriggerTap(frame.tap_vel_p11, true);

        // 5. Standalone Pad Plucks and Pings
        frame.tap_amp = false;
        frame.tap_flt = false;
        frame.tap_rst = false;

        if (new_scan) {
            for (size_t i = 0; i < 12; ++i) {
                float p = frame.pressures[i];
                if (p <= 0.05f) {
                    if (_rest_scans[i] < 255) _rest_scans[i]++;
                } else {
                    if (_rest_scans[i] >= 3 && p > 0.14f) {
                        if (i == NodeAmp) {
                            frame.tap_amp = true;
                            _board.GetPreamp().TriggerPluck(0.85f * (frame.s30_amp + 0.25f));
                        } else if (i == NodeFlt) {
                            frame.tap_flt = true;
                            _board.GetFilter().TriggerPing(0.75f);
                        } else if (i == NodeRst) {
                            frame.tap_rst = true;
                        }
                    }
                    _rest_scans[i] = 0;
                }
            }
        }

        frame.frame_index++;
        _frame_buffer.CommitWrite();

        UpdateLedState(frame.is_latched);
    }

    // Called from 750 Hz hardware audio DMA interrupt
    void ProcessBlock(const float* in_l, const float* in_r,
                      float* out_l, float* out_r,
                      size_t size) {
        const ControlFrame& frame = _frame_buffer.ReadLatest();
        _board.ProcessBlock(in_l, in_r, out_l, out_r, size, frame);
    }

    // Flicker-free delta-sigma PWM for Daisy Seed onboard LED
    void UpdateLedPwm(daisy::DaisySeed& hw) {
        _led_pwm_accum += _led_brightness;
        if (_led_pwm_accum >= 1.0f) {
            _led_pwm_accum -= 1.0f;
            hw.SetLed(true);
        } else {
            hw.SetLed(false);
        }
    }

    Board& GetBoard() { return _board; }
    const Board& GetBoard() const { return _board; }

private:
    void UpdateLedState(bool is_latched) {
        float vcc = _board.GetRail().EffectiveVcc();
        bool beat = _board.ClkBeat();

        float target_b = vcc * 0.25f;
        if (beat) {
            target_b = std::min(1.0f, target_b + 0.65f);
        }

        if (_latch_flash > 0) {
            _latch_flash--;
            target_b = 1.0f;
        } else if (is_latched && _board.GetContacts().HasLatchedBridges()) {
            _latch_pulse_phase += 0.035f;
            if (_latch_pulse_phase >= kTwoPi) _latch_pulse_phase -= kTwoPi;
            float pulse = 0.5f + 0.5f * FastSin(_latch_pulse_phase);
            target_b = std::max(target_b, 0.15f + 0.35f * pulse);
        }

        _led_brightness += 0.25f * (target_b - _led_brightness);
    }

    float _sample_rate{48000.0f};
    Board _board;
    FrameBuffer _frame_buffer;

    PadGestureTracker _tracker_p10;
    PadGestureTracker _tracker_p11;
    uint8_t _rest_scans[12]{10};

    float _led_brightness{0.0f};
    float _led_pwm_accum{0.0f};
    uint16_t _latch_flash{0};
    float _latch_pulse_phase{0.0f};
};

} // namespace bends
