#pragma once

#include "daisy_seed.h"
#include "touch/touch.h"
#include "engine/bends_engine.h"
#include "engine/touch_bender.h"
#include "core/musical_tuning.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

// Maps Touch 2 pads, knobs, and proximity to synth parameters.
class BendsController {
public:
    void Init(BendsEngine* engine) {
        _engine = engine;
        _prev_p02 = 0.0f;
        _prev_p09 = 0.0f;
        _p02_strike_cooldown = 0;
        _p09_strike_cooldown = 0;
        _was_a_up = false;
        _was_a_down = false;
        _drone_scale_snap.Init(110.0f);
    }

    void Process(daisy::DaisySeed& hw, synthux::Touch& touch, TouchBender& bender, uint32_t loop_counter) {
        if (!_engine) return;

        bender.Process(touch.pads());

        ProcessSwitchA(touch);
        float clock_val = ProcessKnobs(touch, bender);
        float starve_amount = 1.0f - _engine->GetVcc();

        ProcessDrone(touch, bender);
        ProcessClick(touch, bender, clock_val);
        ProcessHumAndRadio(touch, bender, clock_val, starve_amount);
        ProcessDelayAndBends(touch, bender, clock_val);
        UpdateLed(hw, touch, loop_counter);
    }

private:
    void ProcessSwitchA(synthux::Touch& touch) {
        bool a_up = touch.switches().IsAUp();
        bool a_down = touch.switches().IsADown();

        // Switch A Up: Dedicated buffer freeze
        if (a_up && !_was_a_up) {
            _engine->SetDelayFreeze(true);
        } else if (!a_up && _was_a_up) {
            _engine->SetDelayFreeze(false);
        }

        // Switch A Down: Enter floating / warming-up circuit state
        if (a_down && !_was_a_down) {
            _engine->EnterFloatingState();
        } else if (!a_down && _was_a_down) {
            // Flipped back to Center: Clean reset!
            // Clears delay buffers, flushes feedback energy, resets connections & offsets
            _engine->CleanReset();
        }

        _was_a_up = a_up;
        _was_a_down = a_down;
    }

    float ProcessKnobs(synthux::Touch& touch, TouchBender& bender) {
        float vcc = touch.knobs().LeftFader();

        float drive_raw = touch.knobs().s30();
        float cutoff_raw = 55.0f * std::pow(270.0f, touch.knobs().s31());
        float res_raw = 0.05f + touch.knobs().s32() * 0.95f; // Reaches 1.0 for singing self-oscillation!
        daisysp::fonepole(_smooth_s33, touch.knobs().s33(), 0.15f);
        float s33 = _smooth_s33;
        float fb_raw = touch.knobs().s34() * 1.20f;
        float mix_raw = touch.knobs().s35();

        _current_res = res_raw;
        _current_cutoff = cutoff_raw;

        // S30 Drive load on the power rail: cranking drive pulls extra current from Vcc (~6% sag at max)
        float drive_draw = (drive_raw * drive_raw) * 0.06f;
        float current_draw = bender.CurrentLoad() + drive_draw;
        float effective_vcc = daisysp::fclamp(vcc - current_draw * 0.15f, 0.02f, 1.0f);
        _engine->SetVcc(effective_vcc);

        // Entanglement factor: scales with touch load AND starve depth
        float starve_amount = 1.0f - effective_vcc;
        _current_starve = starve_amount;
        float entangle = daisysp::fclamp(
            current_draw * 1.5f * (0.3f + starve_amount * 0.7f),
            0.0f, 0.20f
        );

        _engine->SetDrive(drive_raw);
        _engine->SetCutoff(cutoff_raw);
        _engine->SetRes(res_raw);

        // S33 Delay Time curve: fine musical control (0.8ms - 1.5s) over first 75%,
        // then smoothly opens up to 8.0s long cassette tape loop in the final 20-25%
        float s33_sq = s33 * s33;
        float s33_quad = s33_sq * s33_sq;
        float s33_oct = s33_quad * s33_quad;
        float delay_time_sec = 0.0008f + s33_sq * 1.50f + s33_oct * 6.50f;
        _engine->SetDelayTime(delay_time_sec);
        _engine->SetFeedback(fb_raw);
        _engine->SetMix(mix_raw);

        // Sideways knob leakage when entangled:
        // Drive leaks into filter resonance drive
        _engine->SetFilterAux(1, drive_raw * entangle * 0.4f);
        // Cutoff leaks into delay damping
        _engine->SetDelayAux(1, cutoff_raw * entangle * 0.00002f);
        // Feedback leaks into filter self-oscillation tendency
        _engine->SetFilterAux(1, fb_raw * entangle * 0.15f);
        // Delay time leaks into click timing
        _engine->SetClickTimeLeak(s33 * entangle);

        // Pitch and master clock with scale snapping
        daisysp::fonepole(_smooth_clock_fader, touch.knobs().RightFader(), 0.15f);
        float clock_val = _smooth_clock_fader;
        _engine->SetDelayClock(clock_val);
        float raw_pitch = 18.35f * std::pow(2.0f, clock_val * 5.0f);
        _engine->SetPitch(_drone_scale_snap.Process(raw_pitch));

        touch.air().SetNoiseGate(1.1f - starve_amount * 0.4f);
        touch.air().SetMaxProximity(18.0f - starve_amount * 8.0f);

        return clock_val;
    }

    void ProcessDrone(synthux::Touch& touch, TouchBender& bender) {
        float p00_touch = bender.Pressure(0);
        float p00_prox  = touch.air().PadProximity(0);

        if (p00_prox > 0.05f && p00_touch <= 0.02f) {
            _engine->SetFilterAux(0, p00_prox * 0.5f);
        }

        // Pad 0 pressure opens drone volume
        float p00_norm = daisysp::fclamp((p00_touch - 0.025f) / 0.975f, 0.0f, 1.0f);
        _engine->SetVca(p00_norm * p00_norm);
        _engine->SetSub(0.25f + p00_touch * 0.75f);

        // P00 to P08 ground fault pitch drop
        float pitch_drop = bender.Conductance(0, 8) * 1.5f;
        if (pitch_drop > 0.05f) {
            _engine->SetDroneAux(0, -pitch_drop * 1.5f);
        }

        // P10 pitch modulation
        float p10_touch = bender.Pressure(10);
        if (p10_touch > 0.05f) {
            _engine->SetDroneAux(0, p10_touch * 1.0f);
        }
    }

    void ProcessClick(synthux::Touch& /*touch*/, TouchBender& bender, float clock_val) {
        if (_p02_strike_cooldown > 0) {
            _p02_strike_cooldown--;
        }
        float p02_touch = bender.Pressure(2);
        float p02_dp    = p02_touch - _prev_p02;
        bool strike_triggered = false;
        if (_p02_strike_cooldown == 0 && p02_dp > 0.13f) {
            _engine->StrikeClick(p02_dp * 2.5f);
            _p02_strike_cooldown = 140; // 140ms debounce window
            strike_triggered = true;
        }
        _prev_p02 = p02_touch;

        _engine->SetClickPressure(p02_touch);
        _engine->SetClickPitch(clock_val * clock_val);

        // P02<->P06: Delay subdivision sync (smooth crossfade with conductance)
        float g_delay_sync = bender.Conductance(2, 6);
        _engine->SetClickDelaySync(g_delay_sync);

        // P01<->P02: Click excites resonant filter ping and redirects direct output
        float g_filter_ping = bender.Conductance(1, 2);
        // Only ping the filter if there is actual physical bridging between P01 and P02
        float total_ping = g_filter_ping * (1.0f + _current_res * 0.50f);

        if (total_ping > 0.05f) {
            _engine->SetClickFilterBlend(total_ping);
            if (strike_triggered) {
                _engine->PingFilter(p02_dp * 2.0f * total_ping);
            }
        } else {
            _engine->SetClickFilterBlend(0.0f);
        }
        _engine->SetBridgeGain(BridgeId::FilterBpToCutoff, total_ping * 1.8f);

        // P02<->P08: Ground fault beat drops
        float g_ground = bender.Conductance(2, 8);
        _engine->SetClickGroundFault(g_ground);

        // P00<->P02: Drone pitch sync
        float g_drone = bender.Conductance(0, 2);
        _engine->SetClickDroneSync(g_drone);

        // Polyrhythmic pulses when bridged to pads 0, 1, 4, 6
        float g_p02_poly = std::max({bender.Conductance(0, 2), bender.Conductance(1, 2),
                                     bender.Conductance(2, 4), bender.Conductance(2, 6)});
        _engine->SetClickPoly(g_p02_poly);

        float g_click_delay = bender.Conductance(2, 6) * 1.5f;
        _engine->SetBridgeGain(BridgeId::ClickToDelayL, g_click_delay);
        _engine->SetBridgeGain(BridgeId::ClickToDelayR, g_click_delay);
    }

    void ProcessHumAndRadio(synthux::Touch& touch, TouchBender& bender, float clock_val, float starve_amount) {
        // P03 audio / hum antenna with chassis grounding & reverb diffusion
        float p03_touch = bender.Pressure(3);
        float p03_prox  = touch.air().PadProximity(3);
        float p03_shunt = bender.Conductance(3, 8);
        _engine->SetDroneAux(1, p03_touch * 0.85f - p03_shunt * 1.5f);

        // Chassis grounding: hovering / light contact (< 0.15) picks up mains buzz/crackle.
        // As finger contact firms up, chassis grounding smoothly silences the hum.
        float raw_ground = daisysp::fclamp((p03_touch - 0.08f) / 0.50f, 0.0f, 1.0f);
        float grounding = raw_ground * raw_ground;
        if (p03_shunt > 0.05f) {
            grounding = std::max(grounding, daisysp::fclamp(p03_shunt * 1.5f, 0.0f, 1.0f));
        }

        // P08 chassis ground hum
        float p08_touch = bender.Pressure(8);
        float p08_prox  = touch.air().PadProximity(8);
        float drive_val = _engine->GetDrive();
        float hum_gain  = drive_val * drive_val * 0.55f;
        // Buzz is active on light touch / antenna hover, but extinguished as chassis is grounded
        float p03_buzz  = (p03_touch * 0.25f + p03_prox * 0.08f) * hum_gain * (1.0f - grounding);
        float touch_total = bender.CurrentLoad();
        float base_hum  = (touch_total > 0.005f) ? ((0.0008f + 0.0018f * starve_amount) * (drive_val * drive_val)) : 0.0f;
        float p08_hum   = (base_hum + p08_touch * 0.25f + p08_prox * 0.08f) * hum_gain;
        _engine->SetHum(p03_buzz, p08_hum);

        // Capacitive body grounding on P03 introduces all-pass diffusion into the delay loop (reverb wash!)
        // Smooth, gradual quadratic curve across the entire touch range (0.05 to 0.95)
        float touch_norm = daisysp::fclamp((p03_touch - 0.05f) / 0.88f, 0.0f, 1.0f);
        float touch_diff = touch_norm * touch_norm; // Organic quadratic response
        float shunt_diff = p03_shunt * p03_shunt * 0.85f;
        float diffusion_amt = daisysp::fclamp(touch_diff * 0.92f + shunt_diff, 0.0f, 0.92f);
        _engine->SetDelayDiffusion(diffusion_amt);

        // P07 radio whine
        float p07_touch = bender.Pressure(7);
        float p07_prox  = touch.air().PadProximity(7);
        float p07_presence = p07_touch + p07_prox;
        if (p07_presence > 0.02f) {
            float base_rf = 293.66f * std::pow(2.0f, clock_val * 4.0f);
            float rf_freq = base_rf / (1.0f + p07_touch * 3.5f + p07_prox * 1.5f);
            float rf_amp  = daisysp::fclamp(p07_touch * 0.035f + p07_prox * 0.020f, 0.0f, 0.05f);
            _engine->SetWhine(rf_amp, rf_freq);
        } else {
            _engine->SetWhine(0.0f, 1200.0f);
        }
    }

    void ProcessDelayAndBends(synthux::Touch& touch, TouchBender& bender, float clock_val) {
        _engine->SetGlitchDensity(clock_val);

        // P09 stutter and memory glitch
        float p09_touch = bender.Pressure(9);
        float p09_prox  = touch.air().PadProximity(9);

        if (_p09_strike_cooldown > 0) {
            _p09_strike_cooldown--;
        }
        float p09_dp = p09_touch - _prev_p09;
        if (_p09_strike_cooldown == 0 && p09_dp > 0.16f) {
            _engine->TriggerBufferSkip(p09_dp * 1.5f);
            _p09_strike_cooldown = 150;
        }
        _prev_p09 = p09_touch;

        float delay_mangle = daisysp::fclamp(p09_touch * 0.85f + p09_prox * 0.25f, 0.0f, 1.0f);
        _engine->SetDelayMangle(delay_mangle);

        float g_adjacent = std::max({bender.Conductance(7, 9), bender.Conductance(6, 9), bender.Conductance(5, 9)});
        float packet_loss_rate = daisysp::fclamp(p09_touch * 0.25f + g_adjacent * 1.1f, 0.0f, 1.0f);
        _engine->SetDelayGlitch(packet_loss_rate);

        // Switch A Up: dedicated buffer freeze
        bool a_up = touch.switches().IsAUp();
        _engine->SetDelayFreeze(a_up);

        // P07 feedback bridges to filter and starve (ONLY active when explicitly touched!)
        float g_delay_feed = std::max(bender.Conductance(7, 5) * 0.35f, bender.Pressure(7) * 0.20f);
        _engine->SetBridgeGain(BridgeId::DelayToFilterL, g_delay_feed);
        _engine->SetBridgeGain(BridgeId::DelayToFilterR, g_delay_feed);
        _engine->SetBridgeGain(BridgeId::DelayToCutoff, bender.Conductance(7, 1) * 1.1f);
        float g_delay_starve = bender.Conductance(7, 4) * 0.60f;
        _engine->SetBridgeGain(BridgeId::DelayToStarveL, g_delay_starve);
        _engine->SetBridgeGain(BridgeId::DelayToStarveR, g_delay_starve);

        // S&H chorus modulation
        float vcc = _engine->GetVcc();
        float lower_half_starve = daisysp::fclamp((0.50f - vcc) * 2.0f, 0.0f, 1.0f);
        float subtle_mod = daisysp::fclamp(
            bender.Conductance(1, 6) * 1.3f +
            bender.Conductance(4, 6) * 1.3f +
            bender.Conductance(2, 6) * 0.8f +
            lower_half_starve * 0.45f,
            0.0f, 1.0f
        );
        _engine->SetDelaySubtleMod(subtle_mod);

        // P06 Clock bend & P00 to P06 FM bridge
        float p06_prox = touch.air().PadProximity(6);
        float clock_bend = bender.Conductance(6, 8) * 1.8f + bender.Pressure(6) * 0.80f + p06_prox * 0.20f;
        _engine->SetDelayAux(0, clock_bend);
        _engine->SetBridgeGain(BridgeId::DroneToDelayClock, bender.Conductance(0, 6) * 0.65f);

        // P04 Starve sag + high resonance dynamic pull on power rail
        float res_sag = (_current_res * _current_res) * _current_starve * 0.15f;
        float starve_sag = bender.Conductance(4, 8) * 1.8f + bender.Pressure(4) * 0.85f + bender.CurrentLoad() + res_sag;
        _engine->SetStarveAux(0, starve_sag);

        // Pad 05: Dedicated Delay Reverse (touch pressure + conductance bridging)
        // In normal delay: crossfades into click-free dual-head reverse delay
        // In freeze mode: flips tape direction smoothly (+1.0 -> -1.0) with analog tape inertia
        float p05_touch = bender.Pressure(5);
        float p05_prox  = touch.air().PadProximity(5);
        float p05_bridge = std::max({
            bender.Conductance(5, 6),  // P06 (Clock) <-> P05 (Reverse)
            bender.Conductance(5, 9),  // P09 (Stutter/Glitch) <-> P05 (Reverse)
            bender.Conductance(5, 4),  // P04 (Starve) <-> P05 (Reverse)
            bender.Conductance(5, 8),  // P08 (Ground) <-> P05 (Reverse)
            bender.Conductance(5, 7),  // P07 (Radio) <-> P05 (Reverse)
            bender.Conductance(5, 1)   // P01 (Filter) <-> P05 (Reverse)
        });
        float reverse_amt = daisysp::fclamp(p05_touch * 1.2f + p05_prox * 0.3f + p05_bridge * 1.5f, 0.0f, 1.0f);
        _engine->SetDelayReverse(reverse_amt);

        // P11 Utility starve bridge
        float p11_touch = bender.Pressure(11);
        if (p11_touch > 0.05f) {
            _engine->SetBridgeGain(BridgeId::DelayToStarveL, p11_touch * 0.9f);
            _engine->SetBridgeGain(BridgeId::DelayToStarveR, p11_touch * 0.9f);
        }
    }

    void UpdateLed(daisy::DaisySeed& hw, synthux::Touch& touch, uint32_t loop_counter) {
        bool any_active = touch.pads().HasTouch() || touch.air().HasAir();
        if (any_active) {
            hw.SetLed(true);
        } else if (_engine->IsFloatingActive()) {
            // Pulse smoothly with drift while floating / warming up
            float progress = _engine->GetWarmupProgress();
            uint32_t period = static_cast<uint32_t>(200.0f - progress * 90.0f);
            if (period < 50) period = 50;
            hw.SetLed((loop_counter % period) < (period / 2));
        } else if (_engine->GetVcc() < 0.30f) {
            hw.SetLed((loop_counter % 200) < 40);
        } else {
            hw.SetLed(false);
        }
    }

    BendsEngine* _engine{nullptr};
    float _prev_p02{0.0f};
    float _prev_p09{0.0f};
    uint32_t _p02_strike_cooldown{0};
    uint32_t _p09_strike_cooldown{0};
    bool _was_a_up{false};
    bool _was_a_down{false};
    MusicalTuning::ScaleSnapper _drone_scale_snap;
    float _current_res{0.05f};
    float _current_cutoff{1000.0f};
    float _current_starve{0.0f};
    float _smooth_s33{0.3f};
    float _smooth_clock_fader{0.5f};
};

} // namespace bends
