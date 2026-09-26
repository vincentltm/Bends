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
        _drone_scale_snap.Init(110.0f);
    }

    void Process(daisy::DaisySeed& hw, synthux::Touch& touch, TouchBender& bender, uint32_t loop_counter) {
        if (!_engine) return;

        bender.Process(touch.pads());

        float clock_val = ProcessKnobs(touch);
        float starve_amount = 1.0f - _engine->GetVcc();

        ProcessDrone(touch, bender);
        ProcessClick(touch, bender, clock_val);
        ProcessHumAndRadio(touch, bender, clock_val, starve_amount);
        ProcessDelayAndBends(touch, bender, clock_val);
        UpdateLed(hw, touch, loop_counter);
    }

private:
    float ProcessKnobs(synthux::Touch& touch) {
        float vcc = touch.knobs().LeftFader();
        _engine->SetVcc(vcc);
        _engine->SetDrive(touch.knobs().s30());

        _engine->SetCutoff(55.0f * std::pow(270.0f, touch.knobs().s31()));
        _engine->SetRes(0.05f + touch.knobs().s32() * 0.88f);

        float s33 = touch.knobs().s33();
        _engine->SetDelayTime(0.0008f + (s33 * s33) * 2.2f);
        _engine->SetFeedback(touch.knobs().s34() * 1.20f);
        _engine->SetMix(touch.knobs().s35());

        // Pitch and master clock with scale snapping
        float clock_val = touch.knobs().RightFader();
        float raw_pitch = 18.35f * std::pow(2.0f, clock_val * 5.0f);
        _engine->SetPitch(_drone_scale_snap.Process(raw_pitch));

        float starve_amount = 1.0f - _engine->GetVcc();
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
        float p02_touch = bender.Pressure(2);
        float p02_dp    = p02_touch - _prev_p02;
        if (p02_dp > 0.07f) {
            _engine->StrikeClick(p02_dp * 2.5f);
        }
        _prev_p02 = p02_touch;

        _engine->SetClickPressure(p02_touch);
        _engine->SetClickPitch(clock_val * clock_val);
        _engine->SetBridgeGain(BridgeId::FilterBpToCutoff, bender.Conductance(1, 2) * 2.2f);

        // Polyrhythmic pulses when bridged to pads 0, 1, 4, 6
        float g_p02_poly = std::max({bender.Conductance(0, 2), bender.Conductance(1, 2),
                                     bender.Conductance(2, 4), bender.Conductance(2, 6)});
        _engine->SetClickPoly(g_p02_poly);

        float g_click_delay = bender.Conductance(2, 6) * 1.5f;
        _engine->SetBridgeGain(BridgeId::ClickToDelayL, g_click_delay);
        _engine->SetBridgeGain(BridgeId::ClickToDelayR, g_click_delay);
    }

    void ProcessHumAndRadio(synthux::Touch& touch, TouchBender& bender, float clock_val, float starve_amount) {
        // P03 audio / hum antenna
        float p03_touch = bender.Pressure(3);
        float p03_prox  = touch.air().PadProximity(3);
        float p03_shunt = bender.Conductance(3, 8);
        _engine->SetDroneAux(1, p03_touch * 0.85f - p03_shunt * 1.5f);

        // P08 chassis ground hum
        float p08_touch = bender.Pressure(8);
        float p08_prox  = touch.air().PadProximity(8);
        float drive_val = _engine->GetDrive();
        float hum_gain  = drive_val * drive_val * 0.55f;
        float p03_buzz  = (p03_touch * 0.25f + p03_prox * 0.08f) * hum_gain;
        float base_hum  = (0.0012f + 0.0025f * starve_amount) * (drive_val * drive_val);
        float p08_hum   = (base_hum + p08_touch * 0.25f + p08_prox * 0.08f) * hum_gain;
        _engine->SetHum(p03_buzz, p08_hum);

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

        // P06 to P08 buffer freeze
        _engine->SetDelayFreeze(bender.Conductance(6, 8) > 0.35f);

        // P07 feedback bridges to filter and starve
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

        // P04 Starve sag
        float starve_sag = bender.Conductance(4, 8) * 1.8f + bender.Pressure(4) * 0.85f + bender.CurrentLoad();
        _engine->SetStarveAux(0, starve_sag);

        // P11 Utility bridge
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
        } else if (_engine->GetVcc() < 0.30f) {
            hw.SetLed((loop_counter % 200) < 40);
        } else {
            hw.SetLed(false);
        }
    }

    BendsEngine* _engine{nullptr};
    float _prev_p02{0.0f};
    float _prev_p09{0.0f};
    uint32_t _p09_strike_cooldown{0};
    MusicalTuning::ScaleSnapper _drone_scale_snap;
};

} // namespace bends
