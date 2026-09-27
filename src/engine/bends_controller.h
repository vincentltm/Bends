#pragma once

#include "daisy_seed.h"
#include "touch/touch.h"
#include "engine/bends_engine.h"
#include "engine/touch_bender.h"
#include "core/musical_tuning.h"
#include "core/fast_math.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

// Maps Touch 2 pads, knobs, and proximity to synth parameters.
class BendsController {
public:
    void Init(BendsEngine* engine) {
        _engine = engine;
        _prev_p01 = 0.0f;
        _prev_p02 = 0.0f;
        _prev_p09 = 0.0f;
        _p02_strike_cooldown = 0;
        _p09_strike_cooldown = 0;
        _was_a_up = false;
        _was_a_down = false;
        _drone_scale_snap.Init(110.0f);
    }

    void Process(daisy::DaisySeed& /*hw*/, synthux::Touch& touch, TouchBender& bender, uint32_t /*loop_counter*/) {
        if (!_engine) return;

        bender.Process(touch.pads());

        ProcessSwitchA(touch);
        float drone_pitch_val = ProcessKnobs(touch, bender);
        float starve_amount = 1.0f - _engine->GetVcc();
        float drive_val = _engine->GetDrive();
        // Master internal generator drive: when S30 Drive is turned low, internal generators
        // (P00 drone, P01 corrupt, P02 click, P03 hum, P07 radio) are muted so clean external
        // input audio can be processed through filter, starve, delay, and bends without interference!
        float gen_drive = daisysp::fclamp((drive_val - 0.03f) / 0.97f, 0.0f, 1.0f);

        ProcessDrone(touch, bender, gen_drive);
        ProcessCorrupt(touch, bender, drone_pitch_val, starve_amount, gen_drive);
        ProcessClick(touch, bender, drone_pitch_val, gen_drive);
        ProcessHumAndRadio(touch, bender, drone_pitch_val, starve_amount, gen_drive);
        ProcessDelayAndBends(touch, bender, drone_pitch_val);
        UpdateLedBrightness(touch, bender);
    }

    // Flicker-free 750 Hz block-rate delta-sigma PWM for Daisy Seed onboard LED
    void UpdateLedPwm(daisy::DaisySeed& hw) {
        _led_pwm_accum += _smooth_led_brightness;
        if (_led_pwm_accum >= 1.0f) {
            _led_pwm_accum -= 1.0f;
            hw.SetLed(true);
        } else {
            hw.SetLed(false);
        }
    }

private:
    void ProcessSwitchA(synthux::Touch& touch) {
        bool a_up   = touch.switches().IsAUp();
        bool a_down = touch.switches().IsADown();
        bool b_down = touch.switches().IsBDown();
        bool any_down = a_down || b_down;

        if (_first_switch_read) {
            _first_switch_read = false;
            _was_a_up = a_up;
            _was_a_down = any_down;
            if (a_up) {
                // Boot in Freeze: clean freeze on whatever is in buffer
                _engine->SetDelayFreeze(true);
            }
            return;
        }

        if (any_down) {
            // While held Down: keep Down-to-Freeze gesture timer primed (~350ms in 1ms ticks)
            _down_to_freeze_counter = 280;
            if (!_was_a_down) {
                // Instantly wipe runaway audio & engage chassis bootup drone + clicks + sprinkled memory
                _engine->EnterFloatingState();
            }
        } else {
            if (_down_to_freeze_counter > 0) {
                _down_to_freeze_counter--;
            }
            if (_was_a_down) {
                // Released Down position:
                // If not flipped directly to UP (Freeze), execute clean reset so normal play begins fresh
                if (!a_up) {
                    _engine->CleanReset();
                }
            }
        }

        // Switch A Up: Dedicated clean buffer freeze
        if (a_up && !_was_a_up) {
            _engine->SetDelayFreeze(true);
            _down_to_freeze_counter = 0;
        } else if (!a_up && _was_a_up) {
            _engine->SetDelayFreeze(false);
        }

        _was_a_up = a_up;
        _was_a_down = any_down;
    }

    float ProcessKnobs(synthux::Touch& touch, TouchBender& bender) {
        float vcc = touch.knobs().LeftFader();

        float drive_raw = touch.knobs().s30();
        float cutoff_raw = 55.0f * FastPow2(touch.knobs().s31() * 8.0769f); // 55 * 2^(s31 * log2(270))
        float res_raw = 0.05f + touch.knobs().s32() * 0.95f; // Reaches 1.0 for self-oscillation
        // S33 Delay Time deadband: prevents ADC noise jitter from wiggling delay time when knob is untouched
        float raw_s33 = touch.knobs().s33();
        if (std::abs(raw_s33 - _last_stable_s33) > 0.0035f) {
            _last_stable_s33 = raw_s33;
        }
        daisysp::fonepole(_smooth_s33, _last_stable_s33, 0.25f);
        float s33 = _smooth_s33;
        float fb_raw = touch.knobs().s34() * 1.20f;
        float mix_raw = touch.knobs().s35();

        _current_res = res_raw;
        _current_cutoff = cutoff_raw;
        _current_fb = touch.knobs().s34();

        // S30 Drive load on the power rail: cranking drive pulls extra current from Vcc (~6% sag at max)
        float drive_draw = (drive_raw * drive_raw) * 0.06f;
        float current_draw = bender.CurrentLoad() + drive_draw;
        float effective_vcc = daisysp::fclamp(vcc - current_draw * 0.15f, 0.02f, 1.0f);
        _engine->SetVcc(effective_vcc);

        // Left Fader (Voltage): Controls power rail starve AND delay clock downsampling!
        // Sweeps sample rate from 48 kHz clean down into crunchy PT2399 aliasing without Doppler wobble.
        _engine->SetDelayClock(vcc);

        // Entanglement factor: scales with touch load and starve depth
        float starve_amount = 1.0f - effective_vcc;
        _current_starve = starve_amount;
        float entangle = daisysp::fclamp(
            current_draw * 1.5f * (0.3f + starve_amount * 0.7f),
            0.0f, 0.20f
        );

        _engine->SetDrive(drive_raw);
        _engine->SetCutoff(cutoff_raw);
        _engine->SetRes(res_raw);

        // S33 Delay Time curve:
        // 0.00 - 0.15: Comb resonator & flanger (0.8ms to 45ms)
        // 0.15 - 0.40: Slapback to standard musical echo (45ms to 650ms)
        // 0.40 - 0.70: Ambient tape echo wash (650ms to 2.4s)
        // 0.70 - 1.00: Cassette tape loop & sound-on-sound (2.4s to 9.5s)
        float s33_sq = s33 * s33;
        float delay_time_sec = 0.0008f + s33 * 0.040f + s33_sq * 1.50f + (s33_sq * s33) * 7.96f;
        _engine->SetDelayTime(delay_time_sec);
        _engine->SetFeedback(fb_raw);
        _engine->SetMix(mix_raw);

        // Sideways knob leakage when entangled:
        // Cutoff leaks into delay damping
        _engine->SetDelayAux(1, cutoff_raw * entangle * 0.00002f);
        // Drive leaks into filter resonance drive, feedback leaks into filter self-oscillation tendency
        _engine->SetFilterAux(1, drive_raw * entangle * 0.4f + fb_raw * entangle * 0.15f);
        // Delay time leaks into click timing
        _engine->SetClickTimeLeak(s33 * entangle);

        // Right Fader: Dedicated Drone Pitch with D Dorian scale snapping
        daisysp::fonepole(_smooth_clock_fader, touch.knobs().RightFader(), 0.15f);
        float drone_pitch_val = _smooth_clock_fader;
        float raw_pitch = 18.35f * FastPow2(drone_pitch_val * 5.0f);
        _engine->SetPitch(_drone_scale_snap.Process(raw_pitch));

        touch.air().SetNoiseGate(1.1f - starve_amount * 0.4f);
        touch.air().SetMaxProximity(18.0f - starve_amount * 8.0f);

        return drone_pitch_val;
    }

    void ProcessDrone(synthux::Touch& touch, TouchBender& bender, float gen_drive) {
        float p00_touch = bender.Pressure(0);
        float p00_prox  = touch.air().PadProximity(0);

        if (p00_prox > 0.05f && p00_touch <= 0.02f) {
            _engine->SetFilterAux(0, p00_prox * 0.5f);
        }

        // Pad 0 pressure opens drone volume:
        // Responsive sensitivity curve that picks up early on light metal while maintaining dynamic range
        // Muted when S30 Drive is low
        float p00_norm = daisysp::fclamp((p00_touch - 0.008f) / 0.992f, 0.0f, 1.0f);
        float p00_vca  = p00_norm * (0.35f + 0.65f * p00_norm) * gen_drive;
        _engine->SetVca(p00_vca);
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

    void ProcessCorrupt(synthux::Touch& touch, TouchBender& bender, float drone_pitch_val, float starve_amount, float gen_drive) {
        float p01_touch = bender.Pressure(1);
        float p01_prox  = touch.air().PadProximity(1);

        // Touch attack triggers a new corrupted memory personality and anchor
        if (p01_touch > 0.02f && _prev_p01 <= 0.02f) {
            _engine->TriggerCorruptPersonality();
        }
        _prev_p01 = p01_touch;

        // Gentle volume curve: never loud or harsh (max 0.28), muted when S30 is low
        float p01_norm = daisysp::fclamp((p01_touch - 0.015f) / 0.985f, 0.0f, 1.0f);
        float p01_air  = daisysp::fclamp((p01_prox - 0.08f) / 0.80f, 0.0f, 1.0f) * 0.10f;
        float corrupt_vol = (p01_norm > 0.0f || p01_air > 0.0f) ? (0.05f + 0.23f * p01_norm + p01_air) * gen_drive : 0.0f;
        _engine->SetCorruptVolume(corrupt_vol);
        _engine->SetCorruptPressure(p01_norm);
        _engine->SetCorruptPitch(drone_pitch_val);

        // Pad 1 circuit bend interactions:
        _engine->SetCorruptClickMod(bender.Conductance(1, 2));     // P01 <-> P02 Click pings
        _engine->SetCorruptReverseMod(bender.Conductance(1, 5));   // P01 <-> P05 Reverse scan
        _engine->SetCorruptClockMod(bender.Conductance(1, 6));     // P01 <-> P06 Clock FM
        _engine->SetCorruptDiffusionMod(bender.Conductance(1, 8)); // P01 <-> P08 Ground / Diffusion
        _engine->SetCorruptGlitchMod(bender.Conductance(1, 9));    // P01 <-> P09 S&H / Glitch
        _engine->SetCorruptStarveMod(daisysp::fclamp(bender.Conductance(1, 4) * 1.5f + starve_amount, 0.0f, 1.0f)); // P01 <-> P04 Starve
        _engine->SetCorruptDroneMod(bender.Conductance(0, 1));     // P01 <-> P00 Drone modulation
    }

    void ProcessClick(synthux::Touch& /*touch*/, TouchBender& bender, float clock_val, float gen_drive) {
        if (_p02_strike_cooldown > 0) {
            _p02_strike_cooldown--;
        }
        float p02_touch = bender.Pressure(2);
        float p02_dp    = p02_touch - _prev_p02;
        bool strike_triggered = false;
        // Sensitive strike threshold (0.04f) catches light metal taps & coin drops
        if (_p02_strike_cooldown == 0 && p02_dp > 0.04f) {
            float strike_vel = daisysp::fclamp(p02_dp * 3.2f + 0.08f, 0.08f, 1.0f);
            if (gen_drive > 0.01f) {
                _engine->StrikeClick(strike_vel * gen_drive);
            }
            _p02_strike_cooldown = 110; // 110ms debounce window
            strike_triggered = true;
        }
        _prev_p02 = p02_touch;

        _engine->SetClickPressure(p02_touch * gen_drive);
        _engine->SetClickPitch(clock_val);
        _engine->SetClickRate(clock_val);

        // Geiger scaling: metronome when high voltage and unbridged, Poisson Geiger clicks when starved or bridged to P01/P04/P08
        float cond_geiger = std::max({bender.Conductance(1, 2), bender.Conductance(2, 4), bender.Conductance(2, 8)});
        _engine->SetClickGeigerBend(cond_geiger);

        // P02<->P06: Delay subdivision sync (smooth crossfade with conductance)
        float cond_delay_sync = bender.Conductance(2, 6);
        _engine->SetClickDelaySync(cond_delay_sync);

        // P01<->P02: Click excites resonant filter ping and redirects direct output
        float cond_filter_ping = bender.Conductance(1, 2);
        // Only ping the filter if there is actual physical bridging between P01 and P02
        float total_ping = cond_filter_ping * (1.0f + _current_res * 0.50f);

        if (total_ping > 0.05f) {
            _engine->SetClickFilterBlend(total_ping);
            if (strike_triggered && gen_drive > 0.01f) {
                _engine->PingFilter(p02_dp * 2.0f * total_ping * gen_drive);
            }
        } else {
            _engine->SetClickFilterBlend(0.0f);
        }
        _engine->SetBridgeGain(BridgeId::FilterBpToCutoff, total_ping * 1.8f);

        // P02<->P08: Ground fault beat drops
        float cond_ground = bender.Conductance(2, 8);
        _engine->SetClickGroundFault(cond_ground);

        // P00<->P02: Drone pitch sync
        float cond_drone = bender.Conductance(0, 2);
        _engine->SetClickDroneSync(cond_drone);

        // Polyrhythmic pulses when bridged to pads 0, 1, 4, 6
        float cond_poly = std::max({bender.Conductance(0, 2), bender.Conductance(1, 2),
                                    bender.Conductance(2, 4), bender.Conductance(2, 6)});
        _engine->SetClickPoly(cond_poly);

        // Direct Click to Delay: tamed from 1.5f to 0.30f to stop runaway audio-rate blowout!
        float cond_click_delay = bender.Conductance(2, 6) * 0.30f;
        _engine->SetBridgeGain(BridgeId::ClickToDelayL, cond_click_delay);
        _engine->SetBridgeGain(BridgeId::ClickToDelayR, cond_click_delay);
    }

    void ProcessHumAndRadio(synthux::Touch& touch, TouchBender& bender, float clock_val, float starve_amount, float gen_drive) {
        // P03 Audio / Hum Antenna:
        // Captures authentic 50 Hz mains electromagnetic buzz and contact crackle.
        // Hovering / light contact picks up air buzz; firm touch increases hum volume.
        // Bridging P03 to Ground (P08) shunts and silences the buzz.
        // Muted when S30 Drive is low.
        float p03_touch = bender.Pressure(3);
        float p03_prox  = touch.air().PadProximity(3);
        float p03_shunt = bender.Conductance(3, 8);

        float hum_gain  = gen_drive * gen_drive * 0.55f;

        // Grounding shunt: bridging P03 to P08 silences the antenna buzz
        float grounding = daisysp::fclamp(p03_shunt * 1.6f, 0.0f, 1.0f);
        float p03_air   = daisysp::fclamp((p03_prox - 0.08f) / 0.80f, 0.0f, 1.0f);
        float p03_buzz  = (p03_touch * 0.22f + p03_air * 0.07f) * hum_gain * (1.0f - grounding);

        // Background chassis load hum (very quiet, only deepens when circuit is actually starved)
        float touch_total = bender.CurrentLoad();
        float base_hum  = (touch_total > 0.02f && starve_amount > 0.08f) ? ((0.0006f + 0.0016f * starve_amount) * (gen_drive * gen_drive)) : 0.0f;
        _engine->SetHum(p03_buzz, base_hum);

        // P08 Delay Diffusion (Chassis Ground / Reverb Wash):
        // Touching P08 diffuses delay repeats into a lush, smooth reverberant space.
        // Air proximity adds a misty ambient halo.
        // Bridging P03 to P08 also bleeds into diffusion.
        // Bridging P08 to P09 diffuses stutter sequencer slices!
        float p08_touch = bender.Pressure(8);
        float p08_prox  = touch.air().PadProximity(8);

        float p08_norm  = daisysp::fclamp((p08_touch - 0.04f) / 0.88f, 0.0f, 1.0f);
        float p08_diff  = p08_norm * p08_norm; // Smooth quadratic response
        float p08_prox_diff = daisysp::fclamp((p08_prox - 0.06f) / 0.80f, 0.0f, 1.0f) * 0.25f;
        float shunt_diff = p03_shunt * 0.85f;
        float p09_diff   = bender.Conductance(8, 9) * 0.90f;
        float diffusion_amt = daisysp::fclamp(p08_diff * 0.92f + p08_prox_diff + shunt_diff + p09_diff, 0.0f, 0.95f);
        _engine->SetDelayDiffusion(diffusion_amt);

        // P07 radio whine:
        // Only responds to air theremin if holding P08 (Ground)!
        // Otherwise, soft touch whines gently; pressing harder triggers runaway feedback.
        // Muted when S30 is low.
        float p07_touch = bender.Pressure(7);
        bool holding_p08 = (bender.Pressure(8) > 0.03f) || (bender.Conductance(7, 8) > 0.05f);
        float p07_air = 0.0f;
        if (holding_p08) {
            float raw_prox = touch.air().PadProximity(7);
            p07_air = daisysp::fclamp((raw_prox - 0.08f) / 0.80f, 0.0f, 1.0f);
        }

        float p07_soft = daisysp::fclamp((p07_touch - 0.015f) / 0.985f, 0.0f, 1.0f);
        float p07_presence = p07_soft + p07_air;
        if (p07_presence > 0.01f && gen_drive > 0.001f) {
            float base_rf = 293.66f * FastPow2(clock_val * 4.0f);
            float rf_freq = base_rf / (1.0f + p07_soft * 3.5f + p07_air * 1.5f);
            float rf_amp  = daisysp::fclamp(p07_soft * 0.035f + p07_air * 0.020f, 0.0f, 0.05f) * gen_drive;
            _engine->SetWhine(rf_amp, rf_freq);
        } else {
            _engine->SetWhine(0.0f, 1200.0f);
        }
    }

    void ProcessDelayAndBends(synthux::Touch& touch, TouchBender& bender, float drone_pitch_val) {
        // Right Fader controls stutter rate / slice length & glitch density:
        _engine->SetStutterRate(drone_pitch_val);
        _engine->SetGlitchDensity(drone_pitch_val);

        // P09 granular stutter and memory glitch
        float p09_touch = bender.Pressure(9);
        float p09_prox  = touch.air().PadProximity(9);

        // Fast strike triggers instant buffer skip / tape jump
        if (_p09_strike_cooldown > 0) {
            _p09_strike_cooldown--;
        }
        float p09_dp = p09_touch - _prev_p09;
        if (_p09_strike_cooldown == 0 && p09_dp > 0.40f) {
            _engine->TriggerBufferSkip(p09_dp * 2.0f);
            _p09_strike_cooldown = 120;
        }
        _prev_p09 = p09_touch;

        // Bridging on P09:
        float cond_5_9 = bender.Conductance(5, 9); // P05 (Reverse) <-> P09 (Stutter/Glitch)
        float cond_6_9 = bender.Conductance(6, 9); // P06 (Clock/Brake) <-> P09
        float cond_7_9 = bender.Conductance(7, 9); // P07 (Radio/Whine) <-> P09
        float cond_8_9 = bender.Conductance(8, 9); // P08 (Ground/Diffusion) <-> P09
        float cond_1_9 = bender.Conductance(1, 9); // P01 (Filter) <-> P09
        float p09_bridge = std::max({cond_5_9, cond_6_9, cond_7_9, cond_8_9, cond_1_9});

        // P09 air proximity: responsive air sensitivity
        float p09_air = daisysp::fclamp(p09_prox * 1.5f, 0.0f, 1.0f);
        float p09_effective = daisysp::fclamp(p09_touch * 0.90f + p09_air * 0.55f, 0.0f, 1.0f);
        float bridge_effective = (p09_bridge > 0.04f) ? p09_bridge : 0.0f;
        float delay_mangle = daisysp::fclamp(p09_effective + bridge_effective * 1.30f, 0.0f, 1.0f);
        _engine->SetDelayMangle(delay_mangle);

        // P09 pressure + air + adjacent bridging directly control glitch engine (bit decimation, sample decimation, dropouts)
        float glitch_amt = daisysp::fclamp(p09_touch * 0.85f + p09_air * 0.35f + p09_bridge * 0.95f, 0.0f, 1.0f);
        _engine->SetDelayGlitch(glitch_amt);

        // Switch A Up: dedicated buffer freeze
        bool a_up = touch.switches().IsAUp();
        _engine->SetDelayFreeze(a_up);

        // P07 feedback bridges to filter and starve: only engages when pressing down HARD (> 0.45 pressure) or bridging.
        // Scaled by S34 feedback knob so feedback level is controllable and prevents runaway screeching.
        float p07_touch = bender.Pressure(7);
        float p07_hard = daisysp::fclamp((p07_touch - 0.45f) / 0.55f, 0.0f, 1.0f);
        float fb_gain = daisysp::fclamp(_current_fb * 1.05f, 0.0f, 1.0f);
        float cond_delay_feed = std::max(bender.Conductance(7, 5) * 0.35f, p07_hard * 0.35f) * fb_gain;
        _engine->SetBridgeGain(BridgeId::DelayToFilterL, cond_delay_feed);
        _engine->SetBridgeGain(BridgeId::DelayToFilterR, cond_delay_feed);
        _engine->SetBridgeGain(BridgeId::DelayToCutoff, bender.Conductance(7, 1) * 1.1f + cond_1_9 * 0.8f);

        // P07 and P11 both contribute to DelayToStarve bridge (unified without clobbering)
        float cond_delay_starve = bender.Conductance(7, 4) * 0.60f;
        float p11_touch = bender.Pressure(11);
        float total_delay_starve = std::max(cond_delay_starve, p11_touch * 0.9f);
        _engine->SetBridgeGain(BridgeId::DelayToStarveL, total_delay_starve);
        _engine->SetBridgeGain(BridgeId::DelayToStarveR, total_delay_starve);

        // S&H chorus modulation: only active when starved (Vcc < 0.35) or when bridging P01/P04/P02 to P06
        float vcc = _engine->GetVcc();
        float lower_half_starve = daisysp::fclamp((0.35f - vcc) * 3.0f, 0.0f, 1.0f);
        float cond_1_6 = bender.Conductance(1, 6);
        float cond_4_6 = bender.Conductance(4, 6);
        float cond_2_6 = bender.Conductance(2, 6);
        float subtle_mod = daisysp::fclamp(
            cond_1_6 * 1.3f +
            cond_4_6 * 1.3f +
            cond_2_6 * 0.8f +
            lower_half_starve * 0.45f,
            0.0f, 1.0f
        );
        _engine->SetDelaySubtleMod(subtle_mod);

        // P06 Tape Reel Brake & P00 to P06 FM bridge (+ P09 to P06 warp)
        float p06_press = bender.Pressure(6);
        float p06_cond = bender.Conductance(6, 8);
        float clock_bend = daisysp::fclamp(p06_cond * 1.8f + p06_press * 0.85f + cond_6_9 * 1.4f, 0.0f, 1.0f);
        _engine->SetClockBend(clock_bend);

        float cond_0_6 = bender.Conductance(0, 6);
        float fm_gain = daisysp::fclamp(cond_0_6 * 1.5f, 0.0f, 1.0f) * 0.65f;
        _engine->SetBridgeGain(BridgeId::DroneToDelayClock, fm_gain);

        // P04 Starve sag + high resonance dynamic pull on power rail
        float res_sag = (_current_res * _current_res) * _current_starve * 0.15f;
        float starve_sag = bender.Conductance(4, 8) * 1.8f + bender.Pressure(4) * 0.85f + bender.CurrentLoad() + res_sag;
        _engine->SetStarveAux(0, starve_sag);

        // Pad 05: Dedicated Delay Reverse (touch pressure + conductance bridging)
        float p05_press = bender.Pressure(5);
        float p05_bridge = std::max({
            bender.Conductance(5, 6),  // P06 (Clock) <-> P05 (Reverse)
            cond_5_9,                 // P09 (Stutter/Glitch) <-> P05 (Reverse)
            bender.Conductance(5, 4),  // P04 (Starve) <-> P05 (Reverse)
            bender.Conductance(5, 8),  // P08 (Ground) <-> P05 (Reverse)
            bender.Conductance(5, 7),  // P07 (Radio) <-> P05 (Reverse)
            bender.Conductance(5, 1)   // P01 (Filter) <-> P05 (Reverse)
        });
        float reverse_amt = daisysp::fclamp(p05_press * 1.2f + p05_bridge * 1.5f, 0.0f, 1.0f);
        _engine->SetDelayReverse(reverse_amt);

        // Reverse Mod: when P05 and P09 are bridged, or both active, slices alternate forward/reverse randomly!
        float reverse_mod = daisysp::fclamp(cond_5_9 * 1.8f + (p05_press > 0.08f && delay_mangle > 0.08f ? 0.70f : 0.0f), 0.0f, 1.0f);
        _engine->SetDelayReverseMod(reverse_mod);
    }

    void UpdateLedBrightness(synthux::Touch& touch, TouchBender& bender) {
        float max_touch = 0.0f;
        for (int i = 0; i < 12; ++i) {
            max_touch = std::max(max_touch, bender.Pressure(i));
        }
        float max_prox = 0.0f;
        for (int i = 0; i < 12; ++i) {
            max_prox = std::max(max_prox, touch.air().PadProximity(i));
        }
        float audio_peak = _engine->GetAudioPeak();
        float vcc = _engine->GetVcc();
        bool floating = _engine->IsFloatingActive();

        // Organic idle breathing wave (~0.22 Hz)
        _led_breath_phase += 0.0018f;
        if (_led_breath_phase >= 6.2831853f) _led_breath_phase -= 6.2831853f;
        float breath = 0.05f + 0.04f * (sinf(_led_breath_phase) * 0.5f + 0.5f);

        float target_led = 0.0f;
        if (floating) {
            // Warmup pulse
            float progress = _engine->GetWarmupProgress();
            float pulse_speed = 0.010f + progress * 0.020f;
            _led_floating_phase += pulse_speed;
            if (_led_floating_phase >= 6.2831853f) _led_floating_phase -= 6.2831853f;
            target_led = 0.08f + 0.50f * (sinf(_led_floating_phase) * 0.5f + 0.5f) + audio_peak * 0.30f;
        } else if (max_touch > 0.02f || max_prox > 0.08f) {
            // Touch force response + audio VU pulse
            target_led = 0.12f + max_touch * 0.65f + max_prox * 0.20f + audio_peak * 0.30f;
        } else {
            // Idle breathing + audio glow
            target_led = breath + audio_peak * 0.40f;
        }

        // Starve sputtering and flickering
        if (vcc < 0.35f) {
            float starve_depth = (0.35f - vcc) / 0.35f;
            _led_rng = _led_rng * 1664525u + 1013904223u;
            float flicker = static_cast<float>(_led_rng & 0xFF) / 255.0f;
            if (flicker < starve_depth * 0.40f) {
                target_led *= 0.15f; // Random dropout
            } else if (flicker > 0.85f) {
                target_led = std::min(1.0f, target_led * 1.5f); // Sputter flare
            }
        }

        daisysp::fonepole(_smooth_led_brightness, daisysp::fclamp(target_led, 0.0f, 1.0f), 0.08f);
    }

    void UpdateLed(daisy::DaisySeed& /*hw*/, synthux::Touch& /*touch*/, uint32_t /*loop_counter*/) {
        // Backwards compatibility stub; LED PWM is now run in AudioCallback via UpdateLedPwm()
    }

    BendsEngine* _engine{nullptr};
    float _prev_p01{0.0f};
    float _prev_p02{0.0f};
    float _prev_p09{0.0f};
    uint32_t _p02_strike_cooldown{0};
    uint32_t _p09_strike_cooldown{0};
    bool _was_a_up{false};
    bool _was_a_down{false};
    uint32_t _down_to_freeze_counter{0};
    bool _first_switch_read{true};
    MusicalTuning::ScaleSnapper _drone_scale_snap;
    float _current_res{0.05f};
    float _current_cutoff{1000.0f};
    float _current_fb{0.35f};
    float _current_starve{0.0f};
    float _smooth_s33{0.3f};
    float _last_stable_s33{0.3f};
    float _smooth_clock_fader{0.5f};
    float _smooth_led_brightness{0.0f};
    float _led_pwm_accum{0.0f};
    float _led_breath_phase{0.0f};
    float _led_floating_phase{0.0f};
    uint32_t _led_rng{0xACE1};
};

} // namespace bends
