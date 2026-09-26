#pragma once

#include "circuit.h"
#include "drone_module.h"
#include "click_module.h"
#include "hum_module.h"
#include "radio_module.h"
#include "filter_module.h"
#include "starve_module.h"
#include "delay_module.h"
#include "fast_math.h"
#include "daisysp.h"
#include <cmath>

namespace bends {

enum class ModuleId : uint8_t {
    Drone  = 0,
    Click  = 1,
    Hum    = 2,
    Radio  = 3,
    Filter = 4,
    Starve = 5,
    Delay  = 6
};

enum class BridgeId : uint8_t {
    DelayToFilterL    = 12,
    DelayToFilterR    = 13,
    FilterBpToCutoff  = 14,
    DelayToCutoff     = 15,
    DroneToDelayClock = 16,
    DelayToStarveL    = 17,
    DelayToStarveR    = 18,
    ClickToDelayL     = 19,
    ClickToDelayR     = 20
};

class BendsEngine {
public:
    void Init(float sample_rate, uint16_t block_size) {
        _sample_rate = sample_rate;
        _block_size = block_size;

        _circuit.Init(sample_rate, block_size);
        _circuit.RegisterModule(ModuleId::Drone,  &_mod_drone);
        _circuit.RegisterModule(ModuleId::Click,  &_mod_click);
        _circuit.RegisterModule(ModuleId::Hum,    &_mod_hum);
        _circuit.RegisterModule(ModuleId::Radio,  &_mod_radio);
        _circuit.RegisterModule(ModuleId::Filter, &_mod_filter);
        _circuit.RegisterModule(ModuleId::Starve, &_mod_starve);
        _circuit.RegisterModule(ModuleId::Delay,  &_mod_delay);

        // Primary signal chain: Sources -> Filter -> Starve -> Delay
        _circuit.Connect({ModuleId::Drone,  Port::MainL, ModuleId::Filter, Port::MainL, 1.0f});
        _circuit.Connect({ModuleId::Drone,  Port::MainR, ModuleId::Filter, Port::MainR, 1.0f});
        _circuit.Connect({ModuleId::Click,  Port::MainL, ModuleId::Filter, Port::MainL, 1.0f});
        _circuit.Connect({ModuleId::Click,  Port::MainR, ModuleId::Filter, Port::MainR, 1.0f});
        _circuit.Connect({ModuleId::Hum,    Port::MainL, ModuleId::Filter, Port::MainL, 1.0f});
        _circuit.Connect({ModuleId::Hum,    Port::MainR, ModuleId::Filter, Port::MainR, 1.0f});
        _circuit.Connect({ModuleId::Radio,  Port::MainL, ModuleId::Filter, Port::MainL, 1.0f});
        _circuit.Connect({ModuleId::Radio,  Port::MainR, ModuleId::Filter, Port::MainR, 1.0f});
        _circuit.Connect({ModuleId::Filter, Port::MainL, ModuleId::Starve, Port::MainL, 1.0f});
        _circuit.Connect({ModuleId::Filter, Port::MainR, ModuleId::Starve, Port::MainR, 1.0f});
        _circuit.Connect({ModuleId::Starve, Port::MainL, ModuleId::Delay,  Port::MainL, 1.0f});
        _circuit.Connect({ModuleId::Starve, Port::MainR, ModuleId::Delay,  Port::MainR, 1.0f});

        // Virtual circuit bend bridges (default gain = 0.0f, bridged via touch)
        _circuit.Connect({ModuleId::Delay,  Port::Aux0,  ModuleId::Filter, Port::MainL, 0.0f}); // 12: Delay to Filter L
        _circuit.Connect({ModuleId::Delay,  Port::Aux0,  ModuleId::Filter, Port::MainR, 0.0f}); // 13: Delay to Filter R
        _circuit.Connect({ModuleId::Filter, Port::Aux0,  ModuleId::Filter, Port::Aux0,  0.0f}); // 14: Filter BP to Cutoff
        _circuit.Connect({ModuleId::Delay,  Port::Aux0,  ModuleId::Filter, Port::Aux0,  0.0f}); // 15: Delay to Cutoff
        _circuit.Connect({ModuleId::Drone,  Port::MainL, ModuleId::Delay,  Port::Aux0,  0.0f}); // 16: Drone to Delay Clock
        _circuit.Connect({ModuleId::Delay,  Port::Aux0,  ModuleId::Starve, Port::MainL, 0.0f}); // 17: Delay to Starve L
        _circuit.Connect({ModuleId::Delay,  Port::Aux0,  ModuleId::Starve, Port::MainR, 0.0f}); // 18: Delay to Starve R
        _circuit.Connect({ModuleId::Click,  Port::MainL, ModuleId::Delay,  Port::MainL, 0.0f}); // 19: Direct Click to Delay L
        _circuit.Connect({ModuleId::Click,  Port::MainR, ModuleId::Delay,  Port::MainR, 0.0f}); // 20: Direct Click to Delay R

        _circuit.Compile();

        _limiter_l.Init();
        _limiter_r.Init();
    }

    void ProcessBlock(const float* in_l, const float* in_r,
                      float* out_l, float* out_r, size_t size) {
        // Block-rate parameter smoothing (1000 Hz at 48 samples / 48 kHz)
        daisysp::fonepole(_smooth_drive,        _target_drive,        0.02f);
        daisysp::fonepole(_smooth_pitch,        _target_pitch,        0.02f);
        daisysp::fonepole(_smooth_vca,          _target_vca,          0.05f);
        daisysp::fonepole(_smooth_sub,          _target_sub,          0.03f);
        daisysp::fonepole(_smooth_cutoff,       _target_cutoff,       0.02f);
        daisysp::fonepole(_smooth_res,          _target_res,          0.02f);
        daisysp::fonepole(_smooth_delay_time,   _target_delay_time,   0.08f);
        daisysp::fonepole(_smooth_feedback,     _target_feedback,     0.02f);
        daisysp::fonepole(_smooth_vcc,          _target_vcc,          0.02f);
        daisysp::fonepole(_smooth_mix,          _target_mix,          0.02f);
        daisysp::fonepole(_smooth_delay_mangle,     _target_delay_mangle,     0.05f);
        daisysp::fonepole(_smooth_delay_glitch,     _target_delay_glitch,     0.05f);
        daisysp::fonepole(_smooth_delay_subtle_mod, _target_delay_subtle_mod, 0.05f);
        daisysp::fonepole(_smooth_click_poly,       _target_click_poly,       0.05f);
        daisysp::fonepole(_smooth_delay_clock,      _target_delay_clock,      0.05f);
        daisysp::fonepole(_smooth_delay_reverse,    _target_delay_reverse,    0.05f);
        daisysp::fonepole(_smooth_delay_diffusion,  _target_delay_diffusion,  0.015f);

        // Rail sag from memory glitch
        float glitch_sag = _mod_delay.GetGlitchSag();
        float lower_half_starve = daisysp::fclamp((0.50f - _smooth_vcc) * 2.0f, 0.0f, 1.0f);
        float effective_sag = glitch_sag * lower_half_starve;

        float sagged_cutoff = _smooth_cutoff * (1.0f - effective_sag * 0.45f);
        float sagged_pitch  = _smooth_pitch * (1.0f - effective_sag * 0.28f);

        float eff_cutoff = sagged_cutoff;
        float eff_pitch  = sagged_pitch;
        float eff_vca    = _smooth_vca;
        float eff_vcc    = _smooth_vcc;
        float eff_res    = _smooth_res;
        float eff_feedback = _smooth_feedback;
        float eff_radio_amp = _target_radio_amp;
        float eff_radio_freq = _target_radio_freq;
        float eff_hum_buzz = _target_hum_buzz;
        float eff_hum_gnd  = _target_hum_gnd;

        if (_floating_active) {
            if (_warmup_delay > 0) {
                _warmup_delay--;
                // During initial quiet settle delay (~185ms), keep floating additions silent.
                // A quick flick down and back stays within this window and clears patch without sound.
            } else {
                // Warm-up progress: smoothly ramps from 0 to 1 over ~2.5s (asymptotic thermal settling)
                _warmup_progress += 0.0010f * (1.0f - _warmup_progress);

                // Incommensurate drift phases advance (simulate chaotic thermal wandering)
                _floating_phase1 += 0.00018f;
                if (_floating_phase1 >= 1.0f) _floating_phase1 -= 1.0f;
                _floating_phase2 += 0.00035f;
                if (_floating_phase2 >= 1.0f) _floating_phase2 -= 1.0f;
                _floating_phase3 += 0.00057f;
                if (_floating_phase3 >= 1.0f) _floating_phase3 -= 1.0f;

                float drift_sin1 = std::sin(_floating_phase1 * 6.2831853f);
                float drift_sin2 = std::sin(_floating_phase2 * 6.2831853f);
                float drift_sin3 = std::sin(_floating_phase3 * 6.2831853f);

                // Subtle microtonal pitch drift: wandering by +/- 18 cents (0.015 octaves) max!
                // NO siren sweep, NO racecar revving!
                float pitch_drift_oct = (drift_sin1 * 0.010f + drift_sin2 * 0.005f) * _warmup_progress;

                // Cutoff drift: breathes slowly like thermal expansion
                float cutoff_drift_oct = _floating_cutoff_base + (drift_sin2 * 0.35f + drift_sin3 * 0.20f) * _warmup_progress;
                eff_cutoff = daisysp::fclamp(sagged_cutoff * FastPow2(cutoff_drift_oct), 30.0f, 11000.0f);

                // Floating rail sag
                eff_vcc = daisysp::fclamp(_smooth_vcc - _floating_starve_bias * _warmup_progress, 0.02f, 1.0f);

                // Extra singing resonance
                eff_res = daisysp::fclamp(_smooth_res + _floating_res_boost * _warmup_progress, 0.0f, 0.90f);

                // Leaky capacitor static discharge accumulator
                _leak_accum += _leak_rate;
                if (_leak_accum >= 1.0f) {
                    _leak_accum -= 1.0f;
                    _floating_rng = _floating_rng * 1664525u + 1013904223u;

                    switch (_uninit_state) {
                        case UninitializedState::StaticClicks: {
                            float click_vel = 0.06f + ((_floating_rng & 0xFF) / 255.0f) * 0.16f;
                            _mod_click.Strike(click_vel);
                            if ((_floating_rng & 0x07) == 0) {
                                _mod_filter.Ping(0.08f);
                            }
                            _leak_rate = 0.005f + (((_floating_rng >> 8) & 0xFF) / 255.0f) * 0.035f;
                            break;
                        }
                        case UninitializedState::MemoryGhost: {
                            float click_vel = 0.04f + ((_floating_rng & 0x7F) / 127.0f) * 0.06f;
                            _mod_click.Strike(click_vel);
                            _leak_rate = 0.002f + (((_floating_rng >> 8) & 0xFF) / 255.0f) * 0.004f;
                            break;
                        }
                        default: {
                            float click_vel = 0.03f + ((_floating_rng & 0x3F) / 63.0f) * 0.04f;
                            _mod_click.Strike(click_vel);
                            _leak_rate = 0.0006f + (((_floating_rng >> 8) & 0xFF) / 255.0f) * 0.0015f;
                            break;
                        }
                    }
                }

                // Personality-specific uninitialized circuit states
                switch (_uninit_state) {
                    case UninitializedState::StaticClicks:
                        // Drone is COMPLETELY SILENT!
                        eff_vca = _smooth_vca;
                        eff_hum_buzz = std::max(_target_hum_buzz, 0.020f * _warmup_progress);
                        eff_hum_gnd  = std::max(_target_hum_gnd, 0.015f * _warmup_progress);
                        eff_feedback = daisysp::fclamp(_smooth_feedback * 0.75f + 0.20f * _warmup_progress, 0.0f, 0.70f);
                        break;

                    case UninitializedState::SputteringRail:
                        // Sub-harmonic motorboat throb (18–32 Hz)
                        eff_pitch = 18.0f + 14.0f * _warmup_progress + drift_sin1 * 3.0f;
                        eff_vca = std::max(_smooth_vca, 0.14f * _warmup_progress);
                        eff_vcc = daisysp::fclamp(_smooth_vcc - 0.45f * _warmup_progress, 0.03f, 0.40f);
                        break;

                    case UninitializedState::RadioWhistle:
                        // Drone is COMPLETELY SILENT!
                        eff_vca = _smooth_vca;
                        eff_radio_amp = std::max(_target_radio_amp, 0.028f * _warmup_progress);
                        eff_radio_freq = 1100.0f + 700.0f * (drift_sin1 * 0.5f + 0.5f);
                        eff_res = daisysp::fclamp(_smooth_res + 0.18f * _warmup_progress, 0.0f, 0.88f);
                        break;

                    case UninitializedState::MemoryGhost:
                        // Drone is COMPLETELY SILENT!
                        eff_vca = _smooth_vca;
                        eff_feedback = daisysp::fclamp(_smooth_feedback + 0.35f * _warmup_progress, 0.0f, 0.86f);
                        break;

                    case UninitializedState::SingingHarmonic:
                        // Gentle tuned harmonic drone, NO pitch sweep!
                        eff_pitch = sagged_pitch * _floating_pitch_ratio * _floating_detune * FastPow2(pitch_drift_oct);
                        eff_vca = std::max(_smooth_vca, _floating_ghost_vca * _warmup_progress);
                        break;
                }
            }
        } else {
            _warmup_progress = 0.0f;
            _warmup_delay = 0;
        }

        _mod_drone.SetParam(0, eff_pitch);
        _mod_drone.SetParam(1, eff_vca);
        _mod_drone.SetParam(2, _smooth_sub);
        _mod_drone.SetParam(3, _smooth_drive);
        _mod_drone.SetParam(4, eff_vcc);

        _mod_click.SetParam(0, _target_click_pressure);
        _mod_click.SetParam(1, _target_click_pitch);
        _mod_click.SetParam(2, 1.0f - eff_vcc);
        _mod_click.SetParam(3, _smooth_drive);
        _mod_click.SetPolyMod(_smooth_click_poly);
        _mod_click.SetDelayTime(_mod_delay.GetCurrentDelayTime());
        _mod_click.SetDroneFreq(eff_pitch);

        _mod_hum.SetParam(0, eff_hum_buzz);
        _mod_hum.SetParam(1, eff_hum_gnd);
        _mod_hum.SetParam(2, _smooth_drive);
        _mod_hum.SetParam(3, 1.0f - eff_vcc);

        _mod_radio.SetParam(0, eff_radio_freq);
        _mod_radio.SetParam(1, eff_radio_amp);

        _mod_filter.SetParam(0, eff_cutoff);
        _mod_filter.SetParam(1, eff_res);
        _mod_filter.SetParam(2, _smooth_drive);

        _mod_starve.SetParam(0, daisysp::fclamp(eff_vcc - effective_sag * 0.25f, 0.0f, 1.0f));
        _mod_starve.SetParam(1, _smooth_drive);

        _mod_delay.SetParam(0, _smooth_delay_time);
        _mod_delay.SetParam(1, eff_feedback);
        _mod_delay.SetParam(2, _smooth_cutoff);
        _mod_delay.SetParam(3, _smooth_mix);
        _mod_delay.SetParam(4, 1.0f - _smooth_vcc);
        _mod_delay.SetMangle(_smooth_delay_mangle);
        _mod_delay.SetGlitch(_smooth_delay_glitch);
        _mod_delay.SetFreeze(_target_delay_freeze);
        _mod_delay.SetSubtleMod(_smooth_delay_subtle_mod);
        _mod_delay.SetMasterClock(_smooth_delay_clock);
        _mod_delay.SetReverse(_smooth_delay_reverse);
        _mod_delay.SetDiffusion(_smooth_delay_diffusion);

        _mod_delay.PrepareBlock(size);

        _circuit.ProcessBlock(in_l, in_r, out_l, out_r, size);

        // Signal energy creates gentle rail breathing
        float signal_energy = 0.0f;
        for (size_t i = 0; i < size; ++i) {
            signal_energy += std::abs(out_l[i]) + std::abs(out_r[i]);
        }
        signal_energy /= (2.0f * static_cast<float>(size));
        float energy_sag = signal_energy * 0.06f * (1.0f - _smooth_vcc);
        _target_vcc = std::max(0.02f, _target_vcc - energy_sag);

        _limiter_l.ProcessBlock(out_l, size, 1.0f);
        _limiter_r.ProcessBlock(out_r, size, 1.0f);

        for (size_t i = 0; i < size; ++i) {
            out_l[i] = daisysp::fclamp(out_l[i], -0.95f, 0.95f);
            out_r[i] = daisysp::fclamp(out_r[i], -0.95f, 0.95f);
        }
    }

    void SetDrive(float val)          { _target_drive = val; }
    void SetPitch(float val)          { _target_pitch = val; }
    void SetVca(float val)            { _target_vca = val; }
    void SetSub(float val)            { _target_sub = val; }
    void SetCutoff(float val)         { _target_cutoff = val; }
    void SetRes(float val)            { _target_res = val; }
    void SetDelayTime(float val)      { _target_delay_time = val; }
    void SetDelayClock(float val)     { _target_delay_clock = val; }
    void SetFeedback(float val)       { _target_feedback = val; }
    void SetVcc(float val)            { _target_vcc = val; }
    void SetMix(float val)            { _target_mix = val; }
    void SetDelayMangle(float val)    { _target_delay_mangle = val; }
    void SetDelayGlitch(float val)    { _target_delay_glitch = val; }
    void SetDelayFreeze(bool f)       { _target_delay_freeze = f; }
    void SetDelayReverse(float val)   { _target_delay_reverse = val; }
    void SetDelayDiffusion(float val) { _target_delay_diffusion = val; }
    float GetDelayDiffusion() const   { return _smooth_delay_diffusion; }
    void SetDelaySubtleMod(float val) { _target_delay_subtle_mod = val; }
    void SetGlitchDensity(float val)  { _mod_delay.SetGlitchDensity(val); }
    void SetClickPoly(float val)      { _target_click_poly = val; }

    float GetVcc() const           { return _smooth_vcc; }
    float GetDrive() const         { return _smooth_drive; }

    void StrikeClick(float vel)          { _mod_click.Strike(vel); }
    void SetClickPressure(float val)     { _target_click_pressure = val; }
    void SetClickPitch(float val)        { _target_click_pitch = val; }
    float GetGlitchSag() const     { return _mod_delay.GetGlitchSag(); }
    void TriggerBufferSkip(float strength = 1.0f) { _mod_delay.TriggerBufferSkip(strength); }

    float GetDelayTime() const     { return _mod_delay.GetCurrentDelayTime(); }
    float GetPitch() const         { return _smooth_pitch; }

    void EnterFloatingState() {
        _floating_active = true;
        _warmup_delay = 140; // ~185ms quiet contact settle window (clean flick won't emit sound)
        _warmup_progress = 0.0f; // Start cold, warm up gently over ~2.5s

        // 1. Immediately wipe delay buffers & reset feedback so uninitialized state starts clean
        _mod_delay.ClearBuffers();
        _circuit.Reset();
        _target_delay_freeze = false;
        _target_delay_reverse = 0.0f;
        _smooth_delay_reverse = 0.0f;

        // NO loud filter ping!
        // NO buffer skip glitch!

        // 2. Roll one of 5 distinct stochastic uninitialized circuit personalities
        _floating_rng = _floating_rng * 1664525u + 1013904223u;
        uint32_t roll = (_floating_rng >> 4) % 5;
        _uninit_state = static_cast<UninitializedState>(roll);

        // Seed random leak rates & phases
        _leak_accum = 0.0f;
        _floating_rng = _floating_rng * 1664525u + 1013904223u;
        _leak_rate = 0.005f + ((_floating_rng & 0xFF) / 255.0f) * 0.015f;

        // Incommensurate drift phases
        _floating_phase1 = static_cast<float>((_floating_rng >> 8) & 0xFF) / 255.0f;
        _floating_phase2 = static_cast<float>((_floating_rng >> 16) & 0xFF) / 255.0f;
        _floating_phase3 = static_cast<float>((_floating_rng >> 24) & 0xFF) / 255.0f;

        // Pick musical harmonic ratios (used only if SingingHarmonic is rolled)
        static constexpr float kHarmonics[] = {
            0.50f, 0.75f, 1.0f, 1.25f, 1.333f, 1.50f, 2.0f
        };
        size_t h_idx = (_floating_rng >> 2) % (sizeof(kHarmonics) / sizeof(kHarmonics[0]));
        _floating_pitch_ratio = kHarmonics[h_idx];

        // Subtle analog microtonal detune (+/- 15 cents max)
        _floating_rng = _floating_rng * 1664525u + 1013904223u;
        float detune_semi = (static_cast<float>(static_cast<int32_t>(_floating_rng)) * 2.3283e-10f) * 0.15f;
        _floating_detune = FastPow2(detune_semi / 12.0f);

        // Ghost VCA level (gentle, 0.10 - 0.18 max, only for SingingHarmonic)
        _floating_rng = _floating_rng * 1664525u + 1013904223u;
        _floating_ghost_vca = 0.10f + (static_cast<float>(_floating_rng & 0xFF) / 255.0f) * 0.08f;

        // Grittiness / starve bias
        _floating_rng = _floating_rng * 1664525u + 1013904223u;
        _floating_starve_bias = 0.15f + (static_cast<float>(_floating_rng & 0xFF) / 255.0f) * 0.25f;

        // Cutoff offset & resonance boost
        _floating_rng = _floating_rng * 1664525u + 1013904223u;
        _floating_cutoff_base = (static_cast<float>(static_cast<int32_t>(_floating_rng)) * 2.3283e-10f) * 0.35f;
        _floating_res_boost = 0.05f + (static_cast<float>(_floating_rng & 0x7F) / 127.0f) * 0.15f;
    }

    void CleanReset() {
        _floating_active = false;
        _warmup_progress = 0.0f;
        _warmup_delay = 0;

        // 1. Immediately wipe all delay audio buffers in SDRAM and reset tap heads
        _mod_delay.ClearBuffers();

        // 2. Reset modules and feedback buffers in Circuit
        _circuit.Reset();

        // 3. Clear transient states
        _target_delay_freeze = false;
        _target_delay_reverse = 0.0f;
        _smooth_delay_reverse = 0.0f;
        _target_delay_diffusion = 0.0f;
        _smooth_delay_diffusion = 0.0f;
        _target_radio_amp = 0.0f;

        // Pure silent, clean reset. NO filter ping!
    }

    bool IsFloatingActive() const   { return _floating_active; }
    float GetWarmupProgress() const { return _warmup_progress; }

    // Backwards-compatible aliases
    void TriggerCircuitCrash() { EnterFloatingState(); }
    void SetCrashLatch(bool on) { if (on) EnterFloatingState(); else CleanReset(); }

    void SetClickDelaySync(float val)   { _mod_click.SetDelaySync(val); }
    void SetClickFilterBlend(float val) { _mod_click.SetFilterBlend(val); }
    void SetClickGroundFault(float val) { _mod_click.SetGroundFault(val); }
    void SetClickDroneSync(float val)   { _mod_click.SetDroneSync(val); }
    void SetClickTimeLeak(float val)    { _mod_click.SetTimeLeak(val); }

    void SetHum(float buzz, float hum)   { _target_hum_buzz = buzz; _target_hum_gnd = hum; }
    void SetWhine(float amp, float freq) { _target_radio_amp = amp; _target_radio_freq = freq; }
    void PingFilter(float strength)      { _mod_filter.Ping(strength); }

    void SetDroneAux(uint8_t ch, float val)  { _mod_drone.SetAuxInput(ch, val); }
    void SetFilterAux(uint8_t ch, float val) { _mod_filter.SetAuxInput(ch, val); }
    void SetStarveAux(uint8_t ch, float val) { _mod_starve.SetAuxInput(ch, val); }
    void SetDelayAux(uint8_t ch, float val)  { _mod_delay.SetAuxInput(ch, val); }

    void SetBridgeGain(BridgeId id, float gain) {
        _circuit.SetConnectionGain(static_cast<uint8_t>(id), gain);
    }

    const Circuit& GetCircuit() const { return _circuit; }
    Circuit& GetCircuit()             { return _circuit; }

private:
    float _sample_rate{48000.0f};
    uint16_t _block_size{64};

    Circuit      _circuit;
    DroneModule  _mod_drone;
    ClickModule  _mod_click;
    HumModule    _mod_hum;
    RadioModule  _mod_radio;
    FilterModule _mod_filter;
    StarveModule _mod_starve;
    DelayModule  _mod_delay;

    daisysp::Limiter _limiter_l;
    daisysp::Limiter _limiter_r;

    float _target_drive{0.20f},      _smooth_drive{0.20f};
    float _target_pitch{110.0f},     _smooth_pitch{110.0f};
    float _target_vca{0.0f},         _smooth_vca{0.0f};
    float _target_sub{0.25f},        _smooth_sub{0.25f};
    float _target_cutoff{1000.0f},   _smooth_cutoff{1000.0f};
    float _target_res{0.25f},        _smooth_res{0.25f};
    float _target_delay_time{0.25f}, _smooth_delay_time{0.25f};
    float _target_feedback{0.35f},   _smooth_feedback{0.35f};
    float _target_vcc{1.0f},         _smooth_vcc{1.0f};
    float _target_mix{0.40f},        _smooth_mix{0.40f};
    float _target_delay_mangle{0.0f},    _smooth_delay_mangle{0.0f};
    float _target_delay_glitch{0.0f},    _smooth_delay_glitch{0.0f};
    float _target_delay_subtle_mod{0.0f},_smooth_delay_subtle_mod{0.0f};
    float _target_delay_clock{0.5f},     _smooth_delay_clock{0.5f};
    float _target_delay_reverse{0.0f},   _smooth_delay_reverse{0.0f};
    float _target_delay_diffusion{0.0f}, _smooth_delay_diffusion{0.0f};
    bool  _target_delay_freeze{false};

    float _target_click_pressure{0.0f};
    float _target_click_pitch{0.5f};
    float _target_click_poly{0.0f},  _smooth_click_poly{0.0f};
    float _target_hum_buzz{0.0f};
    float _target_hum_gnd{0.0f};
    float _target_radio_amp{0.0f};
    float _target_radio_freq{1200.0f};

    enum class UninitializedState : uint8_t {
        StaticClicks = 0,    // Leaky gate / Geiger static discharges bouncing through delay
        SputteringRail = 1,  // Starved power rail / sub-harmonic motorboat throb (18-32 Hz)
        RadioWhistle = 2,    // High-Z parasitic RF antenna whistle & wind-like resonant filter drift
        MemoryGhost = 3,     // Uninitialized memory comb ringing & sporadic microscopic pings
        SingingHarmonic = 4  // Warm singing harmonic (gentle tuned resonance, NO pitch glide)
    };

    UninitializedState _uninit_state{UninitializedState::StaticClicks};
    uint16_t _warmup_delay{0};
    float    _leak_accum{0.0f};
    float    _leak_rate{0.005f};

    uint32_t _floating_rng{0xDEADBEEF};
    bool     _floating_active{false};
    float    _warmup_progress{0.0f};
    float    _floating_pitch_ratio{1.0f};
    float    _floating_detune{1.0f};
    float    _floating_ghost_vca{0.0f};
    float    _floating_starve_bias{0.0f};
    float    _floating_cutoff_base{0.0f};
    float    _floating_res_boost{0.0f};
    float    _floating_phase1{0.0f};
    float    _floating_phase2{0.0f};
    float    _floating_phase3{0.0f};
};

} // namespace bends
