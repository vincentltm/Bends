#pragma once

#include "circuit.h"
#include "drone_module.h"
#include "click_module.h"
#include "hum_module.h"
#include "radio_module.h"
#include "corrupt_module.h"
#include "filter_module.h"
#include "starve_module.h"
#include "delay_module.h"
#include "fast_math.h"
#include "daisysp.h"
#include <cmath>

namespace bends {

enum class ModuleId : uint8_t {
    Drone   = 0,
    Click   = 1,
    Hum     = 2,
    Radio   = 3,
    Filter  = 4,
    Starve  = 5,
    Delay   = 6,
    Corrupt = 7
};

enum class BridgeId : uint8_t {
    DelayToFilterL    = 14,
    DelayToFilterR    = 15,
    FilterBpToCutoff  = 16,
    DelayToCutoff     = 17,
    DroneToDelayClock = 18,
    DelayToStarveL    = 19,
    DelayToStarveR    = 20,
    ClickToDelayL     = 21,
    ClickToDelayR     = 22
};

class BendsEngine {
public:
    void Init(float sample_rate, uint16_t block_size) {
        _sample_rate = sample_rate;
        _block_size = block_size;

        _circuit.Init(sample_rate, block_size);
        _circuit.RegisterModule(ModuleId::Drone,   &_mod_drone);
        _circuit.RegisterModule(ModuleId::Click,   &_mod_click);
        _circuit.RegisterModule(ModuleId::Hum,     &_mod_hum);
        _circuit.RegisterModule(ModuleId::Radio,   &_mod_radio);
        _circuit.RegisterModule(ModuleId::Filter,  &_mod_filter);
        _circuit.RegisterModule(ModuleId::Starve,  &_mod_starve);
        _circuit.RegisterModule(ModuleId::Delay,   &_mod_delay);
        _circuit.RegisterModule(ModuleId::Corrupt, &_mod_corrupt);

        // Primary signal chain: Sources -> Filter -> Starve -> Delay
        _circuit.Connect({ModuleId::Drone,   Port::MainL, ModuleId::Filter, Port::MainL, 1.0f}); // 0
        _circuit.Connect({ModuleId::Drone,   Port::MainR, ModuleId::Filter, Port::MainR, 1.0f}); // 1
        _circuit.Connect({ModuleId::Click,   Port::MainL, ModuleId::Filter, Port::MainL, 1.0f}); // 2
        _circuit.Connect({ModuleId::Click,   Port::MainR, ModuleId::Filter, Port::MainR, 1.0f}); // 3
        _circuit.Connect({ModuleId::Hum,     Port::MainL, ModuleId::Filter, Port::MainL, 1.0f}); // 4
        _circuit.Connect({ModuleId::Hum,     Port::MainR, ModuleId::Filter, Port::MainR, 1.0f}); // 5
        _circuit.Connect({ModuleId::Radio,   Port::MainL, ModuleId::Filter, Port::MainL, 1.0f}); // 6
        _circuit.Connect({ModuleId::Radio,   Port::MainR, ModuleId::Filter, Port::MainR, 1.0f}); // 7
        _circuit.Connect({ModuleId::Corrupt, Port::MainL, ModuleId::Filter, Port::MainL, 1.0f}); // 8
        _circuit.Connect({ModuleId::Corrupt, Port::MainR, ModuleId::Filter, Port::MainR, 1.0f}); // 9
        _circuit.Connect({ModuleId::Filter,  Port::MainL, ModuleId::Starve, Port::MainL, 1.0f}); // 10
        _circuit.Connect({ModuleId::Filter,  Port::MainR, ModuleId::Starve, Port::MainR, 1.0f}); // 11
        _circuit.Connect({ModuleId::Starve,  Port::MainL, ModuleId::Delay,  Port::MainL, 1.0f}); // 12
        _circuit.Connect({ModuleId::Starve,  Port::MainR, ModuleId::Delay,  Port::MainR, 1.0f}); // 13

        // Virtual circuit bend bridges (default gain = 0.0f, bridged via touch)
        _circuit.Connect({ModuleId::Delay,  Port::Aux0,  ModuleId::Filter, Port::MainL, 0.0f}); // 14: Delay to Filter L
        _circuit.Connect({ModuleId::Delay,  Port::Aux1,  ModuleId::Filter, Port::MainR, 0.0f}); // 15: Delay to Filter R
        _circuit.Connect({ModuleId::Filter, Port::Aux0,  ModuleId::Filter, Port::Aux0,  0.0f}); // 16: Filter BP to Cutoff
        _circuit.Connect({ModuleId::Delay,  Port::Aux0,  ModuleId::Filter, Port::Aux0,  0.0f}); // 17: Delay to Cutoff
        _circuit.Connect({ModuleId::Drone,  Port::MainL, ModuleId::Delay,  Port::Aux0,  0.0f}); // 18: Drone to Delay Clock
        _circuit.Connect({ModuleId::Delay,  Port::Aux0,  ModuleId::Starve, Port::MainL, 0.0f}); // 19: Delay to Starve L
        _circuit.Connect({ModuleId::Delay,  Port::Aux0,  ModuleId::Starve, Port::MainR, 0.0f}); // 20: Delay to Starve R
        _circuit.Connect({ModuleId::Click,  Port::MainL, ModuleId::Delay,  Port::MainL, 0.0f}); // 21: Direct Click to Delay L
        _circuit.Connect({ModuleId::Click,  Port::MainR, ModuleId::Delay,  Port::MainR, 0.0f}); // 22: Direct Click to Delay R

        _circuit.Compile();

        _limiter_l.Init();
        _limiter_r.Init();
        _in_dc_l.Init(sample_rate);
        _in_dc_r.Init(sample_rate);

        // Boot settling & fade-in: 180ms silent DC block stabilization + 220ms smooth fade-in
        _boot_total_samples = static_cast<uint32_t>(sample_rate * 0.40f);
        _boot_samples_processed = 0;
        _reset_fade_samples = 0;
        _reset_samples_processed = 0;
    }

    void ProcessBlock(const float* in_l, const float* in_r,
                      float* out_l, float* out_r, size_t size) {
        if (_pending_reset) {
            _pending_reset = false;
            ExecuteCleanReset();
        }

        // Block-rate parameter smoothing (750 Hz at 64 samples / 48 kHz)
        daisysp::fonepole(_smooth_drive,        _target_drive,        0.02f);
        daisysp::fonepole(_smooth_pitch,        _target_pitch,        0.02f);
        daisysp::fonepole(_smooth_vca,          _target_vca,          0.05f);
        daisysp::fonepole(_smooth_sub,          _target_sub,          0.03f);
        daisysp::fonepole(_smooth_cutoff,       _target_cutoff,       0.02f);
        daisysp::fonepole(_smooth_res,          _target_res,          0.02f);
        daisysp::fonepole(_smooth_delay_time,   _target_delay_time,   0.35f);
        daisysp::fonepole(_smooth_feedback,     _target_feedback,     0.02f);
        daisysp::fonepole(_smooth_vcc,          _target_vcc,          0.02f);
        daisysp::fonepole(_smooth_mix,          _target_mix,          0.02f);
        daisysp::fonepole(_smooth_delay_mangle,     _target_delay_mangle,     0.05f);
        daisysp::fonepole(_smooth_delay_glitch,     _target_delay_glitch,     0.05f);
        daisysp::fonepole(_smooth_delay_subtle_mod, _target_delay_subtle_mod, 0.05f);
        daisysp::fonepole(_smooth_click_poly,       _target_click_poly,       0.05f);
        daisysp::fonepole(_smooth_delay_clock,      _target_delay_clock,      0.05f);
        daisysp::fonepole(_smooth_delay_reverse,    _target_delay_reverse,    0.12f);
        daisysp::fonepole(_smooth_delay_reverse_mod,_target_delay_reverse_mod,0.05f);
        daisysp::fonepole(_smooth_delay_diffusion,  _target_delay_diffusion,  0.015f);
        daisysp::fonepole(_smooth_corrupt_vol,      _target_corrupt_vol,      0.05f);
        daisysp::fonepole(_smooth_corrupt_pressure, _target_corrupt_pressure, 0.05f);

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
        float eff_corrupt_vol = _smooth_corrupt_vol;
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

        _mod_corrupt.SetParam(0, eff_corrupt_vol);
        _mod_corrupt.SetParam(1, _smooth_corrupt_pressure);
        _mod_corrupt.SetParam(2, _smooth_pitch);
        _mod_corrupt.SetDrive(_smooth_drive);

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
        _mod_delay.SetReverseMod(_smooth_delay_reverse_mod);
        _mod_delay.SetDiffusion(_smooth_delay_diffusion);

        _mod_delay.PrepareBlock(size);

        float clean_in_l[64] = {0.0f};
        float clean_in_r[64] = {0.0f};
        size_t n = (size <= 64) ? size : 64;
        for (size_t s = 0; s < n; ++s) {
            clean_in_l[s] = _in_dc_l.Process(in_l[s]);
            clean_in_r[s] = _in_dc_r.Process(in_r[s]);
        }

        _circuit.ProcessBlock(clean_in_l, clean_in_r, out_l, out_r, size);

        // Signal energy creates gentle rail breathing and audio peak tracking
        float signal_energy = 0.0f;
        float block_peak = 0.0f;
        for (size_t i = 0; i < size; ++i) {
            float abs_l = std::abs(out_l[i]);
            float abs_r = std::abs(out_r[i]);
            signal_energy += abs_l + abs_r;
            float mag = std::max(abs_l, abs_r);
            if (mag > block_peak) block_peak = mag;
        }
        signal_energy /= (2.0f * static_cast<float>(size));
        float energy_sag = signal_energy * 0.06f * (1.0f - _smooth_vcc);
        _target_vcc = std::max(0.02f, _target_vcc - energy_sag);
        _audio_peak += 0.15f * (block_peak - _audio_peak);

        _limiter_l.ProcessBlock(out_l, size, 1.0f);
        _limiter_r.ProcessBlock(out_r, size, 1.0f);

        // Settle & smooth fade-in on boot to eliminate hardware power-on DC/codec click noise
        float block_fade = 1.0f;
        if (_boot_samples_processed < _boot_total_samples) {
            uint32_t settle_samples = static_cast<uint32_t>(_sample_rate * 0.18f); // 180ms dead mute settle
            uint32_t fade_samples   = static_cast<uint32_t>(_sample_rate * 0.22f); // 220ms smooth fade-in
            if (_boot_samples_processed < settle_samples) {
                block_fade = 0.0f;
            } else {
                float progress = static_cast<float>(_boot_samples_processed - settle_samples) / static_cast<float>(fade_samples);
                progress = daisysp::fclamp(progress, 0.0f, 1.0f);
                block_fade = 0.5f * (1.0f - cosf(progress * 3.14159265f));
            }
            _boot_samples_processed += size;
        } else if (_reset_samples_processed < _reset_fade_samples) {
            float progress = static_cast<float>(_reset_samples_processed) / static_cast<float>(_reset_fade_samples);
            progress = daisysp::fclamp(progress, 0.0f, 1.0f);
            block_fade = 0.5f * (1.0f - cosf(progress * 3.14159265f));
            _reset_samples_processed += size;
        }

        for (size_t i = 0; i < size; ++i) {
            out_l[i] = daisysp::fclamp(out_l[i] * block_fade, -0.95f, 0.95f);
            out_r[i] = daisysp::fclamp(out_r[i] * block_fade, -0.95f, 0.95f);
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
    void SetDelayReverseMod(float val){ _target_delay_reverse_mod = val; }
    void SetDelayDiffusion(float val) { _target_delay_diffusion = val; }
    float GetDelayDiffusion() const   { return _smooth_delay_diffusion; }
    void SetDelaySubtleMod(float val) { _target_delay_subtle_mod = val; }
    void SetClockBend(float val)      { _mod_delay.SetClockBend(val); }
    void SetGlitchDensity(float val)  { _mod_delay.SetGlitchDensity(val); }
    void SetStutterRate(float val)    { _mod_delay.SetStutterRate(val); }
    void SetClickPoly(float val)      { _target_click_poly = val; }

    float GetVcc() const           { return _smooth_vcc; }
    float GetDrive() const         { return _smooth_drive; }
    float GetAudioPeak() const     { return _audio_peak; }

    void StrikeClick(float vel)          { _mod_click.Strike(vel); }
    void SetClickPressure(float val)     { _target_click_pressure = val; }
    void SetClickPitch(float val)        { _target_click_pitch = val; }
    void SetClickRate(float val)         { _mod_click.SetRate(val); }
    float GetGlitchSag() const     { return _mod_delay.GetGlitchSag(); }
    void TriggerBufferSkip(float strength = 1.0f) { _mod_delay.TriggerBufferSkip(strength); }

    float GetDelayTime() const     { return _mod_delay.GetCurrentDelayTime(); }
    float GetPitch() const         { return _smooth_pitch; }

    void CleanReset() {
        _pending_reset = true;
    }

private:
    void ExecuteCleanReset() {
        // 1. Immediately wipe all delay audio buffers in SDRAM and reset tap heads
        _mod_delay.ClearBuffers();

        // 2. Reset modules and feedback buffers in Circuit
        _circuit.Reset();

        // 3. Reseed submodules with fresh seed for rhythms, S&H intervals, and slice sequences
        _rng = _rng * 1664525u + 1013904223u;
        _mod_delay.Reseed(_rng);
        _mod_click.Reseed(_rng);
        _mod_corrupt.Reset();

        // 4. Clear transient states
        _target_delay_freeze = false;
        _target_delay_reverse = 0.0f;
        _smooth_delay_reverse = 0.0f;
        _target_delay_diffusion = 0.0f;
        _smooth_delay_diffusion = 0.0f;
        _target_radio_amp = 0.0f;
        _target_corrupt_vol = 0.0f;
        _smooth_corrupt_vol = 0.0f;
        _target_corrupt_pressure = 0.0f;
        _smooth_corrupt_pressure = 0.0f;
        _limiter_l.Init();
        _limiter_r.Init();

        // Smooth 40ms de-click fade-in on reset
        _reset_fade_samples = static_cast<uint32_t>(_sample_rate * 0.04f);
        _reset_samples_processed = 0;
    }

public:
    bool IsFloatingActive() const   { return false; }
    float GetWarmupProgress() const { return 0.0f; }

    // Backwards-compatible aliases
    void TriggerCircuitCrash() { CleanReset(); }
    void SetCrashLatch(bool /*on*/) { CleanReset(); }

    void SetClickDelaySync(float val)   { _mod_click.SetDelaySync(val); }
    void SetClickFilterBlend(float val) { _mod_click.SetFilterBlend(val); }
    void SetClickGroundFault(float val) { _mod_click.SetGroundFault(val); }
    void SetClickDroneSync(float val)   { _mod_click.SetDroneSync(val); }
    void SetClickTimeLeak(float val)    { _mod_click.SetTimeLeak(val); }
    void SetClickGeigerBend(float val)  { _mod_click.SetGeigerBend(val); }

    void SetCorruptVolume(float val)       { _target_corrupt_vol = val; }
    void SetCorruptPressure(float val)     { _target_corrupt_pressure = val; }
    void SetCorruptPitch(float val)        { _mod_corrupt.SetPitch(val); }
    void SetCorruptClickMod(float val)     { _mod_corrupt.SetClickMod(val); }
    void SetCorruptReverseMod(float val)   { _mod_corrupt.SetReverseMod(val); }
    void SetCorruptClockMod(float val)     { _mod_corrupt.SetClockMod(val); }
    void SetCorruptDiffusionMod(float val) { _mod_corrupt.SetDiffusionMod(val); }
    void SetCorruptGlitchMod(float val)    { _mod_corrupt.SetGlitchMod(val); }
    void SetCorruptStarveMod(float val)    { _mod_corrupt.SetStarveMod(val); }
    void SetCorruptDroneMod(float val)     { _mod_corrupt.SetDroneMod(val); }
    void TriggerCorruptPersonality()       { _mod_corrupt.TriggerNewPersonality(); }

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

    Circuit       _circuit;
    DroneModule   _mod_drone;
    ClickModule   _mod_click;
    HumModule     _mod_hum;
    RadioModule   _mod_radio;
    CorruptModule _mod_corrupt;
    FilterModule  _mod_filter;
    StarveModule  _mod_starve;
    DelayModule   _mod_delay;

    daisysp::Limiter _limiter_l;
    daisysp::Limiter _limiter_r;
    daisysp::DcBlock _in_dc_l;
    daisysp::DcBlock _in_dc_r;

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
    float _target_delay_reverse_mod{0.0f}, _smooth_delay_reverse_mod{0.0f};
    float _target_delay_diffusion{0.0f}, _smooth_delay_diffusion{0.0f};
    bool  _target_delay_freeze{false};
    float _audio_peak{0.0f};

    float _target_click_pressure{0.0f};
    float _target_click_pitch{0.5f};
    float _target_click_poly{0.0f},  _smooth_click_poly{0.0f};
    float _target_hum_buzz{0.0f};
    float _target_hum_gnd{0.0f};
    float _target_radio_amp{0.0f};
    float _target_radio_freq{1200.0f};
    float _target_corrupt_vol{0.0f},      _smooth_corrupt_vol{0.0f};
    float _target_corrupt_pressure{0.0f}, _smooth_corrupt_pressure{0.0f};
    uint32_t _rng{0xDEADBEEF};
    volatile bool _pending_reset{false};

    uint32_t _boot_total_samples{0};
    uint32_t _boot_samples_processed{0};
    uint32_t _reset_fade_samples{0};
    uint32_t _reset_samples_processed{0};
};

} // namespace bends
