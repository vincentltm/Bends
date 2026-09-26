#pragma once

#include "circuit.h"
#include "drone_module.h"
#include "click_module.h"
#include "hum_module.h"
#include "radio_module.h"
#include "filter_module.h"
#include "starve_module.h"
#include "delay_module.h"
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

        // Rail sag from memory glitch
        float glitch_sag = _mod_delay.GetGlitchSag();
        float lower_half_starve = daisysp::fclamp((0.50f - _smooth_vcc) * 2.0f, 0.0f, 1.0f);
        float effective_sag = glitch_sag * lower_half_starve;

        float sagged_cutoff = _smooth_cutoff * (1.0f - effective_sag * 0.45f);
        float sagged_pitch  = _smooth_pitch * (1.0f - effective_sag * 0.28f);

        _mod_drone.SetParam(0, sagged_pitch);
        _mod_drone.SetParam(1, _smooth_vca);
        _mod_drone.SetParam(2, _smooth_sub);
        _mod_drone.SetParam(3, _smooth_drive);
        _mod_drone.SetParam(4, _smooth_vcc);

        _mod_click.SetParam(0, _target_click_pressure);
        _mod_click.SetParam(1, _target_click_pitch);
        _mod_click.SetParam(2, 1.0f - _smooth_vcc);
        _mod_click.SetParam(3, _smooth_drive);
        _mod_click.SetPolyMod(_smooth_click_poly);

        _mod_hum.SetParam(0, _target_hum_buzz);
        _mod_hum.SetParam(1, _target_hum_gnd);
        _mod_hum.SetParam(2, _smooth_drive);
        _mod_hum.SetParam(3, 1.0f - _smooth_vcc);

        _mod_radio.SetParam(0, _target_radio_freq);
        _mod_radio.SetParam(1, _target_radio_amp);

        _mod_filter.SetParam(0, sagged_cutoff);
        _mod_filter.SetParam(1, _smooth_res);

        _mod_starve.SetParam(0, daisysp::fclamp(_smooth_vcc - effective_sag * 0.25f, 0.0f, 1.0f));

        _mod_delay.SetParam(0, _smooth_delay_time);
        _mod_delay.SetParam(1, _smooth_feedback);
        _mod_delay.SetParam(2, _smooth_cutoff);
        _mod_delay.SetParam(3, _smooth_mix);
        _mod_delay.SetParam(4, 1.0f - _smooth_vcc);
        _mod_delay.SetParam(5, _smooth_res);
        _mod_delay.SetMangle(_smooth_delay_mangle);
        _mod_delay.SetGlitch(_smooth_delay_glitch);
        _mod_delay.SetFreeze(_target_delay_freeze);
        _mod_delay.SetSubtleMod(_smooth_delay_subtle_mod);

        _circuit.ProcessBlock(in_l, in_r, out_l, out_r, size);

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
    void SetFeedback(float val)       { _target_feedback = val; }
    void SetVcc(float val)            { _target_vcc = val; }
    void SetMix(float val)            { _target_mix = val; }
    void SetDelayMangle(float val)    { _target_delay_mangle = val; }
    void SetDelayGlitch(float val)    { _target_delay_glitch = val; }
    void SetDelayFreeze(bool f)       { _target_delay_freeze = f; }
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
    bool  _target_delay_freeze{false};

    float _target_click_pressure{0.0f};
    float _target_click_pitch{0.5f};
    float _target_click_poly{0.0f},  _smooth_click_poly{0.0f};
    float _target_hum_buzz{0.0f};
    float _target_hum_gnd{0.0f};
    float _target_radio_amp{0.0f};
    float _target_radio_freq{1200.0f};
};

} // namespace bends
