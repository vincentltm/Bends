#pragma once

#include "module.h"
#include "fast_math.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

// Click and pulse generator (heartbeat / transient kick).
class ClickModule : public Module {
public:
    ClickModule() = default;

    void Init(float sample_rate, uint16_t /*block_size*/) override {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() override {
        _pulse_phase = 0.0f;
        _poly_phase = 0.0f;
        _step_counter = 0;
        _impulse_decay = 0.0f;
        _impulse_phase = 0.0f;
        _rng = 424242u;
        _pressure = 0.0f;
        _pitch = 0.5f;
        _rate = 0.5f;
        _starve = 0.0f;
        _drive = 0.2f;
        _poly_mod = 0.0f;
        _delay_sync = 0.0f;
        _filter_blend = 0.0f;
        _ground_fault = 0.0f;
        _drone_sync = 0.0f;
        _time_leak = 0.0f;
        _delay_time_sec = 0.25f;
        _drone_freq = 110.0f;
        _geiger_bend = 0.0f;
        _hold_delay = 0;
    }

    void Reseed(uint32_t seed) {
        _rng = seed ? seed : 424242u;
        _pulse_phase = static_cast<float>((_rng >> 4) & 0xFF) / 255.0f;
        _poly_phase = static_cast<float>((_rng >> 12) & 0xFF) / 255.0f;
        _hold_delay = 0;
    }

    // Trigger transient click on pad tap
    void Strike(float velocity) {
        _impulse_decay = daisysp::fclamp(velocity * 1.6f, 0.02f, 2.0f);
        _impulse_phase = 0.0f;
        _pulse_phase = 0.0f;
        _hold_delay = static_cast<uint32_t>(_sample_rate * 0.22f); // ~220ms hold delay before repeat pulses
    }

    void SetPolyMod(float val) {
        _poly_mod = daisysp::fclamp(val, 0.0f, 1.0f);
    }

    void SetRate(float val)         { _rate = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetDelaySync(float val)    { _delay_sync = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetFilterBlend(float val)  { _filter_blend = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetGroundFault(float val)  { _ground_fault = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetDroneSync(float val)    { _drone_sync = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetTimeLeak(float val)     { _time_leak = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetDelayTime(float sec)    { _delay_time_sec = std::max(0.0008f, sec); }
    void SetDroneFreq(float hz)     { _drone_freq = std::max(10.0f, hz); }
    void SetGeigerBend(float val)   { _geiger_bend = daisysp::fclamp(val, 0.0f, 1.0f); }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
        float out = 0.0f;

        // Transient kick / click: base pitch D1 (36.7 Hz) to D5 (587.3 Hz)
        if (_impulse_decay > 0.001f) {
            float pitch_dive = 1.0f + _impulse_decay * 0.65f;
            float pitch_curve = _pitch * _pitch;
            float base_f = 36.71f * FastPow2(pitch_curve * 4.0f);
            float eff_freq = base_f * pitch_dive;
            float phase_inc = (eff_freq * 6.2831853f) / _sample_rate;
            _impulse_phase += phase_inc;
            out += sinf(_impulse_phase) * _impulse_decay * 1.05f;
            float decay_rate = 0.992f + (1.0f - _pitch) * 0.0055f;
            _impulse_decay *= decay_rate;
        } else {
            _impulse_phase = 0.0f;
        }

        // Repeating pulses when pad is held (only after initial hold debounce delay)
        if (_pressure > 0.018f) {
            if (_hold_delay > 0) {
                _hold_delay--;
            } else {
                float norm_p = daisysp::fclamp((_pressure - 0.018f) / 0.982f, 0.0f, 1.0f);
                float rate_curve = _rate * _rate;
                // Slower, more spacious base tempo: 0.25 Hz (15 BPM) to 2.0 Hz (120 BPM)
                float min_rate = 0.25f + rate_curve * 1.75f;
                // Responsive acceleration on light metal contact, capped at ~5.2 Hz max
                float pressure_accel = norm_p * (0.30f + 0.70f * norm_p);
                float base_rate = min_rate + pressure_accel * 3.2f;

                // Delay subdivision sync: locked to musical echoes, clamped to <= 5.0 Hz
                if (_delay_sync > 0.02f) {
                    float delay_hz = 1.0f / std::max(0.12f, _delay_time_sec);
                    float synced_rate = daisysp::fclamp(delay_hz * 0.5f, 0.35f, 5.0f);
                    base_rate = base_rate * (1.0f - _delay_sync) + synced_rate * _delay_sync;
                }

                // Drone pitch tracking (very gentle shift)
                if (_drone_sync > 0.05f) {
                    float drone_rate = _drone_freq * 0.003f * _drone_sync;
                    base_rate += drone_rate;
                }

                // Time knob leak
                if (_time_leak > 0.01f) {
                    base_rate *= (1.0f - _time_leak * 0.35f);
                }
                base_rate = daisysp::fclamp(base_rate, 0.15f, 5.5f);

                // Continuum depth:
                // Zone 1 (depth < 0.15): Steady Metronome Heartbeat (high Vcc, unbridged)
                // Zone 2 (0.15 <= depth < 0.65): Ordered Polyrhythms / Groove (mid Vcc, pad bends)
                // Zone 3 (depth >= 0.65): Sparse Geiger Counter particle clicks (deep brownout, ground faults)
                float depth = daisysp::fclamp(_starve * 1.25f + _geiger_bend * 0.85f + _poly_mod * 0.50f, 0.0f, 1.0f);
                float eff_rate = base_rate;

                if (depth < 0.15f) {
                    // Zone 1: Pure rock-solid metronome heartbeat
                    _poly_phase = 0.0f;
                } else if (depth < 0.65f) {
                    // Zone 2: Ordered Polyrhythms / Grooving syncopation
                    float zone2_norm = (depth - 0.15f) * 2.0f; // 0.0 to 1.0

                    // Organic human swing microtiming (+/- 7%)
                    _rng = _rng * 1664525u + 1013904223u;
                    float swing = (static_cast<float>(static_cast<int32_t>(_rng)) * 2.3283e-10f) * (zone2_norm * 0.12f);
                    eff_rate = std::max(0.2f, base_rate * (1.0f + swing));

                    // Secondary polyrhythm counter-pulse (3:2 hemiola or 4:3 polyrhythm)
                    float poly_ratio = (depth < 0.40f) ? 1.5f : 1.3333333f;
                    _poly_phase += (base_rate * poly_ratio / _sample_rate);
                    if (_poly_phase >= 1.0f) {
                        _poly_phase -= 1.0f;
                        float counter_vel = 0.30f + 0.25f * zone2_norm;
                        _impulse_decay = std::max(_impulse_decay, counter_vel);
                        _impulse_phase = 0.0f;
                    }
                } else {
                    // Zone 3: Breakdown into authentic sparse Geiger counter discharges
                    float geiger_norm = (depth - 0.65f) / 0.35f;
                    _rng = _rng * 1664525u + 1013904223u;

                    // Sparse Poisson Geiger particle discharges (low density: 2-8 clicks/sec max)
                    uint32_t geiger_thresh = static_cast<uint32_t>(geiger_norm * 5.0f);
                    if ((_rng & 0x7FFF) < geiger_thresh) {
                        float particle_vel = 0.25f + (static_cast<float>(_rng & 0x7F) / 127.0f) * 0.40f;
                        _impulse_decay = std::max(_impulse_decay, particle_vel);
                        _impulse_phase = 0.0f;
                    }
                    _poly_phase = 0.0f;
                }

                // Advance primary pulse
                _pulse_phase += (eff_rate / _sample_rate);
                if (_pulse_phase >= 1.0f) {
                    _pulse_phase -= 1.0f;
                    _step_counter = (_step_counter + 1) & 0x0F;

                    bool skip_beat = false;
                    if (depth >= 0.65f) {
                        // In Geiger zone, the main pulse is increasingly erratic/broken
                        float geiger_norm = (depth - 0.65f) / 0.35f;
                        if ((static_cast<float>(_rng & 0xFF) / 255.0f) < geiger_norm * 0.65f) {
                            skip_beat = true;
                        }
                    } else if (_ground_fault > 0.08f) {
                        _rng = _rng * 1664525u + 1013904223u;
                        if ((static_cast<float>(_rng & 0xFF) / 255.0f) < _ground_fault * 0.45f) {
                            skip_beat = true;
                        }
                    }

                    if (!skip_beat) {
                        // Downbeats accented in Zone 2 for ordered groove
                        float beat_vel = 0.65f + norm_p * 0.45f;
                        if (depth >= 0.15f && depth < 0.65f && (_step_counter & 0x03) != 0) {
                            beat_vel *= 0.75f; // ghost beat on offbeats
                        }
                        _impulse_decay = std::max(_impulse_decay, beat_vel);
                        _impulse_phase = 0.0f;
                    }
                }
            }
        } else {
            _pulse_phase = 0.0f;
            _poly_phase = 0.0f;
            _hold_delay = 0;
        }

        // Output scaling: Click/pulse is always playable and clear, scaling up into punchy saturation with Drive
        float direct_gain = 1.0f - _filter_blend * 0.5f;
        float drive_gain = 0.85f + _drive * 0.75f;
        float scaled = out * drive_gain * direct_gain;
        out_l = scaled;
        out_r = scaled;
    }

    uint8_t NumParams() const override { return 5; }

    const char* GetParamName(uint8_t id) const override {
        switch (id) {
            case 0: return "Pressure";
            case 1: return "Pitch";
            case 2: return "Starve";
            case 3: return "Drive";
            case 4: return "Poly";
            default: return "";
        }
    }

    void SetParam(uint8_t id, float value) override {
        switch (id) {
            case 0: _pressure = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 1: _pitch    = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 2: _starve   = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 3: _drive    = daisysp::fclamp(value, 0.0f, 2.0f); break;
            case 4: _poly_mod = daisysp::fclamp(value, 0.0f, 1.0f); break;
        }
    }

    float GetParam(uint8_t id) const override {
        switch (id) {
            case 0: return _pressure;
            case 1: return _pitch;
            case 2: return _starve;
            case 3: return _drive;
            case 4: return _poly_mod;
            default: return 0.0f;
        }
    }

    void SetAuxInput(uint8_t channel, float value) override {
        if (channel == 0) _poly_mod = daisysp::fclamp(value, 0.0f, 1.0f);
    }

    const char* Name() const override { return "Click"; }

private:
    float _sample_rate{48000.0f};
    float _pulse_phase{0.0f};
    float _poly_phase{0.0f};
    float _impulse_decay{0.0f};
    float _impulse_phase{0.0f};
    uint32_t _rng{424242u};

    float _pressure{0.0f};
    float _pitch{0.5f};
    float _rate{0.5f};
    float _starve{0.0f};
    float _drive{0.2f};
    float _poly_mod{0.0f};
    float _delay_sync{0.0f};
    float _filter_blend{0.0f};
    float _ground_fault{0.0f};
    float _drone_sync{0.0f};
    float _time_leak{0.0f};
    float _delay_time_sec{0.25f};
    float _drone_freq{110.0f};
    float _geiger_bend{0.0f};
    uint32_t _hold_delay{0};
    uint8_t  _step_counter{0};
};

} // namespace bends
