#pragma once

#include "module.h"
#include "musical_tuning.h"
#include "daisysp.h"
#include "dev/sdram.h"
#include <cmath>
#include <algorithm>

namespace bends {

// 2.5s stereo buffer at 48kHz (external SDRAM)
static constexpr size_t kMaxDelaySamples = 120000;
static float DSY_SDRAM_BSS g_sdram_delay_l[kMaxDelaySamples];
static float DSY_SDRAM_BSS g_sdram_delay_r[kMaxDelaySamples];

// Lo-fi digital delay model: delta-modulation slew limit & op-amp filter
struct LoFiDelayModel {
    float hold_sample{0.0f};
    float clock_phase{0.0f};
    float integrator{0.0f};
    float lp1{0.0f};
    float lp2{0.0f};

    void Init() {
        Reset();
    }

    void Reset() {
        hold_sample = 0.0f;
        clock_phase = 0.0f;
        integrator = 0.0f;
        lp1 = 0.0f;
        lp2 = 0.0f;
    }

    float Process(float in, float delay_sec, float clock_sag, float sample_rate) {
        // Below 320ms, clock is fast and audio is clean
        float stretch = daisysp::fclamp((delay_sec - 0.32f) / 1.60f, 0.0f, 1.0f);
        float lo_fi = daisysp::fclamp(stretch + clock_sag * 0.75f, 0.0f, 1.0f);

        if (lo_fi <= 0.005f) {
            integrator = in;
            lp1 = in;
            lp2 = in;
            hold_sample = in;
            return in;
        }

        // Clock speed drops from 48kHz down to ~17kHz at max delay
        float clock_speed = 1.0f / (1.0f + lo_fi * 1.85f);

        // 1. Clock sample-and-hold
        clock_phase += clock_speed;
        if (clock_phase >= 1.0f) {
            clock_phase -= 1.0f;
            hold_sample = in;
        }

        // 2. Delta modulation slew limit (soft slope overload compression)
        float max_slew = 0.05f + 0.35f * (clock_speed * clock_speed);
        float diff = hold_sample - integrator;
        float step = daisysp::fclamp(diff, -max_slew, max_slew);
        integrator += step;

        // 3. Dual-stage op-amp low-pass filter (~2.6 kHz to 4.5 kHz)
        float cutoff = 4600.0f * (1.0f - lo_fi * 0.42f);
        float alpha = daisysp::fclamp(cutoff * (6.2831853f / sample_rate), 0.01f, 0.85f);
        lp1 += alpha * (integrator - lp1);
        lp2 += alpha * (lp1 - lp2);

        return lp2;
    }
};

// Stereo SDRAM delay with downsampling, wow/flutter, and granular stutter.
class DelayModule : public Module {
public:
    DelayModule() = default;

    void Init(float sample_rate, uint16_t /*block_size*/) override {
        _sample_rate = sample_rate;
        _write_ptr = 0;
        _lofi_l.Init();
        _lofi_r.Init();

        _damp_l = 0.0f;
        _damp_r = 0.0f;
        _comb_tuner.Init(0.25f);
        Reset();
    }

    void Reset() override {
        for (size_t i = 0; i < kMaxDelaySamples; i++) {
            g_sdram_delay_l[i] = 0.0f;
            g_sdram_delay_r[i] = 0.0f;
        }
        _write_ptr = 0;
        _lofi_l.Reset();
        _lofi_r.Reset();
        _damp_l = 0.0f;
        _damp_r = 0.0f;
        _fb_l = 0.0f;
        _fb_r = 0.0f;
        _aux_in[0] = 0.0f;
        _aux_in[1] = 0.0f;
        _wet_tap_l = 0.0f;
        _wet_tap_r = 0.0f;
        _current_delay_sec = _base_time_sec;
        _current_delay_samples = _base_time_sec * _sample_rate;
        _comb_tuner.Init(_base_time_sec);
        _smooth_cutoff = _cutoff;
        _mangle = 0.0f;
        _glitch = 0.0f;
        _freeze = false;
        _stutter_timer = 0;
        _stutter_offset = 0.0f;
        _subtle_mod = 0.0f;
        _sh_timer = 0;
        _sh_period = 32000;
        _sh_target_l = 0.0f;
        _sh_target_r = 0.0f;
        _sh_smooth_l = 0.0f;
        _sh_smooth_r = 0.0f;
        _tape_slip = 0.0f;
        _target_tape_slip = 0.0f;
        _packet_burst_timer = 0;
        _packet_hold_count = 0;
        _packet_held_l = 0.0f;
        _packet_held_r = 0.0f;
        _skip_click_l = 0.0f;
        _skip_click_r = 0.0f;
        _glitch_cooldown = 0;
        _glitch_density = 0.5f;
        _glitch_sag_target = 0.0f;
        _glitch_sag_smooth = 0.0f;
        _rng = 0x1337BEEF;
    }

    void SetMangle(float amount) {
        _mangle = daisysp::fclamp(amount, 0.0f, 1.0f);
    }

    void SetGlitch(float amount) {
        _glitch = daisysp::fclamp(amount, 0.0f, 1.0f);
    }

    void SetGlitchDensity(float density) {
        _glitch_density = daisysp::fclamp(density, 0.0f, 1.0f);
    }

    void SetFreeze(bool freeze) {
        _freeze = freeze;
    }

    void SetSubtleMod(float amount) {
        _subtle_mod = daisysp::fclamp(amount, 0.0f, 1.0f);
    }

    void TriggerBufferSkip(float strength = 1.0f) {
        // Buffer skip when starved (Vcc < 0.5)
        float lower_half_starve = daisysp::fclamp((_clock_sag - 0.50f) * 2.0f, 0.0f, 1.0f);
        if (lower_half_starve <= 0.01f) return;

        _rng = _rng * 1664525u + 1013904223u;
        float slip_dir = (_rng & 1) ? 1.0f : -1.0f;
        float jump = (160.0f + static_cast<float>(_rng & 0x01FF)) * daisysp::fclamp(strength, 0.5f, 1.8f) * (lower_half_starve * lower_half_starve) * (0.6f + _glitch_density * 0.7f);
        _target_tape_slip = slip_dir * jump;
        _tape_slip = _target_tape_slip;

        // Click on buffer skip
        float click_amp = 0.06f * daisysp::fclamp(strength, 0.5f, 1.5f) * lower_half_starve;
        _skip_click_l += click_amp * slip_dir;
        _skip_click_r -= click_amp * slip_dir;

        // Current sag pulse
        _glitch_sag_target = 1.0f * lower_half_starve;
    }

    float GetGlitchSag() const { return _glitch_sag_smooth; }

    inline float ReadChannel(const float* buf, float delay) const {
        float pos = static_cast<float>(_write_ptr) - delay;
        while (pos < 0.0f) pos += static_cast<float>(kMaxDelaySamples);
        while (pos >= static_cast<float>(kMaxDelaySamples)) pos -= static_cast<float>(kMaxDelaySamples);
        size_t idx0 = static_cast<size_t>(pos);
        size_t idx1 = (idx0 + 1 >= kMaxDelaySamples) ? 0 : (idx0 + 1);
        return buf[idx0] + (pos - static_cast<float>(idx0)) * (buf[idx1] - buf[idx0]);
    }

    inline void ReadDelay(float delay_l, float delay_r, float& out_l, float& out_r) const {
        out_l = ReadChannel(g_sdram_delay_l, delay_l);
        out_r = ReadChannel(g_sdram_delay_r, delay_r);
    }

    inline void WriteDelay(float l, float r) {
        g_sdram_delay_l[_write_ptr] = l;
        g_sdram_delay_r[_write_ptr] = r;
        _write_ptr++;
        if (_write_ptr >= kMaxDelaySamples) _write_ptr = 0;
    }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
        float clock_stretch = 1.0f + _clock_sag * 1.3f;
        float time_mod = std::max(0.0f, _aux_in[0]);

        // Delay time: 0.8ms to 2.25s
        float target_sec = daisysp::fclamp((_base_time_sec + time_mod * 0.8f) * clock_stretch, 0.0008f, 2.25f);

        // Tune short comb delays (< 35ms) to musical scale notes
        _current_delay_sec = _comb_tuner.Process(target_sec);
        _current_delay_samples = _current_delay_sec * _sample_rate;

        // S&H modulation (wow & chorus)
        _sh_timer++;
        if (_sh_timer >= _sh_period) {
            _sh_timer = 0;
            float rate_mult = 1.0f + _clock_sag * 1.6f + _subtle_mod * 2.2f;
            _sh_period = static_cast<uint32_t>((32000.0f + (_rng & 0x7FFF) * 1.5f) / rate_mult);
            if (_sh_period < 6000) _sh_period = 6000;

            _rng = _rng * 1664525u + 1013904223u;
            float raw_l = static_cast<float>(static_cast<int32_t>(_rng)) * 4.6566e-10f; // -1.0 to +1.0
            _rng = _rng * 1664525u + 1013904223u;
            float raw_r = static_cast<float>(static_cast<int32_t>(_rng)) * 4.6566e-10f;
            // Snap S&H to musical intervals
            _sh_target_l = MusicalTuning::QuantizeSHToHarmonic(raw_l);
            _sh_target_r = MusicalTuning::QuantizeSHToHarmonic(raw_r);
        }

        daisysp::fonepole(_sh_smooth_l, _sh_target_l, 0.0003f);
        daisysp::fonepole(_sh_smooth_r, _sh_target_r, 0.0003f);

        // S&H modulation (chorus or wow only when modulated or starved)
        float chorus_offset_l = 0.0f;
        float chorus_offset_r = 0.0f;
        float mod_amount = _subtle_mod + _clock_sag * 0.7f;

        if (mod_amount > 0.02f) {
            if (_current_delay_samples < 2800.0f) {
                float chorus_depth = mod_amount * 14.0f;
                chorus_offset_l = _sh_smooth_l * chorus_depth;
                chorus_offset_r = _sh_smooth_r * chorus_depth;
                _tape_slip = 0.0f;
                _target_tape_slip = 0.0f;
            } else {
                float wow_depth = mod_amount * 8.0f;
                chorus_offset_l = _sh_smooth_l * wow_depth;
                chorus_offset_r = _sh_smooth_r * wow_depth;
            }
        }

        float lower_half_starve = daisysp::fclamp((_clock_sag - 0.50f) * 2.0f, 0.0f, 1.0f);

        // Granular stutter slice (P09)
        if (_mangle > 0.04f) {
            float clock_t = daisysp::fclamp(_glitch_density, 0.0f, 1.0f);
            float max_len_for_clock = 72000.0f * std::pow(0.016f, clock_t);

            // Rhythmic divisions (1/1 to 1/32)
            static constexpr float kSubdivRatios[] = {
                1.0f, 0.75f, 0.50f, 0.333333f, 0.25f, 0.166667f, 0.125f, 0.083333f, 0.0625f, 0.041667f, 0.03125f
            };
            static constexpr size_t kNumSubdivs = sizeof(kSubdivRatios) / sizeof(kSubdivRatios[0]);

            // Touch pressure morphs across divisions
            float subdiv_pos = daisysp::fclamp((_mangle - 0.04f) / 0.96f * static_cast<float>(kNumSubdivs - 1), 0.0f, static_cast<float>(kNumSubdivs - 1));
            size_t s_idx0 = static_cast<size_t>(subdiv_pos);
            size_t s_idx1 = std::min(s_idx0 + 1, kNumSubdivs - 1);
            float s_frac = subdiv_pos - static_cast<float>(s_idx0);
            float s_shaped = s_frac * s_frac * (3.0f - 2.0f * s_frac);
            float subdiv = kSubdivRatios[s_idx0] + s_shaped * (kSubdivRatios[s_idx1] - kSubdivRatios[s_idx0]);
            float target_slice = max_len_for_clock * subdiv;
            uint32_t slice_len = static_cast<uint32_t>(daisysp::fclamp(target_slice, 80.0f, 72000.0f));

            _stutter_timer++;
            if (_stutter_timer >= slice_len) {
                _stutter_timer = 0;
                _repeat_count++;
                uint32_t max_repeats = std::max<uint32_t>(2, 72000 / slice_len);
                if (max_repeats > 16) max_repeats = 16;

                if (_repeat_count >= max_repeats || (_current_delay_samples + _stutter_offset + slice_len) >= (kMaxDelaySamples - 2000)) {
                    _repeat_count = 0;
                    _stutter_offset = 0.0f;
                } else {
                    _stutter_offset += static_cast<float>(slice_len);
                }
            }
        } else {
            _stutter_timer = 0;
            _stutter_offset = 0.0f;
            _repeat_count = 0;
        }

        // Delay read
        float base_read = _current_delay_samples + _stutter_offset + _tape_slip;
        float read_offset_l = daisysp::fclamp(base_read + chorus_offset_l, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
        float read_offset_r = daisysp::fclamp(base_read + chorus_offset_r, 1.0f, static_cast<float>(kMaxDelaySamples - 4));

        float delayed_l = 0.0f;
        float delayed_r = 0.0f;
        ReadDelay(read_offset_l, read_offset_r, delayed_l, delayed_r);

        // Glitch and dropouts under starve
        bool p09_active = (_mangle > 0.12f || _glitch > 0.18f);

        if (_glitch_cooldown > 0) {
            _glitch_cooldown--;
        }

        if (p09_active && lower_half_starve > 0.10f) {
            float starve_curve = lower_half_starve * lower_half_starve * lower_half_starve;
            float glitch_intensity = (_glitch * 0.50f + _mangle * 0.50f) * lower_half_starve;
            float density_factor = _glitch_density * _glitch_density;

            if (_glitch_cooldown == 0 && _packet_burst_timer == 0) {
                _rng = _rng * 1664525u + 1013904223u;
                float r = static_cast<float>(_rng & 0x00FFFFFF) / 16777216.0f;
                float trigger_prob = (0.0000005f + glitch_intensity * 0.000008f) * starve_curve * (0.10f + density_factor * 1.4f);
                if (r < trigger_prob) {
                    uint32_t burst_base = static_cast<uint32_t>((8.0f + static_cast<float>(_rng % 16)) * (1.2f - _glitch_density * 0.4f));
                    _packet_burst_timer = std::max<uint32_t>(6, burst_base);
                    _packet_hold_count = 0;

                    // Cooldown between glitch bursts
                    float cooldown_sec = (10.0f - density_factor * 6.5f) * (1.2f - lower_half_starve * 0.5f);
                    uint32_t base_cooldown = static_cast<uint32_t>(cooldown_sec * 48000.0f + static_cast<float>(_rng % 48000));
                    _glitch_cooldown = std::max<uint32_t>(144000, base_cooldown); // Min 3 seconds
                    _glitch_sag_target = 0.60f * lower_half_starve;
                }
            }

            if (_packet_burst_timer > 0) {
                _packet_burst_timer--;

                // Hold sample (corrupted audio packet)
                if (_packet_hold_count == 0) {
                    _packet_held_l = delayed_l;
                    _packet_held_r = delayed_r;
                    _packet_hold_count = static_cast<uint32_t>((3 + (_rng % 5)) * (1.4f - _glitch_density * 0.6f));

                    // Small tick at packet boundary
                    _rng = _rng * 1664525u + 1013904223u;
                    float tick_val = ((_rng & 1) ? 0.035f : -0.035f) * (0.3f + glitch_intensity * 0.7f);
                    delayed_l += tick_val;
                    delayed_r -= tick_val;
                } else {
                    _packet_hold_count--;
                    _rng = _rng * 1664525u + 1013904223u;
                    float hash = static_cast<float>(static_cast<int16_t>(_rng & 0x0FFF) - 2048) * 0.000010f;
                    delayed_l = _packet_held_l + hash;
                    delayed_r = _packet_held_r - hash;
                }
            }
        } else {
            _packet_burst_timer = 0;
            _packet_hold_count = 0;
        }

        // Click decay
        daisysp::fonepole(_skip_click_l, 0.0f, 0.20f);
        daisysp::fonepole(_skip_click_r, 0.0f, 0.20f);
        delayed_l += _skip_click_l;
        delayed_r += _skip_click_r;

        // Current sag pulse
        daisysp::fonepole(_glitch_sag_smooth, _glitch_sag_target, 0.005f);
        _glitch_sag_target *= 0.9996f;

        // Lo-fi filter: delta modulation slew limit and op-amp low-pass
        delayed_l = _lofi_l.Process(delayed_l, _current_delay_sec, _clock_sag, _sample_rate);
        delayed_r = _lofi_r.Process(delayed_r, _current_delay_sec, _clock_sag, _sample_rate);

        // Tone damping filter (Cutoff knob S31)
        _smooth_cutoff += 0.05f * (_cutoff - _smooth_cutoff);
        float damp_alpha = daisysp::fclamp(_smooth_cutoff * (6.2831853f / _sample_rate), 0.01f, 0.95f);
        _damp_l += damp_alpha * (delayed_l - _damp_l);
        _damp_r += damp_alpha * (delayed_r - _damp_r);
        float filtered_l = _damp_l;
        float filtered_r = _damp_r;

        // Soft clip
        float sat_l = daisysp::SoftClip(filtered_l / (1.0f + 0.25f * std::abs(filtered_l)));
        float sat_r = daisysp::SoftClip(filtered_r / (1.0f + 0.25f * std::abs(filtered_r)));
        _wet_tap_l = sat_l;
        _wet_tap_r = sat_r;

        // Feedback limiter to stop runaway while allowing singing resonance
        float fb_mag = 0.5f * (std::abs(sat_l) + std::abs(sat_r));
        float anti_blowout = 1.0f / (1.0f + std::max(0.0f, fb_mag - 0.80f) * 1.5f);

        float eff_feedback = daisysp::fclamp(_feedback + _aux_in[1], 0.0f, 1.20f) * anti_blowout;
        if (_freeze) {
            eff_feedback = 0.999f * anti_blowout; // Infinite loop
        }

        // Ping-pong feedback
        float cross_ratio = 0.35f;
        float fb_src_l = _freeze ? delayed_l : sat_l;
        float fb_src_r = _freeze ? delayed_r : sat_r;
        _fb_l = (fb_src_l * (1.0f - cross_ratio) + fb_src_r * cross_ratio) * eff_feedback;
        _fb_r = (fb_src_r * (1.0f - cross_ratio) + fb_src_l * cross_ratio) * eff_feedback;

        // Overdub write to delay line
        float write_l = _freeze ? (_fb_l + in_l * 0.5f) : (in_l + _fb_l);
        float write_r = _freeze ? (_fb_r + in_r * 0.5f) : (in_r + _fb_r);
        WriteDelay(write_l, write_r);

        out_l = in_l * (1.0f - _mix) + sat_l * _mix;
        out_r = in_r * (1.0f - _mix) + sat_r * _mix;

        _aux_in[0] = 0.0f;
        _aux_in[1] = 0.0f;
    }

    uint8_t NumParams() const override { return 7; }

    const char* GetParamName(uint8_t id) const override {
        switch (id) {
            case 0: return "Time";
            case 1: return "Feedback";
            case 2: return "Cutoff";
            case 3: return "Mix";
            case 4: return "ClockSag";
            case 5: return "Res";
            case 6: return "Mangle";
            default: return "";
        }
    }

    void SetParam(uint8_t id, float value) override {
        switch (id) {
            case 0: _base_time_sec = daisysp::fclamp(value, 0.0008f, 2.30f); break; // Up to 2.3s
            case 1: _feedback      = daisysp::fclamp(value, 0.0f, 1.20f); break;
            case 2: _cutoff        = daisysp::fclamp(value, 40.0f, 16000.0f); break;
            case 3: _mix           = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 4: _clock_sag     = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 5: _res           = daisysp::fclamp(value, 0.0f, 0.85f); break;
            case 6: SetMangle(value); break;
        }
    }

    float GetParam(uint8_t id) const override {
        switch (id) {
            case 0: return _base_time_sec;
            case 1: return _feedback;
            case 2: return _cutoff;
            case 3: return _mix;
            case 4: return _clock_sag;
            case 5: return _res;
            case 6: return _mangle;
            default: return 0.0f;
        }
    }

    void SetAuxInput(uint8_t channel, float value) override {
        if (channel < 2) _aux_in[channel] += value;
    }

    float GetAuxOutput(uint8_t channel) const override {
        return (channel == 0) ? _wet_tap_l : _wet_tap_r;
    }

    const char* Name() const override { return "Delay"; }

private:
    float _sample_rate{48000.0f};
    size_t _write_ptr{0};
    LoFiDelayModel _lofi_l;
    LoFiDelayModel _lofi_r;
    float _damp_l{0.0f};
    float _damp_r{0.0f};

    float _base_time_sec{0.25f};
    float _current_delay_sec{0.25f};
    MusicalTuning::CombTuner _comb_tuner;
    float _current_delay_samples{12000.0f};
    float _feedback{0.4f};
    float _cutoff{4000.0f};
    float _smooth_cutoff{4000.0f};
    float _res{0.2f};
    float _mix{0.4f};
    float _clock_sag{0.0f};

    float _mangle{0.0f};
    float _glitch{0.0f};
    bool  _freeze{false};
    float _subtle_mod{0.0f};

    // S&H modulation (wow & chorus)
    uint32_t _sh_timer{0};
    uint32_t _sh_period{32000};
    float    _sh_target_l{0.0f};
    float    _sh_target_r{0.0f};
    float    _sh_smooth_l{0.0f};
    float    _sh_smooth_r{0.0f};

    // Tape slip
    float    _tape_slip{0.0f};
    float    _target_tape_slip{0.0f};

    // Granular stutter (P09)
    uint32_t _stutter_timer{0};
    uint32_t _repeat_count{0};
    float    _stutter_offset{0.0f};
    uint32_t _rng{0x1337BEEF};

    // Glitch engine
    uint32_t _glitch_cooldown{0};
    float    _glitch_density{0.5f};
    uint32_t _packet_burst_timer{0};
    uint32_t _packet_hold_count{0};
    float    _packet_held_l{0.0f};
    float    _packet_held_r{0.0f};
    float    _skip_click_l{0.0f};
    float    _skip_click_r{0.0f};
    float    _glitch_sag_target{0.0f};
    float    _glitch_sag_smooth{0.0f};

    float _fb_l{0.0f};
    float _fb_r{0.0f};
    float _wet_tap_l{0.0f};
    float _wet_tap_r{0.0f};
    float _aux_in[2]{0.0f, 0.0f};
};

} // namespace bends
