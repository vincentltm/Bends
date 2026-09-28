#pragma once

#include "module.h"
#include "musical_tuning.h"
#include "fast_math.h"
#include "daisysp.h"
#include "dev/sdram.h"
#include <cmath>
#include <cstring>
#include <algorithm>

#include "sdram_buffer.h"
#include "diffuser.h"
#include "lofi_delay_model.h"

namespace bends {

// Stereo SDRAM delay with downsampling, wow/flutter, and granular stutter.
class DelayModule : public Module {
public:
    enum class AuxPort : uint8_t {
        ClockBend   = 0,
        FeedbackMod = 1
    };

    DelayModule() = default;

    void Init(float sample_rate, uint16_t /*block_size*/) override {
        _sample_rate = sample_rate;
        // Startup zeroing of SDRAM before audio DMA starts
        memset(g_sdram_delay_l, 0, sizeof(g_sdram_delay_l));
        memset(g_sdram_delay_r, 0, sizeof(g_sdram_delay_r));
        _write_ptr = 0;
        _lofi_l.Init();
        _lofi_r.Init();
        _dc_block_l.Reset();
        _dc_block_r.Reset();
        _in_dc_l.Reset();
        _in_dc_r.Reset();

        _damp_l = 0.0f;
        _damp_r = 0.0f;
        _comb_tuner.Init(0.25f);
        Reset();
    }

    void ClearBuffers() {
        memset(g_sdram_delay_l, 0, sizeof(g_sdram_delay_l));
        memset(g_sdram_delay_r, 0, sizeof(g_sdram_delay_r));
        _write_ptr = 0;
        _lofi_l.Reset();
        _lofi_r.Reset();
        _dc_block_l.Reset();
        _dc_block_r.Reset();
        _in_dc_l.Reset();
        _in_dc_r.Reset();
        _damp_l = 0.0f;
        _damp_r = 0.0f;
        _fb_l = 0.0f;
        _fb_r = 0.0f;
        _aux_in[0] = 0.0f;
        _aux_in[1] = 0.0f;
        _wet_tap_l = 0.0f;
        _wet_tap_r = 0.0f;
        _freeze = false;
        _freeze_phase = 0.0f;
        _freeze_anchor = 0;
        _freeze_max_len = 24000.0f;
        _freeze_clock_ref = 0.5f;
        _freeze_clock_speed_ref = 1.0f;
        _last_clock_speed = 1.0f;
        _smooth_freeze_window = 24000.0f;
        _target_freeze_window = 24000.0f;
        _smooth_freeze_speed = 1.0f;
        _target_freeze_speed = 1.0f;
        _freeze_fb_anchor = 0.5f;
        _smooth_freeze_scrub = 0.0f;
        _target_freeze_scrub = 0.0f;
        _unfreeze_xfade = 0.0f;
        _unfreeze_last_l = 0.0f;
        _unfreeze_last_r = 0.0f;
        _stutter_rate = 0.5f;
        _smooth_delay_samples = 12000.0f;
        _target_delay_samples = 12000.0f;
        _target_reverse = 0.0f;
        _smooth_reverse = 0.0f;
        _rev_phase_a = 0.0f;
        _stutter_active = false;
        _stutter_was_active = false;
        _stutter_timer = 0;
        _slice_repeat_count = 0;
        _target_hold_repeats = 2;
        _repeat_count = 0;
        _stutter_offset = 0.0f;
        _stutter_is_reverse = false;
        _reverse_mod = 0.0f;
        _stutter_seam_xfade = 0.0f;
        _stutter_last_read_offset = 0.0f;
        _stutter_smooth_gate = 0.0f;
        _skip_click_l = 0.0f;
        _skip_click_r = 0.0f;
        _glitch_sag_target = 0.0f;
        _glitch_sag_smooth = 0.0f;
        _target_clock_bend = 0.0f;
        _smooth_clock_bend = 0.0f;
        _glitch_held_l = 0.0f;
        _glitch_held_r = 0.0f;
        _packet_hold_count = 0;
        _glitch_sh_count = 0;
        _glitch_sh_l = 0.0f;
        _glitch_sh_r = 0.0f;

        _diff_l1.Reset();
        _diff_l2.Reset();
        _diff_r1.Reset();
        _diff_r2.Reset();
        _target_diffusion = 0.0f;
        _smooth_diffusion = 0.0f;
        _chorus_phase_l = 0.0f;
        _chorus_phase_r = 1.5707963f;
        _smooth_chorus_l = 0.0f;
        _smooth_chorus_r = 0.0f;
        _prep_chorus_l = 0.0f;
        _prep_chorus_r = 0.0f;
        _prep_clock_bend = 0.0f;
    }

    void Reseed(uint32_t seed) {
        _rng = seed ? seed : 0xACE12345u;
        _sh_timer = 0;
        _sh_period = 12000;
        _stutter_timer = 0;
        _slice_repeat_count = 0;
        _repeat_count = 0;
        _target_hold_repeats = 2;
    }

    void SetDiffusion(float val) {
        _target_diffusion = daisysp::fclamp(val, 0.0f, 1.0f);
    }
    float GetDiffusion() const { return _smooth_diffusion; }

    void Reset() override {
        ClearBuffers();
        _current_delay_sec = _base_time_sec;
        _current_delay_samples = _base_time_sec * _sample_rate;
        _smooth_delay_samples = _current_delay_samples;
        _target_delay_samples = _current_delay_samples;
        _old_delay_samples = _current_delay_samples;
        _last_base_time_sec = _base_time_sec;
        _time_xfade_active = false;
        _target_xfade_old = 0.0f;
        _target_xfade_new = 1.0f;
        _smooth_xfade_old = 0.0f;
        _smooth_xfade_new = 1.0f;
        _time_xfade_sample_count = 0;
        _comb_tuner.Init(_base_time_sec);
        _smooth_cutoff = _cutoff;
        _mangle = 0.0f;
        _glitch = 0.0f;
        _freeze = false;
        _freeze_phase = 0.0f;
        _freeze_anchor = 0;
        _freeze_max_len = 24000.0f;
        _freeze_clock_ref = 0.5f;
        _freeze_clock_speed_ref = 1.0f;
        _last_clock_speed = 1.0f;
        _smooth_freeze_window = 24000.0f;
        _target_freeze_window = 24000.0f;
        _smooth_freeze_speed = 1.0f;
        _target_freeze_speed = 1.0f;
        _target_reverse = 0.0f;
        _smooth_reverse = 0.0f;
        _rev_phase_a = 0.0f;
        _subtle_mod = 0.0f;
        _sh_timer = 0;
        _sh_period = 32000;
        _sh_target_l = 0.0f;
        _sh_target_r = 0.0f;
        _sh_smooth_l = 0.0f;
        _sh_smooth_r = 0.0f;
        _tape_slip = 0.0f;
        _target_tape_slip = 0.0f;
        _stutter_active = false;
        _stutter_was_active = false;
        _stutter_timer = 0;
        _slice_repeat_count = 0;
        _target_hold_repeats = 2;
        _repeat_count = 0;
        _stutter_offset = 0.0f;
        _stutter_is_reverse = false;
        _reverse_mod = 0.0f;
        _stutter_seam_xfade = 0.0f;
        _stutter_last_read_offset = 0.0f;
        _stutter_smooth_gate = 0.0f;
        _packet_burst_timer = 0;
        _packet_hold_count = 0;
        _glitch_held_l = 0.0f;
        _glitch_held_r = 0.0f;
        _glitch_sh_count = 0;
        _glitch_sh_l = 0.0f;
        _glitch_sh_r = 0.0f;
        _skip_click_l = 0.0f;
        _skip_click_r = 0.0f;
        _glitch_cooldown = 0;
        _glitch_density = 0.5f;
        _glitch_sag_target = 0.0f;
        _glitch_sag_smooth = 0.0f;
        _voltage_starve = 0.0f;
        _master_clock = 0.5f;
        _target_clock_bend = 0.0f;
        _smooth_clock_bend = 0.0f;
        _motorboat_phase = 0.0f;
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

    void SetStutterRate(float rate) {
        _stutter_rate = daisysp::fclamp(rate, 0.0f, 1.0f);
    }

    void SetReverse(float amount) {
        _target_reverse = daisysp::fclamp(amount, 0.0f, 1.0f);
    }

    void SetReverseMod(float amount) {
        _reverse_mod = daisysp::fclamp(amount, 0.0f, 1.0f);
    }

    float GetReverse() const { return _smooth_reverse; }

    void SetFreeze(bool freeze) {
        if (freeze && !_freeze) {
            _time_xfade_active = false;
            _smooth_xfade_old = 0.0f;
            _smooth_xfade_new = 1.0f;
            _freeze_anchor = _write_ptr;
            _freeze_clock_ref = _master_clock;
            _freeze_clock_speed_ref = (_last_clock_speed > 0.05f) ? _last_clock_speed : 1.0f;
            _freeze_fb_anchor = _feedback;
            _smooth_freeze_scrub = 0.0f;
            _target_freeze_scrub = 0.0f;
            _smooth_freeze_window = daisysp::fclamp(_smooth_delay_samples, 24.0f, static_cast<float>(kMaxDelaySamples - 2000));
            _target_freeze_window = _smooth_freeze_window;
            _freeze_phase = 0.0f;
            float tape_dir = 1.0f - 2.0f * _smooth_reverse;
            _smooth_freeze_speed = 1.0f * tape_dir;
            _target_freeze_speed = 1.0f * tape_dir;
        } else if (!freeze && _freeze) {
            _unfreeze_xfade = 1.0f;
        }
        _freeze = freeze;
    }

    bool GetFreeze() const { return _freeze; }
    float GetCurrentDelayTime() const { return _current_delay_sec; }

    void SetMasterClock(float val)   { _master_clock = daisysp::fclamp(val, 0.0f, 1.0f); }
    float GetMasterClock() const     { return _master_clock; }
    void SetVoltageStarve(float val) { _voltage_starve = daisysp::fclamp(val, 0.0f, 1.0f); }
    float GetVoltageStarve() const   { return _voltage_starve; }

    void SetSubtleMod(float amount) {
        _subtle_mod = daisysp::fclamp(amount, 0.0f, 1.0f);
    }

    void SetClockBend(float bend)    { _target_clock_bend = daisysp::fclamp(bend, 0.0f, 1.0f); }
    float GetClockBend() const       { return _smooth_clock_bend; }

    void TriggerBufferSkip(float strength = 1.0f) {
        _rng = _rng * 1664525u + 1013904223u;
        float slip_dir = (_rng & 1) ? 1.0f : -1.0f;
        float lower_half_starve = daisysp::fclamp((_voltage_starve - 0.40f) * 2.0f, 0.0f, 1.0f);
        float jump = (160.0f + static_cast<float>(_rng & 0x01FF)) * daisysp::fclamp(strength, 0.5f, 1.8f) * (0.5f + lower_half_starve * 0.8f);
        _target_tape_slip = slip_dir * jump;
        _tape_slip = _target_tape_slip;

        // Click on buffer skip
        float click_amp = 0.04f * daisysp::fclamp(strength, 0.5f, 1.5f) * (0.4f + lower_half_starve * 0.6f);
        _skip_click_l += click_amp * slip_dir;
        _skip_click_r -= click_amp * slip_dir;

        // Current sag pulse
        _glitch_sag_target = (0.3f + 0.7f * lower_half_starve) * daisysp::fclamp(strength, 0.5f, 1.0f);
    }

    float GetGlitchSag() const { return _glitch_sag_smooth; }

    inline float ReadPos(const float* buf, float pos) const {
        if (__builtin_isnan(pos) || __builtin_isinf(pos) || pos < -2000000000.0f || pos > 2000000000.0f) {
            return 0.0f;
        }

        int32_t i_pos = static_cast<int32_t>(pos);
        float frac = pos - static_cast<float>(i_pos);
        if (frac < 0.0f) {
            frac += 1.0f;
            i_pos -= 1;
        }

        uint32_t idx0 = static_cast<uint32_t>(i_pos) & kMaxDelayMask;
        uint32_t idx1 = (idx0 + 1) & kMaxDelayMask;

        return buf[idx0] + frac * (buf[idx1] - buf[idx0]);
    }

    inline float ReadChannel(const float* buf, float delay) const {
        return ReadPos(buf, static_cast<float>(_write_ptr) - delay);
    }

    inline void ReadDelay(float delay_l, float delay_r, float& out_l, float& out_r) const {
        out_l = ReadChannel(g_sdram_delay_l, delay_l);
        out_r = ReadChannel(g_sdram_delay_r, delay_r);
    }

    inline void WriteDelay(float l, float r) {
        g_sdram_delay_l[_write_ptr] = l;
        g_sdram_delay_r[_write_ptr] = r;
        _write_ptr = (_write_ptr + 1) & kMaxDelayMask;
    }

    void PrepareBlock(size_t block_size) {
        // Pad 06 Tape Reel Brake: physical touch gently drags the tape reel (pitch smoothly dips up to ~6 semitones)
        daisysp::fonepole(_smooth_clock_bend, _target_clock_bend, 0.08f);
        float p06_brake = 1.0f + _smooth_clock_bend * 0.45f;
        float max_buf_sec = static_cast<float>(kMaxDelaySamples - 2000) / _sample_rate;
        // Direct delay time: Knob S33 sets delay time in seconds (with P06 tape reel brake when touched).
        // Decoupled from Voltage / master_clock so changing voltage NEVER slews delay time or records Doppler wobble!
        float target_sec = daisysp::fclamp(_base_time_sec * p06_brake, 0.0008f, max_buf_sec);

        // Smooth reverse and diffusion targets
        daisysp::fonepole(_smooth_reverse, _target_reverse, 0.08f);
        daisysp::fonepole(_smooth_diffusion, _target_diffusion, 0.02f);

        // Comb tuner updated once per block
        _current_delay_sec = _comb_tuner.Process(target_sec);
        _current_delay_samples = _current_delay_sec * _sample_rate;
        _target_delay_samples = _current_delay_samples;

        // Hybrid delay time tracking:
        // Detect large jumps from turning Knob S33 (outside fine-nudge range)
        float base_diff_sec = std::abs(_base_time_sec - _last_base_time_sec);
        _last_base_time_sec = _base_time_sec;

        float delta_samples = _target_delay_samples - _smooth_delay_samples;
        if (base_diff_sec > 0.003f && std::abs(delta_samples) > 5760.0f) {
            if (!_time_xfade_active) {
                _old_delay_samples = _smooth_delay_samples;
                _time_xfade_sample_count = 0;
                _time_xfade_active = true;
                _smooth_xfade_old = 1.0f;
                _smooth_xfade_new = 0.0f;
            }
            // Update destination head to target delay time immediately
            _smooth_delay_samples = _target_delay_samples;
        }

        if (_time_xfade_active) {
            _time_xfade_sample_count += block_size;
            float progress = static_cast<float>(_time_xfade_sample_count) / static_cast<float>(kTimeXfadeSamples);
            if (progress >= 1.0f) {
                progress = 1.0f;
                _time_xfade_active = false;
            }
            float angle = 1.5707963f * progress;
            _target_xfade_old = cosf(angle);
            _target_xfade_new = sinf(angle);
        } else {
            _target_xfade_old = 0.0f;
            _target_xfade_new = 1.0f;
        }

        // Tape and chorus drift modulation updated per block
        // Non-harmonically related dual rates prevent repetitive siren cycles
        _chorus_phase_l += 0.000028f * static_cast<float>(block_size); // ~0.21 Hz
        _chorus_phase_r += 0.000037f * static_cast<float>(block_size); // ~0.28 Hz
        if (_chorus_phase_l >= 6.2831853f) _chorus_phase_l -= 6.2831853f;
        if (_chorus_phase_r >= 6.2831853f) _chorus_phase_r -= 6.2831853f;

        float sin_l = sinf(_chorus_phase_l);
        float sin_r = sinf(_chorus_phase_r);
        // Softened wave shape: removes sharp turnaround acceleration
        float soft_l = sin_l - 0.12f * sin_l * sin_l * sin_l;
        float soft_r = sin_r - 0.12f * sin_r * sin_r * sin_r;

        // Chorus window: active around Vcc 0.78 -> 0.35 (peaking at Vcc ~0.58)
        // Outside this window (including full rail Vcc = 1.0), chorus is strictly 0 so it doesn't wobble
        float vcc = 1.0f - _voltage_starve;
        float chorus_drive = 0.0f;
        if (vcc < 0.78f && vcc > 0.35f) {
            float norm_pos = (vcc - 0.58f) / 0.20f; // -1.0 to +1.0 across the window
            float bell = 1.0f - norm_pos * norm_pos;
            if (bell > 0.0f) chorus_drive = bell * bell; // Smooth quartic bell curve peaking at 1.0 when Vcc = 0.58
        }

        // Depth calibrated to chorus/wow range (3.5 - 6.0 samples)
        // Karplus-Strong mode (< 2400 samples) scales down to preserve comb tuning
        if (chorus_drive > 0.005f) {
            float ks_scale = daisysp::fclamp((_current_delay_samples - 300.0f) / 2500.0f, 0.0f, 1.0f);
            float base_depth = (3.5f + daisysp::fclamp(_current_delay_samples * 0.00025f, 0.0f, 2.5f)) * ks_scale * chorus_drive;
            _prep_chorus_l = soft_l * base_depth;
            _prep_chorus_r = soft_r * base_depth;
        } else {
            _prep_chorus_l = 0.0f;
            _prep_chorus_r = 0.0f;
        }

        // Voltage fader (Left Fader: _master_clock 1.0 = 5V full rail, 0.0 = starved)
        // Directly controls the PT2399 clock downsampling / sample rate of the delay!
        // At master_clock = 1.0 (5V full rail): full 48 kHz pristine clean delay.
        // As voltage drops towards 0V: sample rate drops smoothly down into the audible aliasing
        // zone (from 48 kHz down to ~3.8 kHz), sweeping rich metallic PT2399 aliasing sidebands!
        float live_voltage = daisysp::fclamp(_master_clock * (1.0f - _voltage_starve * 0.4f), 0.0f, 1.0f);
        _prep_clock_speed = live_voltage;
        _last_clock_speed = live_voltage;
        _lofi_l.Prepare(_current_delay_sec, live_voltage, _sample_rate);
        _lofi_r.Prepare(_current_delay_sec, live_voltage, _sample_rate);

        // Tone damping alpha
        _smooth_cutoff += 0.05f * (_cutoff - _smooth_cutoff);
        _prep_damp_alpha = daisysp::fclamp(_smooth_cutoff * (6.2831853f / _sample_rate), 0.01f, 0.95f);

        // Adaptive DC blocking coefficient:
        // In comb mode (< 45ms), cutoff adapts tracking 2 octaves below the comb fundamental note (4Hz to 45Hz)
        // to aggressively purge DC recirculating hundreds of times per second without touching resonant note pitch.
        // In echo mode (> 65ms), cutoff stays at 4Hz for full sub-bass extension.
        float comb_hz = 1.0f / std::max(0.0008f, _current_delay_sec);
        float target_dc_cutoff = daisysp::fclamp(comb_hz * 0.25f, 4.0f, 45.0f);
        float dc_cutoff_hz = (_current_delay_sec < 0.065f) ? target_dc_cutoff : 4.0f;
        _prep_dc_r = daisysp::fclamp(1.0f - (6.2831853f * dc_cutoff_hz / _sample_rate), 0.990f, 0.9997f);

        // Freeze window, granular scrub offset & varispeed targets
        if (_freeze) {
            float norm_s33_sec = _base_time_sec;
            _target_freeze_window = daisysp::fclamp(norm_s33_sec * _sample_rate, 24.0f, static_cast<float>(kMaxDelaySamples - 2000));

            // S34 Feedback knob granular scrub: offsets the freeze window across SDRAM history
            float fb_delta = _feedback - _freeze_fb_anchor;
            _target_freeze_scrub = fb_delta * 360000.0f;

            // Freeze playback speed centered dynamically around the slider position at freeze time (_freeze_clock_ref)
            // Zero pitch jump upon freeze; moving the slider above freeze position speeds up / pitches up,
            // moving below freeze position slows down / pitches down.
            float tape_dir = 1.0f - 2.0f * _smooth_reverse;
            float delta = _master_clock - _freeze_clock_ref;
            float freeze_clock_oct = 0.0f;
            if (delta >= 0.0f) {
                float range_up = std::max(0.01f, 1.0f - _freeze_clock_ref);
                freeze_clock_oct = (delta / range_up) * 1.5f; // Up to +1.5 octaves
            } else {
                float range_down = std::max(0.01f, _freeze_clock_ref);
                freeze_clock_oct = (delta / range_down) * 2.0f; // Down to -2.0 octaves
            }
            float freeze_brake = 1.0f - daisysp::fclamp(_smooth_clock_bend * 0.60f, 0.0f, 0.85f);
            _target_freeze_speed = daisysp::fclamp(FastPow2(freeze_clock_oct) * freeze_brake * tape_dir, -4.0f, 4.0f);
        }

        // Diffusion parameters
        _prep_diffusion = _smooth_diffusion;
        _prep_diff_coef = 0.48f * _prep_diffusion;

        // Mode detection precomputed at block rate
        _prep_ks_mode = daisysp::fclamp((0.055f - _current_delay_sec) * 40.0f, 0.0f, 1.0f);

        // Granular S&H memory sequencer precalculation (Pad 09 touch + bridging + air):
        if (_mangle > 0.035f) {
            float clock_t = daisysp::fclamp(_stutter_rate, 0.0f, 1.0f);
            // Master slice length: 72,000 samples (1.50s) down to 1,200 samples (25ms) across Clock Fader
            float max_len_for_clock = 72000.0f * FastPow2(-5.50f * clock_t);

            // Musical rhythmic divisions (1/1 down to 1/32)
            static constexpr float kSubdivRatios[] = {
                1.0f, 0.75f, 0.50f, 0.333333f, 0.25f, 0.166667f, 0.125f, 0.083333f, 0.0625f, 0.041667f, 0.03125f
            };
            static constexpr size_t kNumSubdivs = sizeof(kSubdivRatios) / sizeof(kSubdivRatios[0]);

            // Quadratic touch pressure curve: light touch/air stays at whole/half slices; heavy pressure drills into faster subdivisions
            float norm_mangle = daisysp::fclamp((_mangle - 0.035f) / 0.965f, 0.0f, 1.0f);
            float shaped_mangle = norm_mangle * norm_mangle;
            float subdiv_pos = shaped_mangle * static_cast<float>(kNumSubdivs - 1);
            size_t s_idx0 = static_cast<size_t>(subdiv_pos);
            size_t s_idx1 = std::min(s_idx0 + 1, kNumSubdivs - 1);
            float s_frac = subdiv_pos - static_cast<float>(s_idx0);
            float subdiv = kSubdivRatios[s_idx0] + s_frac * (kSubdivRatios[s_idx1] - kSubdivRatios[s_idx0]);
            float target_slice = max_len_for_clock * subdiv;
            _prep_stutter_slice = static_cast<uint32_t>(daisysp::fclamp(target_slice, 80.0f, 72000.0f));
        } else {
            _prep_stutter_slice = 0;
        }
    }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
        // Drone-to-Delay FM bridge: audio-rate vibrato on read pointer
        float drone_fm = _aux_in[0] * 8.0f;

        // Tape slip decays back to zero after buffer skip glitch
        daisysp::fonepole(_tape_slip, 0.0f, 0.003f);
        if (std::abs(_tape_slip) < 0.005f) _tape_slip = 0.0f;

        // S&H modulation (wow & chorus)
        _sh_timer++;
        if (_sh_timer >= _sh_period) {
            _sh_timer = 0;
            float rate_mult = 1.0f + _voltage_starve * 1.6f + _subtle_mod * 2.2f;
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

        // Smooth continuous chorus modulation (per-sample slewing prevents block-rate stepping)
        daisysp::fonepole(_smooth_chorus_l, _prep_chorus_l, 0.04f);
        daisysp::fonepole(_smooth_chorus_r, _prep_chorus_r, 0.04f);
        if (_prep_chorus_l == 0.0f && std::abs(_smooth_chorus_l) < 0.001f) _smooth_chorus_l = 0.0f;
        if (_prep_chorus_r == 0.0f && std::abs(_smooth_chorus_r) < 0.001f) _smooth_chorus_r = 0.0f;

        float chorus_offset_l = _smooth_chorus_l;
        float chorus_offset_r = _smooth_chorus_r;
        float starved_mod = (_voltage_starve > 0.35f) ? (_voltage_starve - 0.35f) * 0.7f : 0.0f;
        float mod_amount = (_subtle_mod > 0.04f ? (_subtle_mod - 0.04f) : 0.0f) + starved_mod;

        if (mod_amount > 0.01f) {
            if (_current_delay_samples < 2400.0f) {
                // In Karplus-Strong range, apply subtle synchronized vibrato to avoid destructive comb cancellation
                float comb_vibrato = _sh_smooth_l * mod_amount * 0.35f;
                chorus_offset_l += comb_vibrato;
                chorus_offset_r += comb_vibrato;
                _tape_slip = 0.0f;
                _target_tape_slip = 0.0f;
            } else {
                float wow_depth = mod_amount * 4.0f;
                chorus_offset_l += _sh_smooth_l * wow_depth;
                chorus_offset_r += _sh_smooth_r * wow_depth;
            }
        }

        // Granular S&H Memory Sequencer (Pad 09 touch + skin bridging + air proximity)
        bool stutter_active = (_mangle > 0.035f);
        if (stutter_active && !_stutter_was_active) {
            _stutter_timer = 0;
            _slice_repeat_count = 0;
            uint32_t slice_len = std::max<uint32_t>(80, _prep_stutter_slice);
            uint32_t base_reps = std::max<uint32_t>(2, 48000 / slice_len);
            uint32_t max_reps = std::min<uint32_t>(12, base_reps);
            _rng = _rng * 1664525u + 1013904223u;
            _target_hold_repeats = 2 + ((_rng >> 8) % std::max<uint32_t>(1, max_reps - 1));
            _stutter_offset = 0.0f;
            _stutter_is_reverse = false;
            _stutter_seam_xfade = 0.0f;
        } else if (!stutter_active && _stutter_was_active) {
            _stutter_timer = 0;
            _stutter_offset = 0.0f;
            _slice_repeat_count = 0;
            _stutter_is_reverse = false;
        }
        _stutter_was_active = stutter_active;

        daisysp::fonepole(_stutter_smooth_gate, stutter_active ? 1.0f : 0.0f, 0.04f);

        if (stutter_active && _prep_stutter_slice > 0) {
            uint32_t slice_len = _prep_stutter_slice;
            _stutter_timer++;
            if (_stutter_timer >= slice_len) {
                _stutter_timer = 0;
                _slice_repeat_count++;

                if (_slice_repeat_count >= _target_hold_repeats) {
                    _slice_repeat_count = 0;
                    _repeat_count++;

                    // Scale hold repeat count to slice duration:
                    // Large slices (500ms-1.5s): 2-3 repeats (phrase groove)
                    // Medium slices (150ms-500ms): 3-5 repeats
                    // Glitch slices (<100ms): 6-12 repeats (buzzing burst)
                    uint32_t base_reps = std::max<uint32_t>(2, 48000 / slice_len);
                    uint32_t max_reps = std::min<uint32_t>(12, base_reps);
                    _rng = _rng * 1664525u + 1013904223u;
                    _target_hold_repeats = 2 + ((_rng >> 8) % std::max<uint32_t>(1, max_reps - 1));

                    // S&H memory slice picker: select random slice in past SDRAM buffer
                    uint32_t max_slices = std::min<uint32_t>(12, static_cast<uint32_t>((kMaxDelaySamples - 4000) / slice_len));
                    if (max_slices < 2) max_slices = 2;

                    _rng = _rng * 1664525u + 1013904223u;
                    uint32_t next_slice_idx = (_rng >> 12) % max_slices;

                    // Accidental patching / bridging P09 with P05:
                    // Randomly alternate forward and reverse on slice boundaries!
                    if (_reverse_mod > 0.03f) {
                        _rng = _rng * 1664525u + 1013904223u;
                        uint32_t thresh = static_cast<uint32_t>(_reverse_mod * 255.0f);
                        _stutter_is_reverse = ((_rng & 0xFF) < thresh);
                    } else {
                        _stutter_is_reverse = false;
                    }

                    _stutter_offset = static_cast<float>(next_slice_idx * slice_len);
                    _stutter_seam_xfade = 1.0f;
                }
            }
        }

        float delayed_l = 0.0f;
        float delayed_r = 0.0f;

        if (_freeze) {
            // Smooth window size, granular scrub offset, and playback speed during freeze
            daisysp::fonepole(_smooth_freeze_window, _target_freeze_window, 0.002f);
            daisysp::fonepole(_smooth_freeze_speed, _target_freeze_speed, 0.002f);
            daisysp::fonepole(_smooth_freeze_scrub, _target_freeze_scrub, 0.005f);

            // Advance freeze phase smoothly through the active window with instant modulo wrapping
            float window = std::max(24.0f, _smooth_freeze_window);
            _freeze_phase += _smooth_freeze_speed;
            if (_freeze_phase >= window || _freeze_phase < 0.0f) {
                float wraps = std::floor(_freeze_phase / window);
                _freeze_phase -= wraps * window;
            }
            if (_freeze_phase < 0.0f || _freeze_phase >= window || __builtin_isnan(_freeze_phase)) {
                _freeze_phase = 0.0f;
            }

            // Read frozen loop from SDRAM with granular scrub offset & S&H sequencer offset
            float rev_sweep = _stutter_is_reverse ? (2.0f * static_cast<float>(_stutter_timer)) : 0.0f;
            float stutter_shift = (_stutter_offset + rev_sweep) * _stutter_smooth_gate;
            float frozen_base_pos = static_cast<float>(_freeze_anchor) - window + _freeze_phase + _smooth_freeze_scrub - stutter_shift;
            float frozen_pos_l = frozen_base_pos + chorus_offset_l + _tape_slip + drone_fm;
            float frozen_pos_r = frozen_base_pos + chorus_offset_r + _tape_slip + drone_fm;

            delayed_l = ReadPos(g_sdram_delay_l, frozen_pos_l);
            delayed_r = ReadPos(g_sdram_delay_r, frozen_pos_r);

            // Smoothstep cubic envelope at window seams
            float seam_len = std::min(32.0f, window * 0.12f);
            if (_freeze_phase < seam_len) {
                float norm = _freeze_phase / seam_len;
                float gain = norm * norm * (3.0f - 2.0f * norm);
                delayed_l *= gain;
                delayed_r *= gain;
            } else if (_freeze_phase > (window - seam_len)) {
                float norm = (window - _freeze_phase) / seam_len;
                float gain = norm * norm * (3.0f - 2.0f * norm);
                delayed_l *= gain;
                delayed_r *= gain;
            }

            _unfreeze_last_l = delayed_l;
            _unfreeze_last_r = delayed_r;
        } else {
            if (!_time_xfade_active) {
                // Direct, responsive per-sample delay tracking for small adjustments / nudges:
                // Lengthening: max_up = 0.80f strictly guarantees forward speed >= 0.20x (pitch dips, tape slows).
                // Shortening: max_down = 2.00f runs forward speed <= 3.00x (pitch rises, tape speeds).
                // In comb mode (< 2400 samples): track_alpha = 0.04f, max_step = 0.85f for melodic pitch tracking.
                float track_alpha = (_target_delay_samples < 2400.0f) ? 0.04f : 0.008f;
                float max_up   = (_target_delay_samples < 2400.0f) ? 0.85f : 0.80f;
                float max_down = (_target_delay_samples < 2400.0f) ? 0.85f : 2.00f;
                float delta = _target_delay_samples - _smooth_delay_samples;
                if (std::abs(delta) > 0.25f) {
                    _smooth_delay_samples += daisysp::fclamp(delta * track_alpha, -max_down, max_up);
                }
            }

            // Normal delay read (target head) with S&H memory sequencer offset
            float rev_sweep = _stutter_is_reverse ? (2.0f * static_cast<float>(_stutter_timer)) : 0.0f;
            float stutter_shift = (_stutter_offset + rev_sweep) * _stutter_smooth_gate;
            float normal_base_read = _smooth_delay_samples + stutter_shift + _tape_slip + drone_fm;
            float normal_read_l = daisysp::fclamp(normal_base_read + chorus_offset_l, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
            float normal_read_r = daisysp::fclamp(normal_base_read + chorus_offset_r, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
            ReadDelay(normal_read_l, normal_read_r, delayed_l, delayed_r);

            // Slice boundary crossfade for S&H sequencer
            if (_stutter_seam_xfade > 0.001f && _stutter_smooth_gate > 0.01f) {
                float seam_read_l = daisysp::fclamp(_stutter_last_read_offset + chorus_offset_l, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
                float seam_read_r = daisysp::fclamp(_stutter_last_read_offset + chorus_offset_r, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
                float seam_l = ReadChannel(g_sdram_delay_l, seam_read_l);
                float seam_r = ReadChannel(g_sdram_delay_r, seam_read_r);
                delayed_l = delayed_l * (1.0f - _stutter_seam_xfade) + seam_l * _stutter_seam_xfade;
                delayed_r = delayed_r * (1.0f - _stutter_seam_xfade) + seam_r * _stutter_seam_xfade;
                _stutter_seam_xfade *= 0.92f;
                _stutter_last_read_offset += 1.0f;
            } else {
                _stutter_last_read_offset = normal_base_read;
            }

            // Equal-power crossfade during large time jumps
            daisysp::fonepole(_smooth_xfade_old, _target_xfade_old, 0.04f);
            daisysp::fonepole(_smooth_xfade_new, _target_xfade_new, 0.04f);
            if (_smooth_xfade_old > 0.002f) {
                float old_base_read = _old_delay_samples + _tape_slip + drone_fm;
                float old_read_l = daisysp::fclamp(old_base_read + chorus_offset_l, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
                float old_read_r = daisysp::fclamp(old_base_read + chorus_offset_r, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
                float old_del_l, old_del_r;
                ReadDelay(old_read_l, old_read_r, old_del_l, old_del_r);
                delayed_l = old_del_l * _smooth_xfade_old + delayed_l * _smooth_xfade_new;
                delayed_r = old_del_r * _smooth_xfade_old + delayed_r * _smooth_xfade_new;
            } else {
                _smooth_xfade_old = 0.0f;
                _smooth_xfade_new = 1.0f;
            }

            // Smooth unfreeze crossfade if exiting freeze mode
            if (_unfreeze_xfade > 0.001f) {
                delayed_l = delayed_l * (1.0f - _unfreeze_xfade) + _unfreeze_last_l * _unfreeze_xfade;
                delayed_r = delayed_r * (1.0f - _unfreeze_xfade) + _unfreeze_last_r * _unfreeze_xfade;
                _unfreeze_xfade *= 0.992f;
            }

            // Dual-head crossfaded reverse delay in normal mode
            if (_smooth_reverse > 0.001f) {
                // Chunk length: tracks delay time, bounded between 50ms and 1.0s for musical reverse slices
                float rev_chunk_len = daisysp::fclamp(_smooth_delay_samples, 2400.0f, 48000.0f);
                _rev_phase_a += 1.0f;
                if (_rev_phase_a >= rev_chunk_len || _rev_phase_a < 0.0f) {
                    _rev_phase_a = 0.0f;
                }
                float rev_phase_b = _rev_phase_a + rev_chunk_len * 0.5f;
                if (rev_phase_b >= rev_chunk_len) {
                    rev_phase_b -= rev_chunk_len;
                }

                // In rev_chunk_len output samples, read head sweeps backward at rate 1.0 (read delay offset increases at rate 2.0)
                // Offset is centered around _smooth_delay_samples so reverse echoes land at the exact delay time
                float base_offset = std::max(32.0f, _smooth_delay_samples - rev_chunk_len);
                float offset_a = base_offset + 2.0f * _rev_phase_a;
                float offset_b = base_offset + 2.0f * rev_phase_b;

                // Hann window crossfade: gain_a + gain_b = 1.0 unconditionally
                // Each head reaches 0.0 gain exactly at its wrap turnaround point, guaranteeing zero clicks!
                float norm_a = _rev_phase_a / rev_chunk_len;
                float cos_val = cosf(6.2831853f * norm_a);
                float gain_a = 0.5f * (1.0f - cos_val);
                float gain_b = 1.0f - gain_a;

                float read_a_l = daisysp::fclamp(offset_a + chorus_offset_l, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
                float read_a_r = daisysp::fclamp(offset_a + chorus_offset_r, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
                float read_b_l = daisysp::fclamp(offset_b + chorus_offset_l, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
                float read_b_r = daisysp::fclamp(offset_b + chorus_offset_r, 1.0f, static_cast<float>(kMaxDelaySamples - 4));

                float rev_l = gain_a * ReadChannel(g_sdram_delay_l, read_a_l) + gain_b * ReadChannel(g_sdram_delay_l, read_b_l);
                float rev_r = gain_a * ReadChannel(g_sdram_delay_r, read_a_r) + gain_b * ReadChannel(g_sdram_delay_r, read_b_r);

                delayed_l = delayed_l * (1.0f - _smooth_reverse) + rev_l * _smooth_reverse;
                delayed_r = delayed_r * (1.0f - _smooth_reverse) + rev_r * _smooth_reverse;
            } else {
                _rev_phase_a = 0.0f;
            }
        }

        // Glitch Engine (Pad 09 firm pressure & Starve): Bit decimation & CD-skip dropouts
        float lower_half_starve = daisysp::fclamp((_voltage_starve - 0.40f) * 2.0f, 0.0f, 1.0f);
        if (_glitch > 0.08f || lower_half_starve > 0.15f) {
            float p_glitch = daisysp::fclamp((_glitch - 0.08f) / 0.92f, 0.0f, 1.0f);

            // 1. Bit decimation (quantization steps reduce from 2048 to 32)
            if (p_glitch > 0.10f || lower_half_starve > 0.25f) {
                float q_norm = std::max(p_glitch > 0.10f ? (p_glitch - 0.10f) / 0.90f : 0.0f, lower_half_starve * 0.70f);
                float steps = 2048.0f / FastPow2(q_norm * 5.0f);
                delayed_l = std::round(delayed_l * steps) / steps;
                delayed_r = std::round(delayed_r * steps) / steps;
            }

            // 2. Sample-and-hold decimation (clock sub-sampling)
            if (p_glitch > 0.30f || lower_half_starve > 0.45f) {
                float sh_drive = std::max(p_glitch > 0.30f ? (p_glitch - 0.30f) : 0.0f, lower_half_starve * 0.60f);
                uint32_t decimate = 1 + static_cast<uint32_t>(sh_drive * 6.0f);
                _glitch_sh_count++;
                if (_glitch_sh_count >= decimate) {
                    _glitch_sh_count = 0;
                    _glitch_sh_l = delayed_l;
                    _glitch_sh_r = delayed_r;
                } else {
                    delayed_l = _glitch_sh_l;
                    delayed_r = _glitch_sh_r;
                }
            }

            // 3. CD-skip packet hold dropouts under heavy pressure (> 0.55) or starve
            if (p_glitch > 0.55f || (lower_half_starve > 0.20f && _glitch > 0.05f)) {
                if (_glitch_cooldown > 0) {
                    _glitch_cooldown--;
                } else if (_packet_burst_timer == 0) {
                    _rng = _rng * 1664525u + 1013904223u;
                    float r = static_cast<float>(_rng & 0x00FFFFFF) * 5.9604645e-8f;
                    float prob = (p_glitch > 0.55f ? (p_glitch - 0.55f) * 0.0004f : 0.0f) + lower_half_starve * 0.0003f;
                    if (r < prob) {
                        _packet_burst_timer = 12 + (_rng % 28);
                        _glitch_cooldown = 2400 + (_rng % 2400);
                        _glitch_held_l = delayed_l;
                        _glitch_held_r = delayed_r;
                        _packet_hold_count = 0;
                    }
                }

                if (_packet_burst_timer > 0) {
                    _packet_burst_timer--;
                    if (_packet_hold_count == 0) {
                        _glitch_held_l = delayed_l;
                        _glitch_held_r = delayed_r;
                        _packet_hold_count = 3 + (_rng % 5);
                        _rng = _rng * 1664525u + 1013904223u;
                        float tick_val = ((_rng & 1) ? 0.035f : -0.035f) * (0.3f + p_glitch * 0.7f);
                        delayed_l += tick_val;
                        delayed_r -= tick_val;
                    } else {
                        _packet_hold_count--;
                        _rng = _rng * 1664525u + 1013904223u;
                        float hash = static_cast<float>(static_cast<int16_t>(_rng & 0x0FFF) - 2048) * 0.000010f;
                        delayed_l = _glitch_held_l + hash;
                        delayed_r = _glitch_held_r - hash;
                    }
                }
            }
        }

        // Click decay
        daisysp::fonepole(_skip_click_l, 0.0f, 0.20f);
        daisysp::fonepole(_skip_click_r, 0.0f, 0.20f);
        delayed_l += _skip_click_l;
        delayed_r += _skip_click_r;

        // Current sag pulse
        daisysp::fonepole(_glitch_sag_smooth, _glitch_sag_target, 0.005f);
        _glitch_sag_target *= 0.9996f;

        // Lo-fi filter: lightweight per-sample processing
        delayed_l = _lofi_l.Process(delayed_l);
        delayed_r = _lofi_r.Process(delayed_r);

        // Tone damping filter (Cutoff knob S31) using block-precomputed alpha
        _damp_l += _prep_damp_alpha * (delayed_l - _damp_l);
        _damp_r += _prep_damp_alpha * (delayed_r - _damp_r);

        // Adaptive DC blocker on delay feedback path (prevents DC rail lockup on short delay times)
        float ac_l = _dc_block_l.Process(_damp_l, _prep_dc_r);
        float ac_r = _dc_block_r.Process(_damp_r, _prep_dc_r);

        // Clean soft clip that preserves loop gain for Karplus-Strong & singing delays
        float sat_l = daisysp::SoftClip(ac_l / (1.0f + 0.15f * std::abs(ac_l)));
        float sat_r = daisysp::SoftClip(ac_r / (1.0f + 0.15f * std::abs(ac_r)));

        // Mode: 1.0 = Karplus-Strong comb (< 35ms), 0.0 = Echo delay (> 60ms)
        float ks_mode = _prep_ks_mode;

        // Dynamic feedback anti-blowout compressor: protects against runaway positive feedback on ALL delay times!
        float fb_mag = 0.5f * (std::abs(sat_l) + std::abs(sat_r));
        float anti_blowout = 1.0f / (1.0f + std::max(0.0f, fb_mag - 0.60f) * 3.0f);

        // Maximum allowable feedback: short times must stay < 0.97 to prevent runaway positive feedback howl
        float max_allowable_fb = 1.08f * (1.0f - ks_mode) + 0.965f * ks_mode;
        float raw_fb = _freeze ? 0.999f : daisysp::fclamp(_feedback + _aux_in[1], 0.0f, max_allowable_fb);
        float eff_feedback = raw_fb * anti_blowout;

        // Feedback source
        float fb_src_l = _freeze ? delayed_l : sat_l;
        float fb_src_r = _freeze ? delayed_r : sat_r;

        // Schroeder/Dattorro all-pass diffusion with smooth crossfade
        if (_prep_diffusion > 0.001f) {
            float diff_l = _diff_l2.Process(_diff_l1.Process(fb_src_l, _prep_diff_coef), _prep_diff_coef);
            float diff_r = _diff_r2.Process(_diff_r1.Process(fb_src_r, _prep_diff_coef), _prep_diff_coef);

            float fb_blend = _prep_diffusion;
            fb_src_l = fb_src_l * (1.0f - fb_blend) + diff_l * fb_blend;
            fb_src_r = fb_src_r * (1.0f - fb_blend) + diff_r * fb_blend;

            float tap_blend = _prep_diffusion * 0.70f;
            sat_l = sat_l * (1.0f - tap_blend) + diff_l * tap_blend;
            sat_r = sat_r * (1.0f - tap_blend) + diff_r * tap_blend;
        }

        _wet_tap_l = sat_l;
        _wet_tap_r = sat_r;

        // Ping-pong cross-coupling on echo delays; dual mono on Karplus-Strong to avoid phase cancellation
        float cross_ratio = 0.35f * (1.0f - ks_mode);
        _fb_l = (fb_src_l * (1.0f - cross_ratio) + fb_src_r * cross_ratio) * eff_feedback;
        _fb_r = (fb_src_r * (1.0f - cross_ratio) + fb_src_l * cross_ratio) * eff_feedback;

        // Overdub write to delay line (halted when frozen so loop is preserved)
        // DC is blocked on the input path to prevent any bias accumulation in SDRAM
        if (!_freeze) {
            float in_ac_l = _in_dc_l.Process(in_l, _prep_dc_r);
            float in_ac_r = _in_dc_r.Process(in_r, _prep_dc_r);
            float write_l = in_ac_l + _fb_l;
            float write_r = in_ac_r + _fb_r;
            WriteDelay(write_l, write_r);
        }

        out_l = in_l * (1.0f - _mix) + sat_l * _mix;
        out_r = in_r * (1.0f - _mix) + sat_r * _mix;

        _aux_in[0] = 0.0f;
        _aux_in[1] = 0.0f;
    }

    uint8_t NumParams() const override { return 8; }

    const char* GetParamName(uint8_t id) const override {
        switch (id) {
            case 0: return "Time";
            case 1: return "Feedback";
            case 2: return "Cutoff";
            case 3: return "Mix";
            case 4: return "Starve";
            case 5: return "Res";
            case 6: return "Mangle";
            case 7: return "Clock";
            default: return "";
        }
    }

    void SetParam(uint8_t id, float value) override {
        switch (id) {
            case 0: _base_time_sec  = daisysp::fclamp(value, 0.0008f, 10.50f); break; // Up to 10.5s tape loop buffer
            case 1: _feedback       = daisysp::fclamp(value, 0.0f, 1.20f); break;
            case 2: _cutoff         = daisysp::fclamp(value, 40.0f, 16000.0f); break;
            case 3: _mix            = daisysp::fclamp(value, 0.0f, 1.0f); break;
            case 4: SetVoltageStarve(value); break;
            case 5: _res            = daisysp::fclamp(value, 0.0f, 0.85f); break;
            case 6: SetMangle(value); break;
            case 7: SetMasterClock(value); break;
        }
    }

    float GetParam(uint8_t id) const override {
        switch (id) {
            case 0: return _base_time_sec;
            case 1: return _feedback;
            case 2: return _cutoff;
            case 3: return _mix;
            case 4: return _voltage_starve;
            case 5: return _res;
            case 6: return _mangle;
            case 7: return _master_clock;
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
    struct AdaptiveDcBlocker {
        float x{0.0f};
        float y{0.0f};

        inline void Reset() {
            x = 0.0f;
            y = 0.0f;
        }

        inline float Process(float in, float r) {
            float out = in - x + r * y;
            x = in;
            y = out;
            return out;
        }
    };

    AdaptiveDcBlocker _dc_block_l;
    AdaptiveDcBlocker _dc_block_r;
    AdaptiveDcBlocker _in_dc_l;
    AdaptiveDcBlocker _in_dc_r;
    float _prep_dc_r{0.9995f};
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
    float _voltage_starve{0.0f};
    float _master_clock{0.5f};
    float _motorboat_phase{0.0f};

    float _mangle{0.0f};
    float _glitch{0.0f};
    bool   _freeze{false};
    float  _freeze_max_len{24000.0f};
    float  _freeze_clock_ref{0.5f};
    float  _freeze_clock_speed_ref{1.0f};
    float  _last_clock_speed{1.0f};
    float  _freeze_phase{0.0f};
    float  _smooth_freeze_window{24000.0f};
    float  _target_freeze_window{24000.0f};
    float  _smooth_freeze_speed{1.0f};
    float  _target_freeze_speed{1.0f};
    float  _freeze_fb_anchor{0.5f};
    float  _smooth_freeze_scrub{0.0f};
    float  _target_freeze_scrub{0.0f};
    float  _unfreeze_xfade{0.0f};
    float  _unfreeze_last_l{0.0f};
    float  _unfreeze_last_r{0.0f};
    float  _stutter_rate{0.5f};
    size_t _freeze_anchor{0};
    float  _smooth_delay_samples{12000.0f};
    float  _target_delay_samples{12000.0f};
    float  _target_reverse{0.0f};
    float  _smooth_reverse{0.0f};
    float  _rev_phase_a{0.0f};
    float  _subtle_mod{0.0f};

    // Hybrid delay time tracking & jump crossfade
    float    _last_base_time_sec{0.25f};
    float    _old_delay_samples{12000.0f};
    float    _target_xfade_old{0.0f};
    float    _target_xfade_new{1.0f};
    float    _smooth_xfade_old{0.0f};
    float    _smooth_xfade_new{1.0f};
    uint16_t _time_xfade_sample_count{0};
    bool     _time_xfade_active{false};
    static constexpr uint16_t kTimeXfadeSamples{1680}; // ~35ms at 48kHz

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

    // Granular S&H memory sequencer (P09)
    bool     _stutter_active{false};
    bool     _stutter_was_active{false};
    uint32_t _stutter_timer{0};
    uint32_t _repeat_count{0};
    uint32_t _slice_repeat_count{0};
    uint32_t _target_hold_repeats{2};
    float    _stutter_offset{0.0f};
    bool     _stutter_is_reverse{false};
    float    _reverse_mod{0.0f};
    float    _stutter_seam_xfade{0.0f};
    float    _stutter_last_read_offset{0.0f};
    float    _stutter_smooth_gate{0.0f};
    uint32_t _rng{0x1337BEEF};

    // Glitch engine
    uint32_t _glitch_cooldown{0};
    float    _glitch_density{0.5f};
    uint32_t _packet_burst_timer{0};
    uint32_t _packet_hold_count{0};
    float    _glitch_held_l{0.0f};
    float    _glitch_held_r{0.0f};
    uint32_t _glitch_sh_count{0};
    float    _glitch_sh_l{0.0f};
    float    _glitch_sh_r{0.0f};
    float    _skip_click_l{0.0f};
    float    _skip_click_r{0.0f};
    float    _glitch_sag_target{0.0f};
    float    _glitch_sag_smooth{0.0f};
    float    _target_clock_bend{0.0f};
    float    _smooth_clock_bend{0.0f};

    float _fb_l{0.0f};
    float _fb_r{0.0f};
    float _wet_tap_l{0.0f};
    float _wet_tap_r{0.0f};
    float _aux_in[2]{0.0f, 0.0f};

    // 2-stage stereo allpass diffusers with prime delay lengths
    DiffuserStage<227> _diff_l1;
    DiffuserStage<563> _diff_l2;
    DiffuserStage<307> _diff_r1;
    DiffuserStage<709> _diff_r2;
    float _target_diffusion{0.0f};
    float _smooth_diffusion{0.0f};

    // Vintage BBD / tape chorus drift
    float _chorus_phase_l{0.0f};
    float _chorus_phase_r{1.5707963f}; // 90 deg initial phase offset
    float _smooth_chorus_l{0.0f};
    float _smooth_chorus_r{0.0f};

    // Block-rate prepared parameters
    float _prep_clock_speed{1.0f};
    float _prep_chorus_l{0.0f};
    float _prep_chorus_r{0.0f};
    float _prep_damp_alpha{0.1f};
    float _prep_diffusion{0.0f};
    float _prep_diff_coef{0.0f};
    float _prep_clock_bend{0.0f};
    float _prep_ks_mode{0.0f};
    uint32_t _prep_stutter_slice{0};
    uint32_t _prep_stutter_max_repeats{0};
};

} // namespace bends
