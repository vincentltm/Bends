#pragma once

#include "module.h"
#include "musical_tuning.h"
#include "daisysp.h"
#include "dev/sdram.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace bends {

// Up to 8.33s stereo buffer at 48kHz (external SDRAM: 400,000 samples = ~3.2 MB)
static constexpr size_t kMaxDelaySamples = 400000;
static float DSY_SDRAM_BSS g_sdram_delay_l[kMaxDelaySamples];
static float DSY_SDRAM_BSS g_sdram_delay_r[kMaxDelaySamples];

// Lightweight Schroeder/Dattorro all-pass diffuser stage in internal SRAM
template <size_t kDelaySize>
struct DiffuserStage {
    float buf[kDelaySize]{0.0f};
    size_t pos{0};

    void Reset() {
        std::memset(buf, 0, sizeof(buf));
        pos = 0;
    }

    inline float Process(float in, float coef) {
        float y = buf[pos];
        float z = coef * y + in;
        buf[pos] = z;
        pos++;
        if (pos >= kDelaySize) pos = 0;
        return y - coef * z;
    }
};

// Lo-fi digital delay model: delta-modulation slew limit & op-amp filter
struct LoFiDelayModel {
    float hold_sample{0.0f};
    float clock_phase{0.0f};
    float integrator{0.0f};
    float lp1{0.0f};
    float clock_speed{1.0f};
    float max_slew{10.0f};
    float alpha{0.85f};
    bool active{false};

    void Init() {
        Reset();
    }

    void Reset() {
        hold_sample = 0.0f;
        clock_phase = 0.0f;
        integrator = 0.0f;
        lp1 = 0.0f;
        clock_speed = 1.0f;
        max_slew = 10.0f;
        alpha = 0.85f;
        active = false;
    }

    inline void Prepare(float delay_sec, float clock_sag, float sample_rate) {
        float ks_blend = daisysp::fclamp((delay_sec - 0.020f) * 33.3333f, 0.0f, 1.0f);
        // Below 320ms, clock is fast and audio is clean
        float stretch = daisysp::fclamp((delay_sec - 0.32f) * 0.625f, 0.0f, 1.0f);
        float lo_fi = daisysp::fclamp(stretch + clock_sag * 0.75f, 0.0f, 1.0f);

        float active_lo_fi = lo_fi * (0.15f + 0.85f * ks_blend);
        if (active_lo_fi <= 0.005f) {
            active = false;
            return;
        }

        active = true;
        // Clock speed drops from 48kHz down to ~17kHz at max delay
        clock_speed = 1.0f / (1.0f + active_lo_fi * 1.85f);
        max_slew = 0.05f + 0.35f * (clock_speed * clock_speed) + (1.0f - ks_blend) * 10.0f;
        float cutoff = (4600.0f * (1.0f - lo_fi * 0.42f)) * ks_blend + 18000.0f * (1.0f - ks_blend);
        alpha = daisysp::fclamp(cutoff * (6.2831853f / sample_rate), 0.01f, 0.85f);
    }

    inline float Process(float in) {
        if (!active) {
            integrator = in;
            lp1 = in;
            hold_sample = in;
            return in;
        }

        // 1. Clock sample-and-hold (authentic PT2399 downsampling aliasing)
        clock_phase += clock_speed;
        if (clock_phase >= 1.0f) {
            clock_phase -= 1.0f;
            hold_sample = in;
        }

        // 2. Delta modulation slew limit
        float diff = hold_sample - integrator;
        float step = daisysp::fclamp(diff, -max_slew, max_slew);
        integrator += step;

        // 3. Fast 1-pole op-amp low-pass filter
        lp1 += alpha * (integrator - lp1);
        return lp1;
    }
};

// Stereo SDRAM delay with downsampling, wow/flutter, and granular stutter.
class DelayModule : public Module {
public:
    DelayModule() = default;

    void Init(float sample_rate, uint16_t /*block_size*/) override {
        _sample_rate = sample_rate;
        // One-time startup zeroing of SDRAM (safe before audio DMA starts!)
        memset(g_sdram_delay_l, 0, sizeof(g_sdram_delay_l));
        memset(g_sdram_delay_r, 0, sizeof(g_sdram_delay_r));
        _write_ptr = 0;
        _lofi_l.Init();
        _lofi_r.Init();
        _dc_block_l.Init(sample_rate);
        _dc_block_r.Init(sample_rate);

        _damp_l = 0.0f;
        _damp_r = 0.0f;
        _comb_tuner.Init(0.25f);
        Reset();
    }

    void ClearBuffers() {
        // Real-time zero-latency reset (NO 3.2MB memset to stall the FMC bus during audio!)
        _write_ptr = 0;
        _lofi_l.Reset();
        _lofi_r.Reset();
        _dc_block_l.Init(_sample_rate);
        _dc_block_r.Init(_sample_rate);
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
        _smooth_delay_samples = 12000.0f;
        _target_delay_samples = 12000.0f;
        _target_reverse = 0.0f;
        _smooth_reverse = 0.0f;
        _rev_phase_a = 0.0f;
        _stutter_timer = 0;
        _stutter_offset = 0.0f;
        _repeat_count = 0;
        _skip_click_l = 0.0f;
        _skip_click_r = 0.0f;
        _glitch_sag_target = 0.0f;
        _glitch_sag_smooth = 0.0f;

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
        _voltage_starve = 0.0f;
        _master_clock = 0.5f;
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
        _max_len_for_clock = 72000.0f * std::pow(0.016f, _glitch_density);
    }

    void SetReverse(float amount) {
        _target_reverse = daisysp::fclamp(amount, 0.0f, 1.0f);
    }

    float GetReverse() const { return _smooth_reverse; }

    void SetFreeze(bool freeze) {
        if (freeze && !_freeze) {
            _freeze_anchor = _write_ptr;
            _freeze_clock_ref = _master_clock;
            _freeze_clock_speed_ref = (_last_clock_speed > 0.05f) ? _last_clock_speed : 1.0f;
            _smooth_freeze_window = daisysp::fclamp(_smooth_delay_samples, 24.0f, static_cast<float>(kMaxDelaySamples - 2000));
            _target_freeze_window = _smooth_freeze_window;
            _freeze_phase = 0.0f;
            _smooth_freeze_speed = 1.0f;
            _target_freeze_speed = 1.0f;
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

    void TriggerBufferSkip(float strength = 1.0f) {
        // Buffer skip when starved (Vcc < 0.6)
        float lower_half_starve = daisysp::fclamp((_voltage_starve - 0.40f) * 2.0f, 0.0f, 1.0f);
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

    inline float ReadPos(const float* buf, float pos) const {
        if (__builtin_isnan(pos) || __builtin_isinf(pos) || pos < -2000000000.0f || pos > 2000000000.0f) {
            return 0.0f;
        }

        // Fast integer wrapping: single-cycle hardware modulo, NO slow libc floor/ceil or float divisions!
        int32_t i_pos = static_cast<int32_t>(pos);
        float frac = pos - static_cast<float>(i_pos);
        if (frac < 0.0f) {
            frac += 1.0f;
            i_pos -= 1;
        }

        i_pos %= static_cast<int32_t>(kMaxDelaySamples);
        if (i_pos < 0) {
            i_pos += static_cast<int32_t>(kMaxDelaySamples);
        }

        size_t idx0 = static_cast<size_t>(i_pos);
        size_t idx1 = idx0 + 1;
        if (idx1 >= kMaxDelaySamples) idx1 = 0;

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
        _write_ptr++;
        if (_write_ptr >= kMaxDelaySamples) _write_ptr = 0;
    }

    void PrepareBlock(size_t block_size) {
        // Master clock speed & target delay time
        float clock_bend = std::max(0.0f, _aux_in[0]);
        float target_clock_speed = daisysp::fclamp(0.45f + _master_clock * 1.10f - clock_bend * 0.40f, 0.20f, 2.2f);
        daisysp::fonepole(_prep_clock_speed, target_clock_speed, 0.08f);
        _last_clock_speed = _prep_clock_speed;
        float max_buf_sec = static_cast<float>(kMaxDelaySamples - 2000) / _sample_rate;
        float target_sec = daisysp::fclamp(_base_time_sec / _prep_clock_speed, 0.0008f, max_buf_sec);

        // Smooth reverse and diffusion targets
        daisysp::fonepole(_smooth_reverse, _target_reverse, 0.02f);
        daisysp::fonepole(_smooth_diffusion, _target_diffusion, 0.02f);

        // Comb tuner (evaluated ONCE per block!)
        _current_delay_sec = _comb_tuner.Process(target_sec);
        _current_delay_samples = _current_delay_sec * _sample_rate;
        _target_delay_samples = _current_delay_samples;

        // Analog tape / BBD drift & stereo chorus (evaluated ONCE per block!)
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

        // Chorus sweet spot window: active ONLY around Vcc 0.85 -> 0.55 (peaking at Vcc ~0.70)
        // Outside this window, chorus is 0 so it doesn't linger at 0V or interfere with normal delay
        float vcc = 1.0f - _voltage_starve;
        float chorus_drive = 0.0f;
        if (vcc < 0.88f && vcc > 0.52f) {
            float norm_pos = (vcc - 0.70f) / 0.18f; // -1.0 to +1.0 across the window
            float bell = 1.0f - norm_pos * norm_pos;
            chorus_drive = bell * bell; // Smooth quartic bell curve peaking at 1.0 when Vcc = 0.70
        }

        // Depth calibrated to vintage BBD chorus / tape wow sweet spot (3.5 - 6.0 samples)
        // Karplus-Strong mode (< 2400 samples) scales down to preserve comb tuning!
        if (chorus_drive > 0.005f) {
            float ks_scale = daisysp::fclamp((_current_delay_samples - 300.0f) / 2500.0f, 0.0f, 1.0f);
            float base_depth = (3.5f + daisysp::fclamp(_current_delay_samples * 0.00025f, 0.0f, 2.5f)) * ks_scale * chorus_drive;
            _prep_chorus_l = soft_l * base_depth;
            _prep_chorus_r = soft_r * base_depth;
        } else {
            _prep_chorus_l = 0.0f;
            _prep_chorus_r = 0.0f;
        }

        // Lo-Fi filter model prepare
        float effective_speed = _freeze ? _smooth_freeze_speed : _prep_clock_speed;
        float speed_slowdown = daisysp::fclamp((1.0f - effective_speed) * 1.42857f, 0.0f, 1.0f);
        float lo_fi_amount = daisysp::fclamp(_voltage_starve * 0.90f + speed_slowdown * 0.80f, 0.0f, 1.0f);
        _lofi_l.Prepare(_current_delay_sec, lo_fi_amount, _sample_rate);
        _lofi_r.Prepare(_current_delay_sec, lo_fi_amount, _sample_rate);

        // Tone damping alpha
        _smooth_cutoff += 0.05f * (_cutoff - _smooth_cutoff);
        _prep_damp_alpha = daisysp::fclamp(_smooth_cutoff * (6.2831853f / _sample_rate), 0.01f, 0.95f);

        // Freeze window & varispeed targets
        if (_freeze) {
            float norm_s33_sec = _base_time_sec / _freeze_clock_speed_ref;
            _target_freeze_window = daisysp::fclamp(norm_s33_sec * _sample_rate, 24.0f, static_cast<float>(kMaxDelaySamples - 2000));

            float clock_delta = _master_clock - _freeze_clock_ref;
            float fader_varispeed = FastPow2(clock_delta * 2.0f);
            float p06_brake = 1.0f - daisysp::fclamp(clock_bend * 0.60f, 0.0f, 0.85f);
            float tape_dir = 1.0f - 2.0f * _smooth_reverse;
            _target_freeze_speed = fader_varispeed * p06_brake * tape_dir;
        }

        // Diffusion parameters
        _prep_diffusion = _smooth_diffusion;
        _prep_diff_coef = 0.65f * _prep_diffusion;
    }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
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

        // Smooth continuous chorus modulation (per-sample slewing eliminates block-rate stepping)
        daisysp::fonepole(_smooth_chorus_l, _prep_chorus_l, 0.04f);
        daisysp::fonepole(_smooth_chorus_r, _prep_chorus_r, 0.04f);

        float chorus_offset_l = _smooth_chorus_l;
        float chorus_offset_r = _smooth_chorus_r;
        float mod_amount = _subtle_mod + _voltage_starve * 0.5f;

        if (mod_amount > 0.02f) {
            if (_current_delay_samples < 2400.0f) {
                // In Karplus-Strong range, apply subtle synchronized vibrato to avoid destructive comb cancellation
                float comb_vibrato = _sh_smooth_l * mod_amount * 0.4f;
                chorus_offset_l += comb_vibrato;
                chorus_offset_r += comb_vibrato;
                _tape_slip = 0.0f;
                _target_tape_slip = 0.0f;
            } else {
                float wow_depth = mod_amount * 5.0f;
                chorus_offset_l += _sh_smooth_l * wow_depth;
                chorus_offset_r += _sh_smooth_r * wow_depth;
            }
        }

        float lower_half_starve = daisysp::fclamp((_voltage_starve - 0.40f) * 2.0f, 0.0f, 1.0f);

        // Granular stutter slice (P09)
        if (_mangle > 0.04f) {
            float max_len_for_clock = _max_len_for_clock;

            // Rhythmic divisions (1/1 to 1/32)
            static constexpr float kSubdivRatios[] = {
                1.0f, 0.75f, 0.50f, 0.333333f, 0.25f, 0.166667f, 0.125f, 0.083333f, 0.0625f, 0.041667f, 0.03125f
            };
            static constexpr size_t kNumSubdivs = sizeof(kSubdivRatios) / sizeof(kSubdivRatios[0]);

            // Touch pressure morphs across divisions
            float subdiv_pos = daisysp::fclamp((_mangle - 0.04f) * 1.04167f * static_cast<float>(kNumSubdivs - 1), 0.0f, static_cast<float>(kNumSubdivs - 1));
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

        float delayed_l = 0.0f;
        float delayed_r = 0.0f;

        if (_freeze) {
            // Per-sample continuous smoothing for window size and playback speed (zero zipper on S33 & clock in freeze)
            daisysp::fonepole(_smooth_freeze_window, _target_freeze_window, 0.002f);
            daisysp::fonepole(_smooth_freeze_speed, _target_freeze_speed, 0.002f);

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

            // Read frozen loop from SDRAM (strictly 2 SDRAM reads per sample, no memory bus saturation!)
            float frozen_base_pos = static_cast<float>(_freeze_anchor) - window + _freeze_phase;
            float frozen_pos_l = frozen_base_pos + chorus_offset_l + _stutter_offset + _tape_slip;
            float frozen_pos_r = frozen_base_pos + chorus_offset_r + _stutter_offset + _tape_slip;

            delayed_l = ReadPos(g_sdram_delay_l, frozen_pos_l);
            delayed_r = ReadPos(g_sdram_delay_r, frozen_pos_r);

            // Smoothstep cubic de-click envelope at window seams (ZERO slow cosf calls!)
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
        } else {
            // Continuous per-sample delay tracking: completely eliminates 750 Hz block-edge stair-steps and zipper noise!
            // Fast tracking for Karplus-Strong (< 2400 samples), smooth tape inertia for echo delays
            float track_alpha = (_target_delay_samples < 2400.0f) ? 0.0035f : 0.00085f;
            float max_step = (_target_delay_samples < 2400.0f) ? 2.5f : 1.25f;
            float delta = _target_delay_samples - _smooth_delay_samples;
            _smooth_delay_samples += daisysp::fclamp(delta * track_alpha, -max_step, max_step);

            // Normal delay read
            float normal_base_read = _smooth_delay_samples + _stutter_offset + _tape_slip;
            float normal_read_l = daisysp::fclamp(normal_base_read + chorus_offset_l, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
            float normal_read_r = daisysp::fclamp(normal_base_read + chorus_offset_r, 1.0f, static_cast<float>(kMaxDelaySamples - 4));
            ReadDelay(normal_read_l, normal_read_r, delayed_l, delayed_r);

            // Dual-head crossfaded reverse delay in normal mode
            if (_smooth_reverse > 0.001f) {
                float rev_chunk_len = daisysp::fclamp(_smooth_delay_samples, 240.0f, static_cast<float>(kMaxDelaySamples - 4000) * 0.5f);
                _rev_phase_a += 1.0f;
                if (_rev_phase_a >= rev_chunk_len) {
                    _rev_phase_a -= rev_chunk_len;
                }
                float rev_phase_b = _rev_phase_a + rev_chunk_len * 0.5f;
                if (rev_phase_b >= rev_chunk_len) {
                    rev_phase_b -= rev_chunk_len;
                }

                // In rev_chunk_len output samples, head sweeps back at rate 1.0 (offset increases at rate 2.0)
                float offset_a = 64.0f + 2.0f * _rev_phase_a;
                float offset_b = 64.0f + 2.0f * rev_phase_b;

                // Smoothstep cubic crossfade (ZERO slow sinf calls!)
                float norm_a = _rev_phase_a / rev_chunk_len;
                float gain_a = norm_a * norm_a * (3.0f - 2.0f * norm_a);
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

        // Address bit-flip glitch (under starve + P09 activity)
        if (_glitch > 0.15f && _voltage_starve > 0.35f) {
            _rng = _rng * 1664525u + 1013904223u;
            float bit_flip_prob = _glitch * _voltage_starve * 0.00002f;
            if (static_cast<float>(_rng & 0x00FFFFFF) * 5.9604645e-8f < bit_flip_prob) {
                uint32_t flip_bit = 4u + (_rng & 0x03);
                uint32_t flip_mask = 1u << flip_bit;
                if (_voltage_starve > 0.6f) {
                    flip_mask |= 1u << (5u + ((_rng >> 4) & 0x03));
                }

                size_t pos_l = static_cast<size_t>(std::abs(_current_delay_samples)) ^ flip_mask;
                size_t corrupted_pos_l = pos_l % kMaxDelaySamples;
                float corruption = daisysp::fclamp(_glitch * _voltage_starve * 0.5f, 0.0f, 0.4f);
                delayed_l = delayed_l * (1.0f - corruption) + g_sdram_delay_l[corrupted_pos_l] * corruption;

                size_t pos_r = static_cast<size_t>(std::abs(_current_delay_samples)) ^ flip_mask;
                size_t corrupted_pos_r = pos_r % kMaxDelaySamples;
                delayed_r = delayed_r * (1.0f - corruption) + g_sdram_delay_r[corrupted_pos_r] * corruption;
            }
        }

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
                float r = static_cast<float>(_rng & 0x00FFFFFF) * 5.9604645e-8f;
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

        // Lo-fi filter: lightweight per-sample processing
        delayed_l = _lofi_l.Process(delayed_l);
        delayed_r = _lofi_r.Process(delayed_r);

        // Tone damping filter (Cutoff knob S31) using block-precomputed alpha
        _damp_l += _prep_damp_alpha * (delayed_l - _damp_l);
        _damp_r += _prep_damp_alpha * (delayed_r - _damp_r);

        // DC blocker on delay feedback path
        float ac_l = _dc_block_l.Process(_damp_l);
        float ac_r = _dc_block_r.Process(_damp_r);

        // Clean soft clip that preserves loop gain for Karplus-Strong & singing delays
        float sat_l = daisysp::SoftClip(ac_l / (1.0f + 0.15f * std::abs(ac_l)));
        float sat_r = daisysp::SoftClip(ac_r / (1.0f + 0.15f * std::abs(ac_r)));

        // Mode detection: 1.0 = Karplus-Strong comb (< 35ms), 0.0 = Echo delay (> 60ms)
        float ks_mode = daisysp::fclamp((0.055f - _current_delay_sec) * 40.0f, 0.0f, 1.0f);

        // Feedback limiter: only for long echo delays to stop runaway buildup; Karplus-Strong sings unattenuated
        float fb_mag = 0.5f * (std::abs(sat_l) + std::abs(sat_r));
        float echo_anti_blowout = 1.0f / (1.0f + std::max(0.0f, fb_mag - 0.85f) * 1.5f);
        float eff_anti_blowout = 1.0f * ks_mode + echo_anti_blowout * (1.0f - ks_mode);

        float raw_fb = _freeze ? 0.999f : daisysp::fclamp(_feedback + _aux_in[1], 0.0f, 1.20f);
        float eff_feedback = raw_fb * eff_anti_blowout;

        // Feedback source
        float fb_src_l = _freeze ? delayed_l : sat_l;
        float fb_src_r = _freeze ? delayed_r : sat_r;

        // True Schroeder/Dattorro all-pass diffusion (strictly flat magnitude response, zero comb notching!)
        // Smooth continuous crossfade eliminates any audible 10ms step jump when engaging Pad 03!
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

        // Ping-pong cross-coupling: only on echo delays! Dual mono on Karplus-Strong to avoid comb phase cancellation!
        float cross_ratio = 0.35f * (1.0f - ks_mode);
        _fb_l = (fb_src_l * (1.0f - cross_ratio) + fb_src_r * cross_ratio) * eff_feedback;
        _fb_r = (fb_src_r * (1.0f - cross_ratio) + fb_src_l * cross_ratio) * eff_feedback;

        // Overdub write to delay line (halted when frozen so loop is preserved)
        if (!_freeze) {
            float write_l = in_l + _fb_l;
            float write_r = in_r + _fb_r;
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
            case 0: _base_time_sec  = daisysp::fclamp(value, 0.0008f, 8.20f); break; // Up to 8.0s cassette tape loop
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
    daisysp::DcBlock _dc_block_l;
    daisysp::DcBlock _dc_block_r;
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
    size_t _freeze_anchor{0};
    float  _smooth_delay_samples{12000.0f};
    float  _target_delay_samples{12000.0f};
    float  _target_reverse{0.0f};
    float  _smooth_reverse{0.0f};
    float  _rev_phase_a{0.0f};
    float  _subtle_mod{0.0f};

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
    float    _max_len_for_clock{72000.0f};
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

    // 2-stage stereo allpass diffusers
    DiffuserStage<149> _diff_l1;
    DiffuserStage<331> _diff_l2;
    DiffuserStage<197> _diff_r1;
    DiffuserStage<397> _diff_r2;
    float _target_diffusion{0.0f};
    float _smooth_diffusion{0.0f};

    // Vintage BBD / tape chorus drift
    float _chorus_phase_l{0.0f};
    float _chorus_phase_r{1.5707963f}; // 90 deg initial phase offset
    float _smooth_chorus_l{0.0f};
    float _smooth_chorus_r{0.0f};

    // Block-rate prepared parameters (avoids expensive per-sample calculations)
    float _prep_clock_speed{1.0f};
    float _prep_chorus_l{0.0f};
    float _prep_chorus_r{0.0f};
    float _prep_damp_alpha{0.1f};
    float _prep_diffusion{0.0f};
    float _prep_diff_coef{0.0f};
};

} // namespace bends
