#pragma once

#include "dsp/fast_math.h"
#include "dsp/filters.h"
#include "dsp/buffer.h"
#include "parts/lofi.h"
#include "bends/mod_pack.h"
#include <cmath>
#include <algorithm>

namespace bends {

class Delay {
public:
    static constexpr size_t kMaxDelay = 768000; // 16.0s at 48kHz stereo (6.14 MB in SDRAM)

    void Init(float sample_rate, float* buf_l, float* buf_r, size_t max_delay = kMaxDelay) {
        _sample_rate = sample_rate;
        _buffer.Init(buf_l, buf_r, max_delay);
        _lofi_l.Init();
        _lofi_r.Init();
        _dc_block_l.Reset();
        _dc_block_r.Reset();
        Reset();
    }

    void Reset() {
        _buffer.Clear();
        _lofi_l.Reset();
        _lofi_r.Reset();
        _dc_block_l.Reset();
        _dc_block_r.Reset();

        _smooth_delay_samples = 28.0f;
        _knob_dly_held = 0.50f;
        _delay_time_curr = 28.0f;
        _delay_time_prev = 28.0f;
        _delay_xfade = 1.0f;
        _delay_time_initialized = false;

        _damp_l = 0.0f;
        _damp_r = 0.0f;

        _rev_phase_l = 0.0f;
        _stutter_pos_l = 0.0f;
        _stutter_pos_r = 0.0f;

        _slip_timer = 0;
        _slip_len = 2400;
        _slip_offset_l = 0.0f;
        _slip_offset_r = 0.0f;
        _lfsr = 0xACE1u;

        _was_frozen = false;
        _freeze_len = 48000.0f;
        _freeze_phase = 0.0f;
        _target_samples = 28.0f;
        _clk_fader_speed = 1.0f;
    }

    inline void ServiceIncrementalClear(size_t chunk_size = 4096) {
        _buffer.ServiceIncrementalClear(chunk_size);
    }

    inline void PrepareBlock(float knob_dly, float knob_clk, float node_pull_dly, float node_pull_clk) {
        float u_raw = std::clamp(knob_dly + node_pull_dly * 0.20f, 0.0f, 1.0f);
        if (!_delay_time_initialized || std::abs(u_raw - _knob_dly_held) > 0.005f) {
            _knob_dly_held = u_raw;
        }
        float u = _knob_dly_held;

        // Base delay time
        float base_time;
        if (u < 0.20f) {
            // Zone 1: Sub-millisecond Karplus-Strong string tuning (0.6ms to 25ms)
            float k = u / 0.20f;
            base_time = 0.0006f * FastPow2(k * 5.3808218f);
        } else {
            // Zone 2: Echo, Space, Looper (25ms to 8.0s)
            float w = (u - 0.20f) / 0.80f;
            base_time = 0.025f * FastPow2(w * 8.321928f);
        }
        base_time = std::clamp(base_time, 0.0006f, 8.0f);

        _target_samples = std::clamp(base_time * _sample_rate, 28.0f, static_cast<float>(_buffer.MaxSize() - 2));

        // Variclock speed base
        float clk_dc = std::clamp(knob_clk + node_pull_clk * 0.20f, 0.0f, 1.0f);
        _clk_fader_speed = 0.20f + 0.80f * (clk_dc * clk_dc);
    }

    inline void Process(float in_l, float in_r,
                        const DelayMods& mods,
                        bool is_frozen, bool clk_is_slow, float clk_phase,
                        float string_damp, float eff_fb,
                        float v_rail_eff, float starve, float shunt_dly, float shunt_fbk,
                        float& out_l, float& out_r) {
        float clk_rail_sag = 0.50f + 0.50f * (v_rail_eff * v_rail_eff);
        float eff_clk_speed = (_clk_fader_speed * (1.0f + mods.overclock) - mods.decimate + mods.clock_fm) * clk_rail_sag;
        eff_clk_speed = std::clamp(eff_clk_speed, 0.0f, 1.45f);

        // Delay Time knob smoothing & raised-cosine crossfading
        if (!_delay_time_initialized) {
            _smooth_delay_samples = _target_samples;
            _delay_time_curr = _target_samples;
            _delay_time_prev = _target_samples;
            _delay_xfade = 1.0f;
            _delay_time_initialized = true;
        } else if (_knob_dly_held < 0.20f) {
            _smooth_delay_samples += 0.025f * (_target_samples - _smooth_delay_samples);
            _delay_time_curr = _smooth_delay_samples;
            _delay_time_prev = _smooth_delay_samples;
            _delay_xfade = 1.0f;
        } else {
            if (_delay_xfade >= 1.0f) {
                if (std::abs(_target_samples - _delay_time_curr) > 36.0f) {
                    _delay_time_prev = _delay_time_curr;
                    _delay_time_curr = _target_samples;
                    _delay_xfade = 0.0f;
                }
            } else {
                _delay_xfade += 0.00085f; // ~25ms crossfade
                if (_delay_xfade >= 1.0f) {
                    _delay_xfade = 1.0f;
                    if (std::abs(_target_samples - _delay_time_curr) > 36.0f) {
                        _delay_time_prev = _delay_time_curr;
                        _delay_time_curr = _target_samples;
                        _delay_xfade = 0.0f;
                    }
                }
            }
            _smooth_delay_samples = _delay_time_curr;
        }

        // Freeze handling
        if (is_frozen) {
            if (!_was_frozen) {
                _was_frozen = true;
                _freeze_len = std::clamp(_smooth_delay_samples, 28.0f, static_cast<float>(_buffer.MaxSize() - 2));
                _freeze_phase = 0.0f;
            }
            float target_freeze_len = std::clamp(_smooth_delay_samples, 28.0f, static_cast<float>(_buffer.MaxSize() - 2));
            _freeze_len += 0.008f * (target_freeze_len - _freeze_len);

            if (_freeze_len > 1.0f) {
                _freeze_phase += eff_clk_speed;
                if (_freeze_phase >= _freeze_len) {
                    _freeze_phase = fmodf(_freeze_phase, _freeze_len);
                } else if (_freeze_phase < 0.0f) {
                    _freeze_phase = fmodf(_freeze_phase, _freeze_len) + _freeze_len;
                }
            }
        } else {
            _was_frozen = false;
        }

        float base_read_samples = is_frozen ? (_freeze_len - _freeze_phase) : _smooth_delay_samples;
        float read_pos_l = base_read_samples + mods.time_mod;
        float read_pos_r = base_read_samples - mods.time_mod;

        // Rhythmic glitch stutter (< 35 Hz)
        if (mods.stutter > 0.015f && clk_is_slow) {
            float stutter_len = std::clamp(base_read_samples * 0.15f, 300.0f, 6000.0f);
            bool stutter_l = (clk_phase < 0.28f);
            bool stutter_r = (clk_phase >= 0.50f && clk_phase < 0.78f);

            if (stutter_l) {
                _stutter_pos_l += 1.0f;
                if (_stutter_pos_l >= stutter_len) _stutter_pos_l = 0.0f;
                read_pos_l = std::max(1.0f, read_pos_l - _stutter_pos_l * mods.stutter);
            } else {
                _stutter_pos_l = 0.0f;
            }

            if (stutter_r) {
                _stutter_pos_r += 1.0f;
                if (_stutter_pos_r >= stutter_len) _stutter_pos_r = 0.0f;
                read_pos_r = std::max(1.0f, read_pos_r - _stutter_pos_r * mods.stutter);
            } else {
                _stutter_pos_r = 0.0f;
            }
        }

        // Read delay buffer with raised-cosine crossfade
        float mod_l = read_pos_l - base_read_samples;
        float mod_r = read_pos_r - base_read_samples;

        float raw_l, raw_r;
        if (!is_frozen && _knob_dly_held >= 0.20f && _delay_xfade < 1.0f) {
            float fade_win = 0.5f * (1.0f - FastCos(_delay_xfade * kPi));
            float prev_l = std::clamp(_delay_time_prev + mod_l, 1.0f, static_cast<float>(_buffer.MaxSize() - 2));
            float curr_l = std::clamp(_delay_time_curr + mod_l, 1.0f, static_cast<float>(_buffer.MaxSize() - 2));
            raw_l = _buffer.ReadL(prev_l) * (1.0f - fade_win) + _buffer.ReadL(curr_l) * fade_win;

            float prev_r = std::clamp(_delay_time_prev + mod_r, 1.0f, static_cast<float>(_buffer.MaxSize() - 2));
            float curr_r = std::clamp(_delay_time_curr + mod_r, 1.0f, static_cast<float>(_buffer.MaxSize() - 2));
            raw_r = _buffer.ReadR(prev_r) * (1.0f - fade_win) + _buffer.ReadR(curr_r) * fade_win;
        } else {
            raw_l = _buffer.ReadL(read_pos_l);
            raw_r = _buffer.ReadR(read_pos_r);
        }

        // Address bus slip & CD skips
        float starve_slip = (v_rail_eff < 0.33f) ? ((0.33f - v_rail_eff) / 0.31f) : 0.0f;
        float glitch_intensity = std::max(mods.address_slip, starve_slip);

        if (glitch_intensity > 0.015f) {
            _slip_timer++;
            if (_slip_timer >= _slip_len) {
                _slip_timer = 0;
                uint32_t base_len = 1200 + (_lfsr % 4560);
                _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
                _slip_len = static_cast<uint16_t>(base_len / (1.0f + glitch_intensity * 1.5f));
                if (_slip_len < 240) _slip_len = 240;

                static const float kAddressJumps[6] = {512.0f, 1024.0f, 2048.0f, 4096.0f, 8192.0f, 16384.0f};
                uint8_t jump_idx = (_lfsr >> 3) % 6;
                _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
                _slip_offset_l = kAddressJumps[jump_idx];
                _slip_offset_r = kAddressJumps[(jump_idx + 1) % 6];
            }

            float grain_win = 1.0f;
            constexpr uint16_t kFadeLen = 120;
            if (_slip_timer < kFadeLen) {
                grain_win = static_cast<float>(_slip_timer) / static_cast<float>(kFadeLen);
            } else if (_slip_len - _slip_timer < kFadeLen) {
                grain_win = static_cast<float>(_slip_len - _slip_timer) / static_cast<float>(kFadeLen);
            }
            grain_win = 0.5f * (1.0f - FastCos(grain_win * kPi));

            float slip_read_l = std::clamp(read_pos_l - _slip_timer + _slip_offset_l, 1.0f, static_cast<float>(_buffer.MaxSize() - 2));
            float slip_read_r = std::clamp(read_pos_r - _slip_timer + _slip_offset_r, 1.0f, static_cast<float>(_buffer.MaxSize() - 2));

            float slipped_l = _buffer.ReadL(slip_read_l);
            float slipped_r = _buffer.ReadR(slip_read_r);

            float g_blend = glitch_intensity * grain_win;
            raw_l = raw_l * (1.0f - g_blend) + slipped_l * g_blend;
            raw_r = raw_r * (1.0f - g_blend) + slipped_r * g_blend;
        }

        // Bitwise XOR memory address scrambling bend
        if (mods.xor_glitch > 0.015f) {
            uint32_t mask = 0;
            if (mods.xor_glitch > 0.05f) mask |= 0x003F; // micro byte fragments
            if (mods.xor_glitch > 0.25f) mask |= 0x03C0; // mid address line scramble
            if (mods.xor_glitch > 0.55f) mask |= 0x1E00; // macro address bus short
            if (mods.xor_glitch > 0.85f) mask |= 0x6000; // wild ring bank short

            uint32_t offset_l = static_cast<uint32_t>(read_pos_l) ^ mask;
            uint32_t offset_r = static_cast<uint32_t>(read_pos_r) ^ (mask << 1);
            float xor_read_l = std::clamp(static_cast<float>(offset_l), 1.0f, static_cast<float>(_buffer.MaxSize() - 2));
            float xor_read_r = std::clamp(static_cast<float>(offset_r), 1.0f, static_cast<float>(_buffer.MaxSize() - 2));

            float xor_l = _buffer.ReadL(xor_read_l);
            float xor_r = _buffer.ReadR(xor_read_r);

            float blend = std::min(1.0f, mods.xor_glitch * 1.4f);
            raw_l = raw_l * (1.0f - blend) + xor_l * blend;
            raw_r = raw_r * (1.0f - blend) + xor_r * blend;
        }

        // 90° Quadrature Reverse Pitch Cloud
        if (mods.reverse > 0.015f) {
            float rev_window = std::clamp(base_read_samples * 0.85f, 1200.0f, 48000.0f);
            float inv_window = 1.0f / rev_window;

            _rev_phase_l += inv_window;
            if (_rev_phase_l >= 1.0f) _rev_phase_l -= 1.0f;

            float p1_l = _rev_phase_l;
            float p2_l = p1_l + 0.5f;
            if (p2_l >= 1.0f) p2_l -= 1.0f;

            float d1_l = read_pos_l + (1.0f - p1_l) * rev_window;
            float d2_l = read_pos_l + (1.0f - p2_l) * rev_window;
            float s1_l = _buffer.ReadL(d1_l);
            float s2_l = _buffer.ReadL(d2_l);
            float w1_l = 4.0f * p1_l * (1.0f - p1_l);
            float w2_l = 4.0f * p2_l * (1.0f - p2_l);
            float rev_read_l = (s1_l * w1_l + s2_l * w2_l) / (w1_l + w2_l + 1e-6f);

            float p1_r = _rev_phase_l + 0.25f;
            if (p1_r >= 1.0f) p1_r -= 1.0f;
            float p2_r = p1_r + 0.5f;
            if (p2_r >= 1.0f) p2_r -= 1.0f;

            float d1_r = read_pos_r + (1.0f - p1_r) * rev_window;
            float d2_r = read_pos_r + (1.0f - p2_r) * rev_window;
            float s1_r = _buffer.ReadR(d1_r);
            float s2_r = _buffer.ReadR(d2_r);
            float w1_r = 4.0f * p1_r * (1.0f - p1_r);
            float w2_r = 4.0f * p2_r * (1.0f - p2_r);
            float rev_read_r = (s1_r * w1_r + s2_r * w2_r) / (w1_r + w2_r + 1e-6f);

            raw_l += mods.reverse * (rev_read_l - raw_l);
            raw_r += mods.reverse * (rev_read_r - raw_r);
        }

        // Loop damping filter
        _damp_l += string_damp * (raw_l - _damp_l);
        _damp_r += string_damp * (raw_r - _damp_r);

        // Lo-Fi downsampler & DAC quantization
        float out_dac_l, out_dac_r;
        if (eff_clk_speed >= 0.98f && v_rail_eff >= 0.95f && mods.decimate < 0.01f) {
            out_dac_l = _damp_l * shunt_dly;
            out_dac_r = _damp_r * shunt_dly;
        } else {
            _lofi_l.Prepare(eff_clk_speed, _sample_rate);
            _lofi_r.Prepare(eff_clk_speed, _sample_rate);
            float proc_l = _lofi_l.Process(_damp_l);
            float proc_r = _lofi_r.Process(_damp_r);

            float bit_depth = std::clamp(16.0f - starve * 8.0f - (1.0f - eff_clk_speed) * 6.0f, 8.0f, 16.0f);
            float levels = FastPow2(bit_depth);
            float inv_levels = 1.0f / levels;
            float dac_dnl = 0.05f + starve * 0.35f + (1.0f - eff_clk_speed) * 0.15f;

            out_dac_l = QuantizeDac(proc_l, levels, inv_levels, dac_dnl) * shunt_dly;
            out_dac_r = QuantizeDac(proc_r, levels, inv_levels, dac_dnl) * shunt_dly;
        }

        // Feedback calculation & Ping-Pong cross-coupling
        float ping_pong_cross = 0.32f + mods.ping_pong;
        float cross = std::clamp(ping_pong_cross, 0.0f, 1.0f);
        float direct = 1.0f - cross;
        float fb_tot_l = (_damp_l * direct + _damp_r * cross) * eff_fb * shunt_fbk;
        float fb_tot_r = (_damp_r * direct + _damp_l * cross) * eff_fb * shunt_fbk;

        // SDRAM buffer write (inhibited during freeze or deep glitch)
        bool write_inhibited = is_frozen || (glitch_intensity > 0.15f) || (mods.xor_glitch > 0.60f);
        if (!write_inhibited) {
            float dly_in_l = _dc_block_l.Process(in_l + FastTanh(fb_tot_l));
            float dly_in_r = _dc_block_r.Process(in_r + FastTanh(fb_tot_r));
            _buffer.Write(dly_in_l, dly_in_r, eff_clk_speed);
        } else if (!is_frozen) {
            _buffer.Advance(eff_clk_speed);
        }

        out_l = out_dac_l;
        out_r = out_dac_r;
    }

private:
    float _sample_rate{48000.0f};
    StereoDelayBuffer _buffer;

    Lofi _lofi_l;
    Lofi _lofi_r;
    DcBlocker _dc_block_l;
    DcBlocker _dc_block_r;

    float _smooth_delay_samples{28.0f};
    float _knob_dly_held{0.50f};
    float _delay_time_curr{28.0f};
    float _delay_time_prev{28.0f};
    float _delay_xfade{1.0f};
    bool  _delay_time_initialized{false};
    float _target_samples{28.0f};
    float _clk_fader_speed{1.0f};

    float _damp_l{0.0f};
    float _damp_r{0.0f};

    float _rev_phase_l{0.0f};
    float _stutter_pos_l{0.0f};
    float _stutter_pos_r{0.0f};

    uint16_t _slip_timer{0};
    uint16_t _slip_len{2400};
    float    _slip_offset_l{0.0f};
    float    _slip_offset_r{0.0f};
    uint16_t _lfsr{0xACE1u};

    bool  _was_frozen{false};
    float _freeze_len{48000.0f};
    float _freeze_phase{0.0f};
};

} // namespace bends
