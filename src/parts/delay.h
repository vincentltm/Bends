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
        _head_a_samples = 28.0f;
        _head_b_samples = 28.0f;
        _active_head = 0;
        _xfade_active = false;
        _xfade_progress = 0.0f;
        _xfade_dir = 1;
        _knob_dly_held = 0.50f;
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
        _clk_fm_slow = 0.0f;
        _tape_braking = false;
        _tape_brake_speed = 1.0f;
        _tape_brake_offset = 28.0f;
    }

    inline void ServiceIncrementalClear(size_t chunk_size = 4096) {
        _buffer.ServiceIncrementalClear(chunk_size);
    }

    inline void PrepareBlock(float knob_dly, float knob_clk, float node_pull_dly, float node_pull_clk) {
        float u_raw = std::clamp(knob_dly + node_pull_dly * 0.20f, 0.0f, 1.0f);
        if (!_delay_time_initialized) {
            _knob_dly_held = u_raw;
        } else {
            // Smooth potentiometer reads continuously: zero staircase steps
            _knob_dly_held += 0.08f * (u_raw - _knob_dly_held);
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
        float eff_clk_speed = _clk_fader_speed * (1.0f + mods.overclock) - mods.decimate;
        eff_clk_speed = std::clamp(eff_clk_speed, 0.05f, 1.45f);

        if (!_delay_time_initialized) {
            _head_a_samples = _target_samples;
            _head_b_samples = _target_samples;
            _smooth_delay_samples = _target_samples;
            _active_head = 0;
            _xfade_active = false;
            _xfade_progress = 0.0f;
            _delay_time_initialized = true;
        } else if (_target_samples < 2400.0f) {
            // Zone 1: Sub-millisecond Karplus-Strong string tuning (0.6ms to 50ms)
            // Direct smooth pitch glide for resonant physical modeling tuning
            _head_a_samples += 0.04f * (_target_samples - _head_a_samples);
            _head_b_samples = _head_a_samples;
            _smooth_delay_samples = _head_a_samples;
            _active_head = 0;
            _xfade_active = false;
            _xfade_progress = 0.0f;
        } else {
            // Zone 2: Digital Echo / Delay Buffer Length setting
            // ZERO tape pitch-warping: read heads play strictly at 1.0x normal speed.
            // Equal-power crossfader between Head A and Head B: head relocation happens ONLY when head volume is exactly 0.0!
            constexpr float kMoveThreshold = 96.0f; // ~2ms threshold
            constexpr float kStep = 1.0f / static_cast<float>(kXfadeSamples);

            if (_xfade_active) {
                _xfade_progress += static_cast<float>(_xfade_dir) * kStep;
                if (_xfade_dir > 0 && _xfade_progress >= 1.0f) {
                    _xfade_progress = 1.0f;
                    _xfade_active = false;
                    _active_head = 1;
                    // Head A is now silent (gain = 0.0): check if target moved further
                    if (std::abs(_target_samples - _head_b_samples) > kMoveThreshold) {
                        _head_a_samples = _target_samples;
                        _xfade_active = true;
                        _xfade_dir = -1;
                    }
                } else if (_xfade_dir < 0 && _xfade_progress <= 0.0f) {
                    _xfade_progress = 0.0f;
                    _xfade_active = false;
                    _active_head = 0;
                    // Head B is now silent (gain = 0.0): check if target moved further
                    if (std::abs(_target_samples - _head_a_samples) > kMoveThreshold) {
                        _head_b_samples = _target_samples;
                        _xfade_active = true;
                        _xfade_dir = +1;
                    }
                }
            } else {
                if (_active_head == 0) {
                    if (std::abs(_target_samples - _head_a_samples) > kMoveThreshold) {
                        _head_b_samples = _target_samples;
                        _xfade_active = true;
                        _xfade_dir = +1;
                    }
                } else {
                    if (std::abs(_target_samples - _head_b_samples) > kMoveThreshold) {
                        _head_a_samples = _target_samples;
                        _xfade_active = true;
                        _xfade_dir = -1;
                    }
                }
            }
            _smooth_delay_samples = _head_a_samples * (1.0f - _xfade_progress) + _head_b_samples * _xfade_progress;
        }

        // Freeze & Pressure-Sensitive Beat-Repeat Micro-Looper handling
        bool eff_frozen = is_frozen || (mods.beat_repeat > 0.05f);
        if (eff_frozen) {
            float target_freeze_len = _smooth_delay_samples;
            if (mods.beat_repeat > 0.05f) {
                if (mods.beat_repeat > 0.80f) {
                    target_freeze_len = 960.0f;  // ~20ms machine-gun roll
                } else if (mods.beat_repeat > 0.50f) {
                    target_freeze_len = 2400.0f; // ~50ms 1/16th roll
                } else {
                    target_freeze_len = 5760.0f; // ~120ms 1/8th beat roll
                }
            }
            if (!_was_frozen) {
                _was_frozen = true;
                _freeze_len = std::clamp(target_freeze_len, 28.0f, static_cast<float>(_buffer.MaxSize() - 2));
                _freeze_phase = 0.0f;
            }
            target_freeze_len = std::clamp(target_freeze_len, 28.0f, static_cast<float>(_buffer.MaxSize() - 2));
            _freeze_len += 0.025f * (target_freeze_len - _freeze_len);

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

        float base_read_samples = eff_frozen ? (_freeze_len - _freeze_phase) : _smooth_delay_samples;

        // Tape Motor Brake (Bend: P06 DLY <-> P05 GND)
        if (mods.tape_stop > 0.05f) {
            if (!_tape_braking) {
                _tape_braking = true;
                _tape_brake_speed = 1.0f;
                _tape_brake_offset = base_read_samples;
            }
            float decel = 0.00015f + mods.tape_stop * 0.00030f;
            _tape_brake_speed = std::max(0.0f, _tape_brake_speed - decel);
            _tape_brake_offset += (1.0f - _tape_brake_speed);
            _tape_brake_offset = std::clamp(_tape_brake_offset, 28.0f, static_cast<float>(_buffer.MaxSize() - 100));
            base_read_samples = _tape_brake_offset;
        } else if (_tape_braking) {
            _tape_brake_speed += 0.0005f * (1.0f - _tape_brake_speed);
            _tape_brake_offset += (1.0f - _tape_brake_speed);
            _tape_brake_offset = std::clamp(_tape_brake_offset, 28.0f, static_cast<float>(_buffer.MaxSize() - 100));
            base_read_samples = _tape_brake_offset;
            if (_tape_brake_speed >= 0.99f) {
                _tape_braking = false;
            }
        }

        // Dynamic Length & Quality scaled modulation from OSC <-> CLK bend
        // - Short length (< 2400 samples, Karplus-Strong string):
        //   Slow LFO rates glide string fundamental pitch (singing melodies, plucks, stepped S&H arpeggios).
        //   Audio rates produce warm acoustic FM bell/chime overtones without harsh metallic clatter.
        // - Long length (>= 2400 samples, Echo & Space):
        //   Slow LFO rates sweep delay heads in quadrature (+ / -) for lush 3D stereo chorus and flanger sweeps.
        //   Audio rates produce warm analog tape flutter and harmonic sheen.
        float clk_fm = mods.clock_fm;
        float clk_fm_time_mod_l = 0.0f;
        float clk_fm_time_mod_r = 0.0f;
        if (std::abs(clk_fm) > 0.001f) {
            _clk_fm_slow += 0.0018f * (clk_fm - _clk_fm_slow);
            float clk_fm_fast = clk_fm - _clk_fm_slow;

            if (base_read_samples < 2400.0f) {
                // Karplus-Strong string tuning range: proportional pitch modulation
                // Musical pitch swing (+/- 2.5 semitones max) + acoustic FM bell overtone
                float slow_depth = std::min(45.0f, base_read_samples * 0.15f);
                clk_fm_time_mod_l = _clk_fm_slow * slow_depth + clk_fm_fast * 1.8f;
                clk_fm_time_mod_r = _clk_fm_slow * (slow_depth * 0.88f) + clk_fm_fast * 1.5f;
            } else {
                // Echo delay range: lush quadrature stereo chorus & flanger sweeps
                float slow_depth = 20.0f + std::sqrt(base_read_samples) * 0.35f;
                clk_fm_time_mod_l = _clk_fm_slow * slow_depth + clk_fm_fast * 1.8f;
                clk_fm_time_mod_r = -_clk_fm_slow * slow_depth - clk_fm_fast * 1.5f;
            }
        } else {
            _clk_fm_slow *= 0.998f;
        }

        // Tier 2: Subtle memory address flutter under starved power rail (0.70 -> 0.45)
        float starve_flutter = (starve > 0.30f) ? (FastSin(static_cast<float>(_lfsr) * 0.005f) * (starve - 0.30f) * 4.0f) : 0.0f;

        float mod_l = mods.time_mod + clk_fm_time_mod_l + starve_flutter;
        float mod_r = -mods.time_mod + clk_fm_time_mod_r - starve_flutter;

        // Rhythmic glitch stutter (< 35 Hz)
        if (mods.stutter > 0.015f && clk_is_slow) {
            float stutter_len = std::clamp(base_read_samples * 0.15f, 300.0f, 6000.0f);
            bool stutter_l = (clk_phase < 0.28f);
            bool stutter_r = (clk_phase >= 0.50f && clk_phase < 0.78f);

            if (stutter_l) {
                _stutter_pos_l += 1.0f;
                if (_stutter_pos_l >= stutter_len) _stutter_pos_l = 0.0f;
                mod_l -= _stutter_pos_l * mods.stutter;
            } else {
                _stutter_pos_l = 0.0f;
            }

            if (stutter_r) {
                _stutter_pos_r += 1.0f;
                if (_stutter_pos_r >= stutter_len) _stutter_pos_r = 0.0f;
                mod_r -= _stutter_pos_r * mods.stutter;
            } else {
                _stutter_pos_r = 0.0f;
            }
        }

        // Read delay buffer (Equal-Power Dual-Head or Direct)
        float raw_l, raw_r;
        if (eff_frozen) {
            float pos_l = std::clamp(base_read_samples + mod_l, 8.0f, static_cast<float>(_buffer.MaxSize() - 2));
            float pos_r = std::clamp(base_read_samples + mod_r, 8.0f, static_cast<float>(_buffer.MaxSize() - 2));
            raw_l = _buffer.ReadL(pos_l);
            raw_r = _buffer.ReadR(pos_r);
        } else if (_xfade_active) {
            float angle = 0.5f * kPi * _xfade_progress;
            float gain_a = FastCos(angle);
            float gain_b = FastSin(angle);

            float pos_a_l = std::clamp(_head_a_samples + mod_l, 8.0f, static_cast<float>(_buffer.MaxSize() - 2));
            float pos_a_r = std::clamp(_head_a_samples + mod_r, 8.0f, static_cast<float>(_buffer.MaxSize() - 2));
            float pos_b_l = std::clamp(_head_b_samples + mod_l, 8.0f, static_cast<float>(_buffer.MaxSize() - 2));
            float pos_b_r = std::clamp(_head_b_samples + mod_r, 8.0f, static_cast<float>(_buffer.MaxSize() - 2));

            raw_l = _buffer.ReadL(pos_a_l) * gain_a + _buffer.ReadL(pos_b_l) * gain_b;
            raw_r = _buffer.ReadR(pos_a_r) * gain_a + _buffer.ReadR(pos_b_r) * gain_b;
        } else if (_active_head == 0) {
            float pos_l = std::clamp(_head_a_samples + mod_l, 8.0f, static_cast<float>(_buffer.MaxSize() - 2));
            float pos_r = std::clamp(_head_a_samples + mod_r, 8.0f, static_cast<float>(_buffer.MaxSize() - 2));
            raw_l = _buffer.ReadL(pos_l);
            raw_r = _buffer.ReadR(pos_r);
        } else {
            float pos_l = std::clamp(_head_b_samples + mod_l, 8.0f, static_cast<float>(_buffer.MaxSize() - 2));
            float pos_r = std::clamp(_head_b_samples + mod_r, 8.0f, static_cast<float>(_buffer.MaxSize() - 2));
            raw_l = _buffer.ReadL(pos_l);
            raw_r = _buffer.ReadR(pos_r);
        }

        float read_pos_l = base_read_samples + mod_l;
        float read_pos_r = base_read_samples + mod_r;

        // Tier 3: Hardware address bus slip & CD skips: low clock quality + clock FM + starved voltage
        float starve_slip = (v_rail_eff < 0.45f) ? ((0.45f - v_rail_eff) / 0.25f) : 0.0f;
        float clk_quality_drop = (_clk_fader_speed < 0.38f) ? ((0.38f - _clk_fader_speed) / 0.25f) : 0.0f;
        float clk_fm_glitch = std::abs(clk_fm) * clk_quality_drop * 1.0f;
        float glitch_intensity = std::max(mods.address_slip, std::max(starve_slip, clk_fm_glitch));

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

        // Bitwise XOR memory address scrambling (Bend or deep starvation < 0.30)
        float auto_xor = (v_rail_eff < 0.30f) ? ((0.30f - v_rail_eff) * 1.5f) : 0.0f;
        float eff_xor = std::max(mods.xor_glitch, auto_xor);
        if (eff_xor > 0.015f) {
            uint32_t mask = 0;
            if (eff_xor > 0.05f) mask |= 0x003F; // micro byte fragments
            if (eff_xor > 0.25f) mask |= 0x03C0; // mid address line scramble
            if (eff_xor > 0.55f) mask |= 0x1E00; // macro address bus short
            if (eff_xor > 0.85f) mask |= 0x6000; // wild ring bank short

            uint32_t offset_l = static_cast<uint32_t>(read_pos_l) ^ mask;
            uint32_t offset_r = static_cast<uint32_t>(read_pos_r) ^ (mask << 1);
            float xor_read_l = std::clamp(static_cast<float>(offset_l), 1.0f, static_cast<float>(_buffer.MaxSize() - 2));
            float xor_read_r = std::clamp(static_cast<float>(offset_r), 1.0f, static_cast<float>(_buffer.MaxSize() - 2));

            float xor_l = _buffer.ReadL(xor_read_l);
            float xor_r = _buffer.ReadR(xor_read_r);

            float blend = std::min(1.0f, eff_xor * 1.4f);
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
        // Clean 16-bit hi-fi path when clock fader is high and rail is good;
        // engages authentic PT2399 downsampling and DAC crush when clock fader is pulled down
        float out_dac_l, out_dac_r;
        if (eff_clk_speed >= 0.98f && v_rail_eff >= 0.95f && mods.decimate < 0.01f) {
            out_dac_l = _damp_l * shunt_dly;
            out_dac_r = _damp_r * shunt_dly;
        } else {
            _lofi_l.Prepare(eff_clk_speed, _sample_rate);
            _lofi_r.Prepare(eff_clk_speed, _sample_rate);
            float proc_l = _lofi_l.Process(_damp_l);
            float proc_r = _lofi_r.Process(_damp_r);

            float bit_depth = std::clamp(16.0f - starve * 9.0f - (1.0f - eff_clk_speed) * 6.0f, 6.0f, 16.0f);
            float levels = FastPow2(bit_depth);
            float inv_levels = 1.0f / levels;
            float dac_dnl = 0.05f + starve * 0.40f + (1.0f - eff_clk_speed) * 0.15f;

            out_dac_l = QuantizeDac(proc_l, levels, inv_levels, dac_dnl) * shunt_dly;
            out_dac_r = QuantizeDac(proc_r, levels, inv_levels, dac_dnl) * shunt_dly;
        }

        // Feedback calculation & Ping-Pong cross-coupling
        // Gentle feedback stabilization during active crossfades prevents phase accumulation crackles
        float fb_xfade_duck = _xfade_active ? (1.0f - 0.20f * (4.0f * _xfade_progress * (1.0f - _xfade_progress))) : 1.0f;
        float ping_pong_cross = 0.32f + mods.ping_pong;
        float cross = std::clamp(ping_pong_cross, 0.0f, 1.0f);
        float direct = 1.0f - cross;

        // Rhythmic dub feedback burst on clock beat (Bend: P00 CLK <-> P02 FBK)
        float clock_burst_fb = (mods.fbk_clock_burst > 0.01f) ? (clk_phase < 0.22f ? mods.fbk_clock_burst * 0.65f : 0.0f) : 0.0f;
        float total_fb = (eff_fb + clock_burst_fb) * shunt_fbk * fb_xfade_duck;

        float fb_tot_l = (_damp_l * direct + _damp_r * cross) * total_fb;
        float fb_tot_r = (_damp_r * direct + _damp_l * cross) * total_fb;

        // SDRAM buffer write (inhibited during freeze or deep glitch)
        bool write_inhibited = eff_frozen || (glitch_intensity > 0.15f) || (eff_xor > 0.60f);
        if (!write_inhibited) {
            float dly_in_l = _dc_block_l.Process(in_l + FastTanh(fb_tot_l));
            float dly_in_r = _dc_block_r.Process(in_r + FastTanh(fb_tot_r));
            _buffer.Write(dly_in_l, dly_in_r, eff_clk_speed);
        } else if (!eff_frozen) {
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
    float _head_a_samples{28.0f};
    float _head_b_samples{28.0f};
    uint8_t _active_head{0};
    bool  _xfade_active{false};
    float _xfade_progress{0.0f};
    int8_t _xfade_dir{1};
    static constexpr uint16_t kXfadeSamples{1680}; // ~35ms at 48kHz
    float _knob_dly_held{0.50f};
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
    float _clk_fm_slow{0.0f};

    bool  _tape_braking{false};
    float _tape_brake_speed{1.0f};
    float _tape_brake_offset{28.0f};
};

} // namespace bends
