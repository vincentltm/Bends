#pragma once

#include "module.h"
#include "fast_math.h"
#include "sdram_buffer.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

// Corrupted Memory & Digital Glitch sound source.
// Samples from SDRAM delay buffer and internal LFSR bit registers.
// Features 4 warm, non-harsh personalities with built-in low-pass filtering.
class CorruptModule : public Module {
public:
    enum class Personality : uint8_t {
        GrainScan   = 0, // Micro-window granular scanner hopping through memory
        AddressArp  = 1, // Stepped binary address arpeggios & retro computer indexing
        CombRing    = 2, // Ultra-short memory recirculator (metallic glassy resonance)
        BitThrob    = 3  // Bitwise XOR address masking / robotic computer murmurs
    };

    CorruptModule() = default;

    void Init(float sample_rate, uint16_t /*block_size*/) override {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() override {
        _rng = 0xBADC0DE1;
        _lfsr = 0xACE1u;
        _personality = Personality::GrainScan;
        _scan_pos = 0.0f;
        _grain_phase = 0.0f;
        _grain_anchor = 0;
        _arp_step = 0;
        _arp_timer = 0;
        _comb_ptr = 0;
        _comb_len = 64;
        _smooth_lp = 0.0f;
        _pressure = 0.0f;
        _volume = 0.0f;
        _pitch = 0.5f;
        _drive = 0.2f;
        _click_mod = 0.0f;
        _reverse_mod = 0.0f;
        _clock_mod = 0.0f;
        _diffusion_mod = 0.0f;
        _glitch_mod = 0.0f;
        _starve_mod = 0.0f;
        _drone_mod = 0.0f;
        _sh_hold_l = 0.0f;
        _sh_hold_r = 0.0f;
        _sh_timer = 0;
        _rec_state = RecordState::Idle;
        _rec_slot = 0;
        _rec_write_ptr = 0;
        _rec_cooldown = static_cast<uint32_t>(_sample_rate * 2.0f);
        _playback_slot = 0;
        for (size_t s = 0; s < kNumCorruptSlots; ++s) {
            _slot_has_audio[s] = false;
        }
    }

    void TriggerNewPersonality() {
        _rng = _rng * 1664525u + 1013904223u;
        uint8_t next = (static_cast<uint8_t>(_personality) + 1 + (_rng & 1)) % 4;
        _personality = static_cast<Personality>(next);

        // Cycle through historical memory snapshot slots
        _playback_slot = (_playback_slot + 1) % kNumCorruptSlots;

        // Pick new randomized anchor inside the active slot
        _grain_anchor = (_rng >> 8) & kCorruptSlotMask;
        _scan_pos = static_cast<float>(_grain_anchor);
        _grain_phase = 0.0f;
        _arp_step = 0;
        _arp_timer = 0;
        _comb_len = 32 + ((_rng >> 4) & 0x7F); // 32 to 159 samples (~300 Hz to 1500 Hz)
    }

    void SetPressure(float val)       { _pressure = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetVolume(float val)         { _volume = daisysp::fclamp(val, 0.0f, 0.40f); }
    void SetPitch(float val)          { _pitch = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetDrive(float val)          { _drive = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetClickMod(float val)       { _click_mod = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetReverseMod(float val)     { _reverse_mod = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetClockMod(float val)       { _clock_mod = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetDiffusionMod(float val)   { _diffusion_mod = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetGlitchMod(float val)      { _glitch_mod = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetStarveMod(float val)      { _starve_mod = daisysp::fclamp(val, 0.0f, 1.0f); }
    void SetDroneMod(float val)       { _drone_mod = daisysp::fclamp(val, 0.0f, 1.0f); }

    void Process(float in_l, float in_r, float& out_l, float& out_r) override {
        // Snapshot recorder into dedicated historical memory slots:
        // Automatically populates slots every now and then when audio is present
        float in_mag = std::max(std::abs(in_l), std::abs(in_r));

        if (_rec_state == RecordState::Recording) {
            g_sdram_corrupt_l[_rec_slot][_rec_write_ptr] = in_l;
            g_sdram_corrupt_r[_rec_slot][_rec_write_ptr] = in_r;
            _rec_write_ptr++;
            if (_rec_write_ptr >= kCorruptSlotSamples) {
                _slot_has_audio[_rec_slot] = true;
                _rec_state = RecordState::Idle;
                // Cooldown: 12s to 24s so slots capture distinctly different eras of the performance
                _rng = _rng * 1664525u + 1013904223u;
                uint32_t jitter_sec = (_rng >> 8) & 0x0B;
                _rec_cooldown = static_cast<uint32_t>(_sample_rate * (12.0f + jitter_sec));
                _rec_slot = (_rec_slot + 1) % kNumCorruptSlots;
            }
        } else {
            if (_rec_cooldown > 0) {
                _rec_cooldown--;
            } else if (in_mag > 0.025f) {
                // Audio detected and cooldown elapsed: start capturing into _rec_slot!
                _rec_state = RecordState::Recording;
                _rec_write_ptr = 0;
            }
        }

        float gen_drive = daisysp::fclamp((_drive - 0.03f) / 0.97f, 0.0f, 1.0f);
        if (_volume < 0.001f || gen_drive < 0.001f) {
            out_l = 0.0f;
            out_r = 0.0f;
            return;
        }

        // Advance internal 16-bit Galois LFSR for digital glitch seed
        uint16_t lsb = _lfsr & 1;
        _lfsr >>= 1;
        if (lsb) _lfsr ^= 0xB400u;
        float digital_noise = (static_cast<float>(_lfsr & 0xFF) / 255.0f - 0.5f) * 0.25f;

        float sig_l = 0.0f;
        float sig_r = 0.0f;

        // Direction: forward by default, reversed when bridged to P05
        float dir = (_reverse_mod > 0.3f) ? -1.0f : 1.0f;

        // P01 <-> P02 Click bridge: hops through historical memory slots
        if (_click_mod > 0.40f && (_rng & 0x7F) == 0) {
            _playback_slot = (_playback_slot + 1) % kNumCorruptSlots;
        }

        auto read_l = [this](size_t idx) -> float {
            if (_slot_has_audio[_playback_slot]) {
                return g_sdram_corrupt_l[_playback_slot][idx & kCorruptSlotMask];
            }
            return g_sdram_delay_l[idx & kMaxDelayMask];
        };

        auto read_r = [this](size_t idx) -> float {
            if (_slot_has_audio[_playback_slot]) {
                return g_sdram_corrupt_r[_playback_slot][idx & kCorruptSlotMask];
            }
            return g_sdram_delay_r[idx & kMaxDelayMask];
        };

        switch (_personality) {
            case Personality::GrainScan: {
                // Micro-window granular scanner
                float window_size = 128.0f + _pressure * 384.0f;
                _grain_phase += (1.0f / window_size) * (1.0f + _clock_mod * 2.0f);
                if (_grain_phase >= 1.0f) {
                    _grain_phase -= 1.0f;
                    _rng = _rng * 1664525u + 1013904223u;
                    // Hop to nearby address or jump if click mod
                    size_t hop = (_click_mod > 0.1f) ? ((_rng >> 6) & 0x3FFF) : ((_rng >> 8) & 0x3FF);
                    _grain_anchor = (_grain_anchor + hop) & kCorruptSlotMask;
                }

                // Grain window envelope (smooth sine window)
                float win = sinf(_grain_phase * 3.14159265f);

                size_t read_idx = (_grain_anchor + static_cast<size_t>(_grain_phase * window_size * dir)) & kCorruptSlotMask;
                float ram_l = read_l(read_idx);
                float ram_r = read_r(read_idx);

                // If slot is quiet, blend subtle digital noise
                ram_l += digital_noise * 0.25f;
                ram_r += digital_noise * 0.25f;

                sig_l = ram_l * win;
                sig_r = ram_r * win;
                break;
            }

            case Personality::AddressArp: {
                // Stepped binary address arpeggios
                _arp_timer++;
                uint32_t step_ticks = static_cast<uint32_t>(300.0f - _pressure * 180.0f - _clock_mod * 80.0f);
                if (step_ticks < 40) step_ticks = 40;

                if (_arp_timer >= step_ticks) {
                    _arp_timer = 0;
                    _arp_step = (_arp_step + 1) & 0x7;
                    if (_click_mod > 0.1f) {
                        _arp_step = (_arp_step + 3) & 0x7;
                    }
                }

                // Power of two address offset pattern
                static constexpr size_t kOffsets[8] = { 0, 64, 128, 256, 512, 1024, 256, 128 };
                size_t offset = kOffsets[_arp_step];
                size_t read_idx = (_grain_anchor + offset) & kCorruptSlotMask;

                float ram_l = read_l(read_idx);
                float ram_r = read_r(read_idx);

                // Light 8-bit quantization & bit mask
                int16_t q_l = static_cast<int16_t>(ram_l * 2048.0f) ^ (static_cast<int16_t>(_arp_step) << 4);
                int16_t q_r = static_cast<int16_t>(ram_r * 2048.0f) ^ (static_cast<int16_t>(_arp_step) << 4);

                sig_l = (static_cast<float>(q_l) / 2048.0f) * 0.40f + digital_noise * 0.15f;
                sig_r = (static_cast<float>(q_r) / 2048.0f) * 0.40f + digital_noise * 0.15f;
                break;
            }

            case Personality::CombRing: {
                // Ultra-short memory recirculator (metallic glassy resonance)
                _comb_ptr = (_comb_ptr + 1) % _comb_len;
                size_t read_idx = (_grain_anchor + _comb_ptr) & kCorruptSlotMask;

                float ram_l = read_l(read_idx) + digital_noise * 0.15f;
                float ram_r = read_r(read_idx) + digital_noise * 0.15f;

                // Modulate comb length with pressure and drone
                float pitch_mod = 1.0f + _drone_mod * 0.5f;
                _grain_anchor = (_grain_anchor + static_cast<size_t>(pitch_mod)) & kCorruptSlotMask;

                sig_l = ram_l * 0.60f;
                sig_r = ram_r * 0.60f;
                break;
            }

            case Personality::BitThrob: {
                // Bitwise XOR address masking / robotic computer murmurs
                _scan_pos += (0.8f + _pressure * 1.5f + _clock_mod * 2.0f) * dir;
                if (_scan_pos >= static_cast<float>(kCorruptSlotSamples)) _scan_pos -= static_cast<float>(kCorruptSlotSamples);
                if (_scan_pos < 0.0f) _scan_pos += static_cast<float>(kCorruptSlotSamples);

                size_t base_idx = static_cast<size_t>(_scan_pos);
                // Bitwise mask creates rhythmic fractals in address space
                uint32_t mask = 0x01E0 ^ ((_click_mod > 0.05f) ? 0x03C0 : 0);
                size_t read_idx_l = (base_idx ^ mask) & kCorruptSlotMask;
                size_t read_idx_r = (base_idx ^ (mask >> 1)) & kCorruptSlotMask;

                float ram_l = read_l(read_idx_l) + digital_noise * 0.20f;
                float ram_r = read_r(read_idx_r) + digital_noise * 0.20f;

                sig_l = ram_l * 0.50f;
                sig_r = ram_r * 0.50f;
                break;
            }
        }

        // Glitch S&H decimation if bridged to P09
        if (_glitch_mod > 0.08f) {
            uint32_t hold_period = static_cast<uint32_t>(_glitch_mod * 48.0f);
            if (++_sh_timer >= hold_period) {
                _sh_timer = 0;
                _sh_hold_l = sig_l;
                _sh_hold_r = sig_r;
            }
            sig_l = _sh_hold_l;
            sig_r = _sh_hold_r;
        }

        // Starve dropouts & bit crunch if starved or bridged to P04
        if (_starve_mod > 0.15f) {
            _rng = _rng * 1664525u + 1013904223u;
            if ((_rng & 0xFF) < static_cast<uint32_t>(_starve_mod * 80.0f)) {
                sig_l = 0.0f;
                sig_r = 0.0f;
            }
        }

        // Built-in warm low-pass filter: guarantees it's never harsh or loud!
        // Cutoff around 1.4 kHz - 1.8 kHz (one-pole filter)
        // Diffusion bridge softens it even further!
        float lp_coef = 0.16f * (1.0f - _diffusion_mod * 0.50f);
        daisysp::fonepole(_smooth_lp, sig_l, lp_coef);
        float filtered = _smooth_lp;

        // Gentle volume curve: scaled by _volume (capped at 0.35f)
        float eff_gain = _volume;
        out_l = daisysp::fclamp(filtered * eff_gain, -0.40f, 0.40f);
        out_r = daisysp::fclamp(filtered * eff_gain, -0.40f, 0.40f);
    }

    uint8_t NumParams() const override { return 3; }

    const char* GetParamName(uint8_t id) const override {
        switch (id) {
            case 0: return "Volume";
            case 1: return "Pressure";
            case 2: return "Pitch";
            default: return "";
        }
    }

    void SetParam(uint8_t id, float value) override {
        switch (id) {
            case 0: SetVolume(value); break;
            case 1: SetPressure(value); break;
            case 2: SetPitch(value); break;
        }
    }

    float GetParam(uint8_t id) const override {
        switch (id) {
            case 0: return _volume;
            case 1: return _pressure;
            case 2: return _pitch;
            default: return 0.0f;
        }
    }

    void SetAuxInput(uint8_t channel, float value) override {
        if (channel == 0) SetClickMod(value);
        else if (channel == 1) SetReverseMod(value);
    }

    const char* Name() const override { return "Corrupt"; }

private:
    float _sample_rate{48000.0f};
    uint32_t _rng{0xBADC0DE1};
    uint16_t _lfsr{0xACE1u};
    Personality _personality{Personality::GrainScan};

    float _scan_pos{0.0f};
    float _grain_phase{0.0f};
    size_t _grain_anchor{0};

    uint8_t _arp_step{0};
    uint32_t _arp_timer{0};

    size_t _comb_ptr{0};
    size_t _comb_len{64};

    float _smooth_lp{0.0f};
    float _pressure{0.0f};
    float _volume{0.0f};
    float _pitch{0.5f};
    float _drive{0.2f};

    float _click_mod{0.0f};
    float _reverse_mod{0.0f};
    float _clock_mod{0.0f};
    float _diffusion_mod{0.0f};
    float _glitch_mod{0.0f};
    float _starve_mod{0.0f};
    float _drone_mod{0.0f};

    float _sh_hold_l{0.0f};
    float _sh_hold_r{0.0f};
    uint32_t _sh_timer{0};

    enum class RecordState : uint8_t {
        Idle,
        Recording
    };

    RecordState _rec_state{RecordState::Idle};
    uint8_t     _rec_slot{0};
    size_t      _rec_write_ptr{0};
    uint32_t    _rec_cooldown{0};
    uint8_t     _playback_slot{0};
    bool        _slot_has_audio[kNumCorruptSlots]{false, false, false, false};
};

} // namespace bends
