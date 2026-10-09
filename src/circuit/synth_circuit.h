#pragma once

#include "circuit/node_table.h"
#include "lofi_delay_model.h"
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cstddef>

namespace bends {

struct ActiveBridge {
    uint8_t a;
    uint8_t b;
    float   g;
};

class SynthCircuit {
public:
    static constexpr size_t kMaxDelay = 19200; // 400ms at 48kHz (76.8 KB SRAM)
    static constexpr size_t kMaxActiveBridges = 24;

    enum OscMode : uint8_t {
        OSC_SIN = 0,
        OSC_SAW = 1,
        OSC_RND = 2
    };

    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        _clk_phase = 0.0f;

        // RC Relaxation oscillator & LFO
        _osc_vc = 0.0f;
        _osc_phase = 0.0f;
        _osc_charging = true;
        _osc_mode = OSC_SAW;
        _lfsr = 0xACE1u;
        _rnd_held = 0.0f;

        // State Variable Filter
        _flt_low = 0.0f;
        _flt_band = 0.0f;
        _flt_ping = 0.0f;

        // Preamp & Micro-events
        _amp_pluck = 0.0f;
        _hum_phase = 0.0f;
        _rst_tap = false;
        _is_frozen = false;

        _delay_write = 0;
        _smooth_delay_samples = 48000.0f * 0.15f;
        _prev_dly_read = 0.0f;
        _ks_damp = 0.0f;
        _inj_clk = 0.0f;
        _clk_beat = false;
        _lofi.Init();
        for (size_t i = 0; i < kMaxDelay; ++i) {
            _delay_buf[i] = 0.0f;
        }

        // Knob defaults
        _knob_drive_pos = 0.35f;
        _knob_osc_pos   = 0.25f;
        _knob_flt_pos   = 0.65f;
        _knob_dly_pos   = 0.50f;
        _knob_fbk_pos   = 0.40f;
        _knob_mix_pos   = 0.50f;
        _knob_clk_pos   = 0.15f;

        _num_bridges = 0;

        for (size_t i = 0; i < 12; ++i) {
            _v_dc[i] = 0.5f;
            _pressures[i] = 0.0f;
            _prev_pressures[i] = 0.0f;
        }
        _v_dc[NodeGnd] = 0.0f;
        _v_dc[NodeVcc] = 1.0f;
        _v_dc[NodeRst] = 1.0f;
    }

    void SetKnobs(float s30_drive, float s31_osc, float s32_flt,
                  float s33_dly, float s34_fbk, float s35_mix,
                  float s36_clk) {
        _knob_drive_pos = s30_drive;
        _knob_osc_pos   = s31_osc;
        _knob_flt_pos   = s32_flt;
        _knob_dly_pos   = s33_dly;
        _knob_fbk_pos   = s34_fbk;
        _knob_mix_pos   = s35_mix;
        _knob_clk_pos   = s36_clk;
    }

    void SetSwitches(uint8_t osc_mode, bool is_frozen) {
        _osc_mode = osc_mode;
        _is_frozen = is_frozen;
    }

    void SetPressures(const float pressures[12]) {
        for (size_t i = 0; i < 12; ++i) {
            float p = pressures[i];
            // Detect tap strikes (rising velocity)
            if (p > 0.15f && _prev_pressures[i] <= 0.05f) {
                if (i == NodeAmp) {
                    _amp_pluck = 0.85f * (_knob_drive_pos + 0.25f);
                } else if (i == NodeFlt) {
                    _flt_ping = 0.75f;
                } else if (i == NodeDly) {
                    _smooth_delay_samples += 1200.0f;
                } else if (i == NodeRst) {
                    _rst_tap = true;
                }
            }
            _pressures[i] = p;
            _prev_pressures[i] = p;
        }
    }

    void SetActiveBridges(const ActiveBridge* bridges, uint8_t count) {
        _num_bridges = (count < kMaxActiveBridges) ? count : kMaxActiveBridges;
        for (uint8_t i = 0; i < _num_bridges; ++i) {
            _bridges[i] = bridges[i];
        }
    }

    // Per-sample audio tick (48 kHz)
    inline float Tick(float ext_in, float vcc_rail) {
        // ====================================================================
        // 1. MASTER CLOCK (Node 0: CLK)
        // ====================================================================
        float clk_dc = _v_dc[NodeClk];

        // S36 clock frequency: 1.0 Hz (rhythmic click) up to 2400 Hz (audio-rate clock whistle)
        // Audio injection into CLK node FM-modulates clock rate (e.g. OSC -> CLK gives wild chorus & FM sidebands)
        float clk_fm = _inj_clk * 1.5f;
        float clk_rate = 1.0f * std::pow(2.0f, std::clamp(clk_dc * 11.23f + clk_fm, 0.0f, 11.5f));
        _clk_phase += clk_rate / _sample_rate;
        _clk_beat = false;
        if (_clk_phase >= 1.0f) {
            _clk_phase -= 1.0f;
            _clk_beat = true;
        }
        // Digital clock pulse waveform (-0.25 to +0.65)
        float s_clk = (_clk_phase < 0.28f) ? 0.65f : -0.25f;

        // Converter clock speed factor (1.15 down to 0.35)
        // CLK pad capacitance pressure also drags clock speed
        float clk_drag = 1.0f / (1.0f + _pressures[NodeClk] * 0.45f);
        float clk_speed = std::clamp((0.35f + 0.75f * clk_dc) * clk_drag, 0.20f, 1.15f);

        // ====================================================================
        // 2. PROBE OSCILLATOR (Node 3: OSC) — SIN, SAW, RND
        // ====================================================================
        float v_rail_eff = std::clamp(_v_dc[NodeVcc], 0.01f, 1.0f);
        float osc_freq = 0.15f * std::pow(2.0f, _knob_osc_pos * 14.0f);
        float dt = 1.0f / _sample_rate;

        _osc_phase += osc_freq * dt;
        if (_osc_phase >= 1.0f) {
            _osc_phase -= 1.0f;
            // Update S&H random voltage step
            _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
            _rnd_held = (static_cast<float>(_lfsr % 2001) - 1000.0f) / 1000.0f;
        }

        float s_osc = 0.0f;
        if (_osc_mode == OSC_SIN) {
            // Smooth sine wave
            s_osc = std::sin(_osc_phase * 6.2831853f) * std::min(1.0f, v_rail_eff * 1.5f);
        } else if (_osc_mode == OSC_SAW) {
            // RC Relaxation oscillator with starvation sputter
            float v_th_hi = 0.65f * v_rail_eff;
            float v_th_lo = 0.30f * v_rail_eff;
            float tau = 1.0f / (6.2831853f * std::max(0.1f, osc_freq));
            float k_charge = dt / tau;
            float k_discharge = dt / (0.12f * tau);

            if (v_rail_eff < 0.25f) {
                float choke = (v_rail_eff / 0.25f);
                k_charge *= (choke * choke);
            }

            if (_osc_charging) {
                _osc_vc += (v_rail_eff - _osc_vc) * k_charge;
                if (_osc_vc >= v_th_hi) _osc_charging = false;
            } else {
                _osc_vc -= _osc_vc * k_discharge;
                if (_osc_vc <= v_th_lo) _osc_charging = true;
            }

            float span = std::max(0.005f, v_th_hi - v_th_lo);
            float norm_vc = std::clamp((_osc_vc - v_th_lo) / span, 0.0f, 1.0f);
            s_osc = (2.0f * norm_vc - 1.0f) * std::min(1.0f, v_rail_eff * 1.5f);
        } else {
            // Stepped Random Sample & Hold
            s_osc = _rnd_held * std::min(1.0f, v_rail_eff * 1.5f);
        }

        // ====================================================================
        // 3. DC REST VOLTAGES & STIFFNESS ADMITTANCES
        // ====================================================================
        float v_rest[12];
        float g_stiff[12];

        v_rest[NodeClk] = _knob_clk_pos;
        g_stiff[NodeClk] = 3.0f;

        v_rest[NodeRst] = 1.0f;
        g_stiff[NodeRst] = 0.3f;

        // VCC pad pressure draws extra current, sagging rail
        float vcc_target = vcc_rail * (1.0f - _pressures[NodeVcc] * 0.40f);
        v_rest[NodeVcc] = vcc_target;
        g_stiff[NodeVcc] = 6.0f * (vcc_target * vcc_target) + 0.1f;

        v_rest[NodeOsc] = 0.5f;
        g_stiff[NodeOsc] = 1.5f;

        v_rest[NodeFlt] = _knob_flt_pos;
        g_stiff[NodeFlt] = 1.6f;

        v_rest[NodeGnd] = 0.0f;
        g_stiff[NodeGnd] = 25.0f;

        v_rest[NodeDly] = _knob_dly_pos;
        g_stiff[NodeDly] = 1.6f;

        v_rest[NodeFbk] = _knob_fbk_pos;
        g_stiff[NodeFbk] = 1.6f;

        v_rest[NodeAmp] = _knob_drive_pos;
        g_stiff[NodeAmp] = 1.8f;

        v_rest[NodeMix] = _knob_mix_pos;
        g_stiff[NodeMix] = 2.0f;

        v_rest[NodeIn]  = 0.5f;
        g_stiff[NodeIn]  = 3.0f;

        v_rest[NodeOut] = 0.5f;
        g_stiff[NodeOut] = 3.0f;

        // ====================================================================
        // 4. DC NODAL RELAXATION & AC BRIDGE INJECTION
        // ====================================================================
        float s_inj[12]{0.0f};

        if (_num_bridges == 0) {
            for (size_t i = 0; i < 12; ++i) {
                _v_dc[i] = v_rest[i];
            }
        } else {
            // Seed initial DC voltages
            for (size_t i = 0; i < 12; ++i) {
                _v_dc[i] = v_rest[i];
            }

            // 2-pass Jacobi DC relaxation
            for (int pass = 0; pass < 2; ++pass) {
                float sum_gv[12];
                float sum_g[12];

                for (size_t i = 0; i < 12; ++i) {
                    sum_gv[i] = g_stiff[i] * v_rest[i];
                    sum_g[i]  = g_stiff[i];
                }

                for (uint8_t k = 0; k < _num_bridges; ++k) {
                    uint8_t a = _bridges[k].a;
                    uint8_t b = _bridges[k].b;
                    float   g = _bridges[k].g;

                    sum_gv[a] += g * _v_dc[b];
                    sum_g[a]  += g;

                    sum_gv[b] += g * _v_dc[a];
                    sum_g[b]  += g;
                }

                for (size_t i = 0; i < 12; ++i) {
                    _v_dc[i] = std::clamp(sum_gv[i] / sum_g[i], 0.0f, 1.2f);
                }
            }

            // Map current node AC signals for bridge transmission
            float s_node[12]{0.0f};
            s_node[NodeClk] = s_clk;
            s_node[NodeOsc] = s_osc;
            s_node[NodeFlt] = _flt_low;
            s_node[NodeDly] = _prev_dly_read;
            s_node[NodeFbk] = _prev_dly_read;
            s_node[NodeAmp] = _prev_amp_out;
            s_node[NodeIn]  = ext_in;

            // Transmit AC audio across active bridges
            for (uint8_t k = 0; k < _num_bridges; ++k) {
                uint8_t a = _bridges[k].a;
                uint8_t b = _bridges[k].b;
                float   g = _bridges[k].g;

                s_inj[a] += g * s_node[b];
                s_inj[b] += g * s_node[a];
            }

            // Store injection into CLK node for next sample's FM clock modulation
            _inj_clk = s_inj[NodeClk];

            // OSC ↔ VCC bridge (P03 ↔ P02) sways power rail
            if (s_inj[NodeVcc] != 0.0f) {
                v_rail_eff = std::clamp(v_rail_eff + s_inj[NodeVcc] * 0.35f, 0.01f, 1.0f);
            }
        }

        // ====================================================================
        // 5. AUDIO SIGNAL PATH
        // ====================================================================

        // Panic Reset (P01 tap or pulled below 0.35V)
        if (_rst_tap || _v_dc[NodeRst] < 0.35f) {
            _rst_tap = false;
            _flt_low = 0.0f;
            _flt_band = 0.0f;
            _ks_damp = 0.0f;
            _smooth_delay_samples = 48000.0f * 0.15f;
            _lofi.Reset();
            for (size_t i = 0; i < kMaxDelay; ++i) _delay_buf[i] = 0.0f;
        }

        // --- Preamp Stage (Node 8: AMP) ---
        // 50Hz mains hum on P08 hold
        _hum_phase += 50.0f * 6.2831853f / _sample_rate;
        if (_hum_phase >= 6.2831853f) _hum_phase -= 6.2831853f;
        float hum = 0.016f * _pressures[NodeAmp] * std::sin(_hum_phase);

        // Preamp input receives external audio + bridged injections (OSC, CLK, DLY) + hum
        float amp_in = ext_in + s_inj[NodeAmp] * 0.90f + hum;

        // Pluck transient (from P08 tap — plucks Karplus string or thumps preamp)
        if (_amp_pluck > 0.001f) {
            amp_in += _amp_pluck;
            _amp_pluck *= 0.995f;
        }

        // Faint air bleed from P03 OSC hold
        if (_pressures[NodeOsc] > 0.05f) {
            amp_in += 0.025f * _pressures[NodeOsc] * s_osc;
        }

        float drive = 0.2f + 2.2f * _knob_drive_pos;
        float headroom = std::clamp(v_rail_eff, 0.10f, 1.0f);
        float amp_out = FastTanh((amp_in * drive) / headroom) * headroom;
        _prev_amp_out = amp_out;

        // --- Resonant SVF (Node 4: FLT) ---
        // Pressure opens cutoff slightly
        float flt_dc = _v_dc[NodeFlt] + 0.12f * _pressures[NodeFlt];
        float flt_freq = 40.0f * std::pow(2.0f, flt_dc * 8.1f);
        if (s_inj[NodeFlt] != 0.0f) {
            flt_freq *= std::pow(2.0f, s_inj[NodeFlt] * 1.5f);
        }
        flt_freq = std::clamp(flt_freq, 25.0f, 14000.0f);

        float f = 2.0f * std::sin(3.14159265f * flt_freq / _sample_rate);
        f = std::clamp(f, 0.005f, 0.95f);

        // Filter ping (from P04 tap)
        if (_flt_ping > 0.001f) {
            _flt_band += _flt_ping;
            _flt_ping = 0.0f;
        }

        // Bridge AMP ↔ FLT (P08 ↔ P04) hard-drives filter input
        float flt_input_sig = amp_out * (1.0f + _bridges_conductance(NodeAmp, NodeFlt) * 1.5f);
        _flt_low += f * _flt_band;
        float flt_high = flt_input_sig - _flt_low - 0.35f * _flt_band;
        _flt_band += f * flt_high;
        _flt_band = FastTanh(_flt_band * 1.2f) * 0.85f;

        // --- Lo-Fi Delay & Karplus-Strong (Node 6: DLY & Node 7: FBK) ---
        // S33 Delay mapping:
        // u in [0.0, 0.25]: Karplus-Strong string (0.5ms to 20ms = 2000 Hz to 50 Hz)
        // u in [0.25, 1.0]: Echo & Ambient Wash (20ms to 395ms)
        float u = _v_dc[NodeDly];
        bool in_karplus = (u < 0.25f);
        float base_time;
        if (in_karplus) {
            float norm_k = u / 0.25f;
            base_time = 0.0005f * std::pow(40.0f, norm_k); // 0.5ms (2000 Hz) to 20ms (50 Hz)
        } else {
            float norm_e = (u - 0.25f) / 0.75f;
            base_time = 0.020f + norm_e * 0.375f; // 20ms to 395ms
        }

        float target_time = base_time / clk_speed;
        if (s_inj[NodeDly] != 0.0f) {
            target_time += s_inj[NodeDly] * (in_karplus ? 0.002f : 0.035f);
        }
        target_time = std::clamp(target_time, 0.0005f, 0.395f);
        float target_samples = target_time * _sample_rate;

        // One-pole readhead slewer (fast in Karplus for accurate pitch, responsive tape glide in echo)
        float slew_rate = in_karplus ? 0.08f : 0.025f;
        _smooth_delay_samples += slew_rate * (target_samples - _smooth_delay_samples);

        // Read raw buffer with smoothed interpolation
        float raw_dly = ReadDelay(_smooth_delay_samples);

        // Acoustic String Filter vs PT2399 Delay Model
        float dly_processed;
        if (in_karplus) {
            // One-pole acoustic loop damping filter (controlled by S32 FLT knob)
            float string_damp = 0.15f + 0.82f * _v_dc[NodeFlt];
            _ks_damp += string_damp * (raw_dly - _ks_damp);
            dly_processed = _ks_damp;
        } else {
            // Process through PT2399 clock downsampling, delta-slew, and op-amp filters
            _lofi.Prepare(target_time, v_rail_eff * clk_speed, _sample_rate);
            float lofi_read = _lofi.Process(raw_dly);

            // Vintage DAC Stage: 12-bit down to 6-bit + R-2R non-linearity
            float bit_depth = 6.0f + 6.0f * (clk_speed * v_rail_eff);
            float dac_dnl = 0.15f * (1.0f - clk_speed);
            dly_processed = ApplyDac(lofi_read, bit_depth, dac_dnl);
        }
        _prev_dly_read = dly_processed;

        // Feedback calculation
        float base_fb;
        if (in_karplus) {
            // Karplus-Strong string sustain (0.82 to 0.99)
            base_fb = 0.82f + 0.17f * _v_dc[NodeFbk];
        } else {
            base_fb = _v_dc[NodeFbk] * 0.85f;
        }
        // FBK pad pressure swell
        base_fb += _pressures[NodeFbk] * 0.30f;
        float eff_fb = base_fb + s_inj[NodeFbk] * 0.75f;
        eff_fb = std::clamp(eff_fb, 0.0f, 1.20f);

        // Delay input: if frozen, circulate indefinitely without writing input
        float dly_in;
        if (_is_frozen) {
            dly_in = FastTanh(dly_processed * 0.999f);
        } else {
            dly_in = _flt_low + FastTanh(dly_processed * eff_fb);
        }
        WriteDelay(dly_in);

        // --- Master Output (Node 9: MIX & Node 11: OUT) ---
        // Bridge AMP ↔ MIX (P08 ↔ P09) direct dry tap
        float dry_sig = _flt_low + amp_out * _bridges_conductance(NodeAmp, NodeMix) * 0.8f;
        float mix_wet = _v_dc[NodeMix];
        float mix_out = (1.0f - mix_wet) * dry_sig + mix_wet * dly_processed;

        return mix_out * 0.70f;
    }

    float GetNodeVoltage(size_t i) const {
        return (i < 12) ? _v_dc[i] : 0.0f;
    }

    bool ClkBeat() const {
        return _clk_beat;
    }

private:
    inline float _bridges_conductance(uint8_t a, uint8_t b) const {
        for (uint8_t k = 0; k < _num_bridges; ++k) {
            if ((_bridges[k].a == a && _bridges[k].b == b) ||
                (_bridges[k].a == b && _bridges[k].b == a)) {
                return _bridges[k].g;
            }
        }
        return 0.0f;
    }

    inline float ApplyDac(float in, float bits, float dnl) const {
        float levels = std::pow(2.0f, bits);
        float inv = 1.0f / levels;
        float q = std::round(in * levels) * inv;
        float sgn = (in >= 0.0f) ? 1.0f : -1.0f;
        q += sgn * dnl * inv * (1.0f - std::abs(in));
        return std::clamp(q, -1.0f, 1.0f);
    }

    inline float FastTanh(float x) const {
        if (x < -3.0f) return -1.0f;
        if (x > 3.0f) return 1.0f;
        return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
    }

    inline float ReadDelay(float delay_samples) const {
        float read_pos = static_cast<float>(_delay_write) - delay_samples;
        while (read_pos < 0.0f) read_pos += kMaxDelay;
        while (read_pos >= kMaxDelay) read_pos -= kMaxDelay;
        size_t idx0 = static_cast<size_t>(read_pos);
        size_t idx1 = (idx0 + 1) % kMaxDelay;
        float frac = read_pos - static_cast<float>(idx0);
        return _delay_buf[idx0] + frac * (_delay_buf[idx1] - _delay_buf[idx0]);
    }

    inline void WriteDelay(float sample) {
        _delay_buf[_delay_write] = sample;
        _delay_write = (_delay_write + 1) % kMaxDelay;
    }

    float _sample_rate{48000.0f};

    // Master Clock
    float _knob_clk_pos{0.15f};
    float _clk_phase{0.0f};
    float _inj_clk{0.0f};
    bool  _clk_beat{false};

    // Probe Oscillator
    uint8_t _osc_mode{OSC_SAW};
    float   _knob_osc_pos{0.25f};
    float   _osc_phase{0.0f};
    float   _osc_vc{0.0f};
    bool    _osc_charging{true};
    uint16_t _lfsr{0xACE1u};
    float   _rnd_held{0.0f};

    // Preamp Drive & Pluck
    float _knob_drive_pos{0.35f};
    float _prev_amp_out{0.0f};
    float _amp_pluck{0.0f};
    float _hum_phase{0.0f};

    // Resonant Filter
    float _knob_flt_pos{0.65f};
    float _flt_low{0.0f};
    float _flt_band{0.0f};
    float _flt_ping{0.0f};

    // Delay & Karplus-Strong
    float _knob_dly_pos{0.50f};
    float _knob_fbk_pos{0.40f};
    float _smooth_delay_samples{9600.0f};
    float _prev_dly_read{0.0f};
    float _ks_damp{0.0f};
    float _delay_buf[kMaxDelay];
    size_t _delay_write{0};
    LoFiDelayModel _lofi;
    bool  _is_frozen{false};
    bool  _rst_tap{false};

    // Mix
    float _knob_mix_pos{0.50f};

    // Active bridges for sparse relaxation
    ActiveBridge _bridges[kMaxActiveBridges];
    uint8_t      _num_bridges{0};

    // Resolved DC node voltages
    float _v_dc[12]{0.5f};

    // Pad pressures for micro-events
    float _pressures[12]{0.0f};
    float _prev_pressures[12]{0.0f};
};

} // namespace bends
