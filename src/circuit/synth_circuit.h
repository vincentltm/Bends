#pragma once

#include "circuit/node_table.h"
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

    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        _clk_phase = 0.0f;

        // RC Relaxation oscillator
        _osc_vc = 0.0f;
        _osc_charging = true;

        // State Variable Filter
        _flt_low = 0.0f;
        _flt_band = 0.0f;

        // Lo-Fi Delay line
        _delay_write = 0;
        _smooth_delay_samples = 48000.0f * 0.20f;
        _prev_dly_read = 0.0f;
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
        float clk_rate = 0.5f + _knob_clk_pos * 27.5f;
        _clk_phase += clk_rate / _sample_rate;
        if (_clk_phase >= 1.0f) _clk_phase -= 1.0f;
        float s_clk = (_clk_phase < 0.20f) ? 0.7f : -0.2f;

        // ====================================================================
        // 2. RC RELAXATION OSCILLATOR (Node 3: OSC)
        // ====================================================================
        // Driven by effective voltage on VCC node (starvation chokes it)
        float v_rail_eff = std::clamp(_v_dc[NodeVcc], 0.01f, 1.0f);
        float v_th_hi = 0.65f * v_rail_eff;
        float v_th_lo = 0.30f * v_rail_eff;

        float osc_freq = 0.15f * std::pow(2.0f, _knob_osc_pos * 14.0f);
        float tau = 1.0f / (6.2831853f * std::max(0.1f, osc_freq));
        float dt = 1.0f / _sample_rate;
        float k_charge = dt / tau;
        float k_discharge = dt / (0.12f * tau);

        // Under deep starvation (< 0.25V), charging current chokes quadratically
        if (v_rail_eff < 0.25f) {
            float choke = (v_rail_eff / 0.25f);
            k_charge *= (choke * choke);
        }

        if (_osc_charging) {
            _osc_vc += (v_rail_eff - _osc_vc) * k_charge;
            if (_osc_vc >= v_th_hi) {
                _osc_charging = false;
            }
        } else {
            _osc_vc -= _osc_vc * k_discharge;
            if (_osc_vc <= v_th_lo) {
                _osc_charging = true;
            }
        }

        float span = std::max(0.005f, v_th_hi - v_th_lo);
        float norm_vc = std::clamp((_osc_vc - v_th_lo) / span, 0.0f, 1.0f);
        float s_osc = (2.0f * norm_vc - 1.0f) * std::min(1.0f, v_rail_eff * 1.5f);

        // ====================================================================
        // 3. DC REST VOLTAGES & STIFFNESS ADMITTANCES
        // ====================================================================
        float v_rest[12];
        float g_stiff[12];

        v_rest[NodeClk] = 0.5f;
        g_stiff[NodeClk] = 4.0f;

        v_rest[NodeRst] = 1.0f;
        g_stiff[NodeRst] = 0.3f;

        v_rest[NodeVcc] = vcc_rail;
        g_stiff[NodeVcc] = 6.0f * (vcc_rail * vcc_rail) + 0.1f;

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
        }

        // ====================================================================
        // 5. AUDIO SIGNAL PATH
        // ====================================================================

        // Panic Reset (Node 1 pulled below 0.35V)
        if (_v_dc[NodeRst] < 0.35f) {
            _flt_low = 0.0f;
            _flt_band = 0.0f;
            _smooth_delay_samples = 48000.0f * 0.20f;
            for (size_t i = 0; i < kMaxDelay; ++i) _delay_buf[i] = 0.0f;
        }

        // --- Preamp Stage (Node 8: AMP) ---
        // Audio input receives external audio + any bridged injections (e.g. OSC, CLK)
        float amp_in = ext_in + s_inj[NodeAmp] * 0.90f;
        float drive = 0.2f + 2.2f * _knob_drive_pos;
        float headroom = std::clamp(_v_dc[NodeVcc], 0.10f, 1.0f);
        float amp_out = FastTanh((amp_in * drive) / headroom) * headroom;
        _prev_amp_out = amp_out;

        // --- Resonant SVF (Node 4: FLT) ---
        // Cutoff set by resolved DC voltage at Node 4 + audio-rate FM from s_inj
        float flt_freq = 40.0f * std::pow(2.0f, _v_dc[NodeFlt] * 8.1f);
        if (s_inj[NodeFlt] != 0.0f) {
            flt_freq *= std::pow(2.0f, s_inj[NodeFlt] * 1.5f);
        }
        flt_freq = std::clamp(flt_freq, 25.0f, 14000.0f);

        float f = 2.0f * std::sin(3.14159265f * flt_freq / _sample_rate);
        f = std::clamp(f, 0.005f, 0.95f);
        float q = 0.30f; // Warm resonance

        _flt_low += f * _flt_band;
        float flt_high = amp_out - _flt_low - q * _flt_band;
        _flt_band += f * flt_high;
        _flt_band = FastTanh(_flt_band * 1.2f) * 0.85f;

        // --- Lo-Fi Delay (Node 6: DLY & Node 7: FBK) ---
        // Delay time smoothly tracks resolved DC time + pitch mod (chorus/vibrato)
        float target_time = 0.03f + _v_dc[NodeDly] * 0.35f;
        if (s_inj[NodeDly] != 0.0f) {
            target_time += s_inj[NodeDly] * 0.03f;
        }
        target_time = std::clamp(target_time, 0.01f, 0.395f);
        float target_samples = target_time * _sample_rate;

        // Smooth delay time with one-pole filter to guarantee ZERO read pointer tearing/noise!
        _smooth_delay_samples += 0.005f * (target_samples - _smooth_delay_samples);

        float dly_read = ReadDelay(_smooth_delay_samples);
        _prev_dly_read = dly_read;

        // Feedback: normal knob setting + injected wet signal from P06 ↔ P07 bridge
        float eff_fb = _v_dc[NodeFbk] * 0.82f + s_inj[NodeFbk] * 0.75f;
        eff_fb = std::clamp(eff_fb, 0.0f, 1.20f);

        // Delay input: filter output + warm saturated feedback
        float dly_in = _flt_low + FastTanh(dly_read * eff_fb);
        WriteDelay(dly_in);

        // --- Master Output (Node 9: MIX & Node 11: OUT) ---
        float mix_wet = _v_dc[NodeMix];
        float mix_out = (1.0f - mix_wet) * _flt_low + mix_wet * dly_read;

        return mix_out * 0.70f;
    }

    float GetNodeVoltage(size_t i) const {
        return (i < 12) ? _v_dc[i] : 0.0f;
    }

private:
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

    // RC Relaxation Oscillator
    float _knob_osc_pos{0.25f};
    float _osc_vc{0.0f};
    bool  _osc_charging{true};

    // Preamp Drive
    float _knob_drive_pos{0.35f};
    float _prev_amp_out{0.0f};

    // Resonant Filter
    float _knob_flt_pos{0.65f};
    float _flt_low{0.0f};
    float _flt_band{0.0f};

    // Delay
    float _knob_dly_pos{0.50f};
    float _knob_fbk_pos{0.40f};
    float _smooth_delay_samples{9600.0f};
    float _prev_dly_read{0.0f};
    float _delay_buf[kMaxDelay];
    size_t _delay_write{0};

    // Mix
    float _knob_mix_pos{0.50f};

    // Active bridges for sparse relaxation
    ActiveBridge _bridges[kMaxActiveBridges];
    uint8_t      _num_bridges{0};

    // Resolved DC node voltages
    float _v_dc[12]{0.5f};
};

} // namespace bends
