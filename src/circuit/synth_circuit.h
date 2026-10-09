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
        for (size_t i = 0; i < kMaxDelay; ++i) {
            _delay_buf[i] = 0.0f;
        }

        // Default knob biases
        _knob_drive = 0.8f;
        _knob_osc_freq = 120.0f;
        _knob_flt_freq = 800.0f;
        _knob_flt_res = 0.70f;
        _knob_dly_time = 0.22f;
        _knob_dly_fb = 0.35f;
        _knob_mix = 0.50f;
        _knob_clk_rate = 3.0f;

        _num_bridges = 0;

        for (size_t i = 0; i < 12; ++i) {
            _v[i] = 0.5f;
        }
        _v[NodeGnd] = 0.0f;
        _v[NodeVcc] = 1.0f;
        _v[NodeRst] = 1.0f;
    }

    void SetKnobs(float s30_drive, float s31_osc, float s32_flt,
                  float s33_dly, float s34_fbk, float s35_mix,
                  float s36_clk) {
        // S30: Preamp drive (0.1 to 2.2)
        _knob_drive = 0.1f + 2.1f * s30_drive;
        // S31: Relaxation oscillator frequency (0.1 Hz to 2400 Hz)
        _knob_osc_freq = 0.15f * std::pow(2.0f, s31_osc * 14.0f);
        // S32: Filter cutoff (40 Hz to 11000 Hz)
        _knob_flt_freq = 40.0f * std::pow(2.0f, s32_flt * 8.1f);
        // S33: Delay time (30ms to 380ms)
        _knob_dly_time = 0.03f + s33_dly * 0.35f;
        // S34: Delay feedback (0.0 to 0.85)
        _knob_dly_fb = s34_fbk * 0.85f;
        // S35: Master wet/dry mix
        _knob_mix = s35_mix;
        // S36: Master clock rate (0.5 Hz to 28 Hz)
        _knob_clk_rate = 0.5f + s36_clk * 27.5f;
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
        _clk_phase += _knob_clk_rate / _sample_rate;
        if (_clk_phase >= 1.0f) _clk_phase -= 1.0f;
        float clk_pulse = (_clk_phase < 0.20f) ? 1.0f : 0.0f;

        // ====================================================================
        // 2. RC RELAXATION OSCILLATOR (Node 3: OSC)
        // ====================================================================
        // Driven by effective voltage on VCC node (allows starvation to choke it)
        float v_rail_eff = std::clamp(_v[NodeVcc], 0.01f, 1.0f);

        // Natural RC charge / discharge thresholds scale with supply rail
        float v_th_hi = 0.65f * v_rail_eff;
        float v_th_lo = 0.30f * v_rail_eff;

        // Effective time constant from S31 pot
        float tau = 1.0f / (6.2831853f * std::max(0.1f, _knob_osc_freq));
        float dt = 1.0f / _sample_rate;
        float k_charge = dt / tau;
        float k_discharge = dt / (0.12f * tau); // Fast discharge snap

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

        // Form relaxation ramp waveform centered around 0.5V
        float span = std::max(0.005f, v_th_hi - v_th_lo);
        float norm_vc = std::clamp((_osc_vc - v_th_lo) / span, 0.0f, 1.0f);
        float s_osc = (2.0f * norm_vc - 1.0f) * std::min(1.0f, v_rail_eff * 1.5f);

        // ====================================================================
        // 3. INTERNAL NODE SOURCES & STIFFNESS ADMITTANCES
        // ====================================================================
        float v_int[12];
        float g_stiff[12];

        // Node 0: CLK (Stiff pulse generator)
        v_int[NodeClk] = 0.5f + 0.45f * (clk_pulse * 2.0f - 1.0f);
        g_stiff[NodeClk] = 4.0f;

        // Node 1: RST (Weak pull-up)
        v_int[NodeRst] = 1.0f;
        g_stiff[NodeRst] = 0.3f;

        // Node 2: VCC (Supply rail with internal resistance)
        v_int[NodeVcc] = vcc_rail;
        g_stiff[NodeVcc] = 6.0f * (vcc_rail * vcc_rail) + 0.1f;

        // Node 3: OSC (Floating probe oscillator — DISCONNECTED from audio by default!)
        v_int[NodeOsc] = 0.5f + 0.45f * s_osc;
        g_stiff[NodeOsc] = 1.5f;

        // Node 4: FLT (Filter cutoff bias + filter audio output)
        v_int[NodeFlt] = 0.5f + 0.45f * _flt_low;
        g_stiff[NodeFlt] = 1.6f;

        // Node 5: GND (Stiff 0V ground reference)
        v_int[NodeGnd] = 0.0f;
        g_stiff[NodeGnd] = 25.0f;

        // Node 6: DLY (Delay time bias + wet readhead output)
        v_int[NodeDly] = 0.5f + 0.45f * _prev_dly_read;
        g_stiff[NodeDly] = 1.6f;

        // Node 7: FBK (Feedback bias + recirculating audio)
        v_int[NodeFbk] = 0.5f + 0.45f * _prev_dly_read;
        g_stiff[NodeFbk] = 1.6f;

        // Node 8: AMP (Preamp overdrive input stage)
        v_int[NodeAmp] = 0.5f + 0.45f * ext_in;
        g_stiff[NodeAmp] = 1.8f;

        // Node 9: MIX (Wet/Dry mix output)
        float dry_sig = _flt_low;
        float mix_sig = (1.0f - _knob_mix) * dry_sig + _knob_mix * _prev_dly_read;
        v_int[NodeMix] = 0.5f + 0.45f * mix_sig;
        g_stiff[NodeMix] = 2.0f;

        // Node 10: IN (Ext In probe)
        v_int[NodeIn] = 0.5f + 0.45f * ext_in;
        g_stiff[NodeIn] = 3.0f;

        // Node 11: OUT (Ext Out probe)
        v_int[NodeOut] = v_int[NodeMix];
        g_stiff[NodeOut] = 3.0f;

        // ====================================================================
        // 4. BIDIRECTIONAL RELAXATION SOLVER (Voltage Divider / Norton)
        // ====================================================================
        if (_num_bridges == 0) {
            for (size_t i = 0; i < 12; ++i) {
                _v[i] = v_int[i];
            }
        } else {
            // Seed initial voltages
            for (size_t i = 0; i < 12; ++i) {
                _v[i] = v_int[i];
            }

            // 2-pass Jacobi relaxation
            for (int pass = 0; pass < 2; ++pass) {
                float sum_gv[12];
                float sum_g[12];

                for (size_t i = 0; i < 12; ++i) {
                    sum_gv[i] = g_stiff[i] * v_int[i];
                    sum_g[i]  = g_stiff[i];
                }

                for (uint8_t k = 0; k < _num_bridges; ++k) {
                    uint8_t a = _bridges[k].a;
                    uint8_t b = _bridges[k].b;
                    float   g = _bridges[k].g;

                    sum_gv[a] += g * _v[b];
                    sum_g[a]  += g;

                    sum_gv[b] += g * _v[a];
                    sum_g[b]  += g;
                }

                for (size_t i = 0; i < 12; ++i) {
                    float raw = sum_gv[i] / sum_g[i];
                    // Soft-limiting around [-0.2, +1.4]
                    _v[i] = std::clamp(raw, -0.2f, 1.4f);
                }
            }
        }

        // ====================================================================
        // 5. RESOLVED STATE FEEDBACK & AUDIO PROCESSING
        // ====================================================================

        // Panic Reset (Node 1 pulled below 0.35V)
        if (_v[NodeRst] < 0.35f) {
            _flt_low = 0.0f;
            _flt_band = 0.0f;
            for (size_t i = 0; i < kMaxDelay; ++i) _delay_buf[i] = 0.0f;
        }

        // Preamp Stage (Node 8: AMP)
        // Converts resolved node voltage back to AC audio: (V_8 - 0.5) * 2.0
        float amp_ac = (_v[NodeAmp] - 0.5f) * 2.0f;
        float headroom = std::clamp(_v[NodeVcc], 0.10f, 1.0f);
        float amp_out = FastTanh((amp_ac * _knob_drive) / headroom) * headroom;

        // Resonant State Variable Filter (Node 4: FLT)
        // Cutoff modulated by resolved voltage V_4
        float flt_dc_pull = (_v[NodeFlt] - 0.5f);
        float flt_freq = _knob_flt_freq * std::pow(2.0f, flt_dc_pull * 6.0f);
        flt_freq = std::clamp(flt_freq, 25.0f, 14000.0f);

        float f = 2.0f * std::sin(3.14159265f * flt_freq / _sample_rate);
        f = std::clamp(f, 0.005f, 0.95f);
        float q = 1.0f - _knob_flt_res;

        _flt_low += f * _flt_band;
        float flt_high = amp_out - _flt_low - q * _flt_band;
        _flt_band += f * flt_high;
        _flt_band = FastTanh(_flt_band * 1.2f) * 0.85f;

        // Lo-Fi Delay (Node 6: DLY & Node 7: FBK)
        // Delay time modulated by resolved voltage V_6
        float dly_time_mod = 1.0f + (_v[NodeDly] - 0.5f) * 0.8f;
        float dly_time = std::clamp(_knob_dly_time * dly_time_mod, 0.008f, 0.395f);
        float dly_samples = dly_time * _sample_rate;

        float dly_read = ReadDelay(dly_samples);
        _prev_dly_read = dly_read;

        // Feedback amount modulated by resolved voltage V_7
        float fb_mod = (_v[NodeFbk] - 0.5f) * 1.3f;
        float eff_fb = std::clamp(_knob_dly_fb + fb_mod, 0.0f, 1.25f);

        // Delay input: filter output + saturated feedback
        float dly_in = _flt_low + FastTanh(dly_read * eff_fb);
        WriteDelay(dly_in);

        // Master Output (Node 11: OUT)
        // Resolved voltage at Node 11 / Node 9 converted to AC audio
        float final_audio = (_v[NodeOut] - 0.5f) * 2.0f;
        return final_audio * 0.55f;
    }

    float GetNodeVoltage(size_t i) const {
        return (i < 12) ? _v[i] : 0.0f;
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
        size_t idx0 = static_cast<size_t>(read_pos) % kMaxDelay;
        size_t idx1 = (idx0 + 1) % kMaxDelay;
        float frac = read_pos - std::floor(read_pos);
        return _delay_buf[idx0] + frac * (_delay_buf[idx1] - _delay_buf[idx0]);
    }

    inline void WriteDelay(float sample) {
        _delay_buf[_delay_write] = sample;
        _delay_write = (_delay_write + 1) % kMaxDelay;
    }

    float _sample_rate{48000.0f};

    // Master Clock
    float _knob_clk_rate{3.0f};
    float _clk_phase{0.0f};

    // RC Relaxation Oscillator
    float _knob_osc_freq{120.0f};
    float _osc_vc{0.0f};
    bool  _osc_charging{true};

    // Preamp Drive
    float _knob_drive{0.8f};

    // Resonant Filter
    float _knob_flt_freq{800.0f};
    float _knob_flt_res{0.70f};
    float _flt_low{0.0f};
    float _flt_band{0.0f};

    // Delay
    float _knob_dly_time{0.22f};
    float _knob_dly_fb{0.35f};
    float _prev_dly_read{0.0f};
    float _delay_buf[kMaxDelay];
    size_t _delay_write{0};

    // Mix
    float _knob_mix{0.50f};

    // Active bridges for sparse relaxation
    ActiveBridge _bridges[kMaxActiveBridges];
    uint8_t      _num_bridges{0};

    // Resolved node voltages
    float _v[12]{0.5f};
};

} // namespace bends
