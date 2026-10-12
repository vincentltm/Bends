#pragma once

#include "circuit/node_table.h"
#include "dsp/lofi_delay_model.h"
#include "dsp/contact_mic_model.h"
#include "core/fast_math.h"
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

// Lightweight 1-pole DC blocker (highpass at ~3 Hz at 48 kHz)
struct DcBlocker {
    float x1{0.0f};
    float y1{0.0f};

    inline void Reset() {
        x1 = 0.0f;
        y1 = 0.0f;
    }

    inline float Process(float in) {
        if (std::isnan(in) || std::isinf(in)) in = 0.0f;
        float out = in - x1 + 0.9996f * y1;
        if (std::abs(out) < 1e-15f) out = 0.0f; // Flush denormals
        x1 = in;
        y1 = out;
        return out;
    }
};

struct PadGestureTracker {
    enum State : uint8_t {
        Idle,
        Onset,
        Held
    };

    State   state{Idle};
    uint8_t rest_scans{10};
    uint8_t onset_scans{0};
    uint8_t lockout_scans{0};
    float   onset_peak_dp{0.0f};
    float   onset_max_p{0.0f};
    float   rub_speed{0.0f};
    float   prev_p{0.0f};

    void Init() {
        state = Idle;
        rest_scans = 10;
        onset_scans = 0;
        lockout_scans = 0;
        onset_peak_dp = 0.0f;
        onset_max_p = 0.0f;
        rub_speed = 0.0f;
        prev_p = 0.0f;
    }

    void Update(float p, float& out_tap_vel) {
        out_tap_vel = 0.0f;
        float dp = p - prev_p;

        if (p < 0.04f) {
            if (rest_scans < 255) rest_scans++;
            state = Idle;
            rub_speed = 0.0f;
            onset_scans = 0;
            lockout_scans = 0;
            prev_p = p;
            return;
        }

        switch (state) {
        case Idle: {
            if (rest_scans >= 3) {
                state = Onset;
                onset_scans = 1;
                onset_peak_dp = (dp > 0.0f) ? dp : 0.0f;
                onset_max_p = p;
                rub_speed = 0.0f;
            } else {
                state = Held;
                lockout_scans = 6;
                rub_speed = 0.0f;
            }
            rest_scans = 0;
            break;
        }

        case Onset: {
            rest_scans = 0;
            onset_scans++;
            if (dp > onset_peak_dp) {
                onset_peak_dp = dp;
            }
            if (p > onset_max_p) {
                onset_max_p = p;
            }

            if (onset_max_p > 0.12f && onset_peak_dp > 0.06f) {
                float vel = onset_peak_dp * 2.8f + (onset_max_p - 0.12f) * 1.2f;
                out_tap_vel = std::clamp(vel, 0.15f, 1.0f);
                state = Held;
                lockout_scans = 25; // ~75ms lockout suppresses scratch during tap rebound
                rub_speed = 0.0f;
            } else if (onset_scans >= 3) {
                // Soft touchdown: brief 2-scan (~6ms) settle so scratching begins immediately
                state = Held;
                lockout_scans = 2;
                rub_speed = 0.0f;
            }
            break;
        }

        case Held: {
            rest_scans = 0;
            if (lockout_scans > 0) {
                lockout_scans--;
                rub_speed = 0.0f;
            } else {
                float abs_dp = std::abs(dp);
                if (abs_dp > 0.0004f) {
                    float excess = abs_dp - 0.0004f;
                    // Responsive curve: square root component gives immediate presence to feather scratches
                    float motion = std::clamp(excess * 20.0f + std::sqrt(excess) * 0.50f, 0.0f, 1.2f);
                    rub_speed += (motion - rub_speed) * 0.50f;
                } else {
                    rub_speed *= 0.86f;
                    if (rub_speed < 0.002f) rub_speed = 0.0f;
                }
            }
            break;
        }
        }

        prev_p = p;
    }
};

class SynthCircuit {
public:
    static constexpr size_t kMaxDelay = 768000; // 16.0s at 48kHz (3.07 MB in SDRAM)
    static constexpr size_t kMaxActiveBridges = 24;
    static constexpr float  kTwoPi = 6.28318530718f;
    static constexpr float  kPi    = 3.14159265359f;

    enum OscMode : uint8_t {
        OSC_SIN = 0,
        OSC_SAW = 1,
        OSC_RND = 2
    };

    void Init(float sample_rate, float* delay_buf_l = nullptr, float* delay_buf_r = nullptr, size_t max_delay = kMaxDelay) {
        _sample_rate = sample_rate;
        _clk_phase = 0.0f;

        // RC Relaxation oscillator & LFO
        _osc_vc = 0.0f;
        _osc_phase = 0.0f;
        _osc_charging = true;
        _osc_mode = OSC_SAW;
        _saw_ramp = 0.0f;
        _saw_reset_pulse = 0.0f;
        _lfsr = 0xACE1u;
        _rnd_held = 0.0f;

        // Topology-Preserving Transform (TPT) State Variable Filter (Stereo)
        _flt_low_l = 0.0f;
        _flt_low_r = 0.0f;
        _flt_s1_l = 0.0f;
        _flt_s2_l = 0.0f;
        _flt_s1_r = 0.0f;
        _flt_s2_r = 0.0f;
        _flt_ping = 0.0f;

        // Preamp & Micro-events
        _amp_pluck = 0.0f;
        _rst_tap = false;
        _is_frozen = false;
        _prev_amp_out_l = 0.0f;
        _prev_amp_out_r = 0.0f;

        // Lo-Fi Dual Delay lines in SDRAM
        _delay_buf_l = delay_buf_l;
        _delay_buf_r = delay_buf_r;
        _max_delay = (delay_buf_l != nullptr && delay_buf_r != nullptr) ? max_delay : 0;
        _delay_write_pos = 0.0f;
        _smooth_delay_samples = 28.0f;
        _dly_knob_held = 0.50f;
        _delay_time_curr = 28.0f;
        _delay_time_prev = 28.0f;
        _delay_xfade = 1.0f;
        _delay_time_initialized = false;
        _g_osc_amp_smooth = 0.0f;
        _damp_l = 0.0f;
        _damp_r = 0.0f;
        _prev_dly_read_l = 0.0f;
        _prev_dly_read_r = 0.0f;
        _inj_clk = 0.0f;
        _inj_osc = 0.0f;
        _clk_beat = false;
        _motorboat_phase = 0.0f;
        _rev_phase_l = 0.0f;
        _rev_phase_r = 0.0f;
        _stutter_pos_l = 0.0f;
        _stutter_pos_r = 0.0f;
        _sh_step_ratio = 1.0f;
        _rst_gnd_hold = 0;
        _slip_timer = 0;
        _slip_len = 2400;
        _slip_offset_l = 0.0f;
        _slip_offset_r = 0.0f;
        _was_frozen = false;
        _freeze_len = 48000.0f;
        _freeze_phase = 0.0f;
        _freeze_anchor = 0;
        _clk_rate = 1.0f;
        _flt_ping = 0.0f;
        _flt_ping_env = 0.0f;
        _lofi_l.Init();
        _lofi_r.Init();
        _contact_mic.Init(sample_rate);
        _rub_speed_p10 = 0.0f;
        _rub_speed_p11 = 0.0f;
        _tracker_p10.Init();
        _tracker_p11.Init();
        for (size_t i = 0; i < 12; ++i) {
            _rest_scans[i] = 10;
        }
        if (_delay_buf_l && _delay_buf_r && _max_delay > 0) {
            std::fill_n(_delay_buf_l, _max_delay, 0.0f);
            std::fill_n(_delay_buf_r, _max_delay, 0.0f);
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
            _shunt_gnd_target[i] = 1.0f;
            _shunt_gnd_smooth[i] = 1.0f;
            _bridged_weight[i]   = 0.0f;
            for (size_t j = 0; j < 12; ++j) {
                _bridge_g[i][j] = 0.0f;
            }
        }

        for (size_t i = 0; i < 12; ++i) {
            _v_dc_target[i]      = 0.5f;
            _v_dc_smooth[i]      = 0.5f;
            _pressures[i]        = 0.0f;
            _pressures_smooth[i] = 0.0f;
            _prev_pressures[i]   = 0.0f;
            _proximities[i]      = 0.0f;
        }
        _smooth_gnd_hum = 0.0f;
        _gnd_hum_phase = 0.0f;
        _gnd_crackle_rng = 13579u;
        _gnd_crackle_filter = 0.0f;

        _v_dc_target[NodeGnd] = 0.0f;
        _v_dc_smooth[NodeGnd] = 0.0f;
        _v_dc_target[NodeVcc] = 1.0f;
        _v_dc_smooth[NodeVcc] = 1.0f;
        _v_dc_target[NodeRst] = 1.0f;
        _v_dc_smooth[NodeRst] = 1.0f;

        _prev_out_l = 0.0f;
        _prev_out_r = 0.0f;

        _dc_block_amp_l.Reset();
        _dc_block_amp_r.Reset();
        _dc_block_flt_l.Reset();
        _dc_block_flt_r.Reset();
        _dc_block_dly_l.Reset();
        _dc_block_dly_r.Reset();
    }

    void SetProximities(const float proximities[12]) {
        for (size_t i = 0; i < 12; ++i) {
            _proximities[i] = proximities[i];
        }
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

    void SetPressures(const float pressures[12], bool new_scan = true) {
        for (size_t i = 0; i < 12; ++i) {
            _pressures[i] = pressures[i];
        }

        if (!new_scan) {
            return;
        }

        for (size_t i = 0; i < 12; ++i) {
            float p = pressures[i];
            if (p <= 0.05f) {
                if (_rest_scans[i] < 255) _rest_scans[i]++;
            } else {
                if (_rest_scans[i] >= 3) {
                    if (p > 0.14f) {
                        if (i == NodeAmp) {
                            _amp_pluck = 0.85f * (_knob_drive_pos + 0.25f);
                        } else if (i == NodeFlt) {
                            _flt_ping = 0.75f;
                        } else if (i == NodeRst) {
                            bool other_pads_held = false;
                            for (size_t k = 0; k < 12; ++k) {
                                if (k != NodeRst && pressures[k] > 0.08f) {
                                    other_pads_held = true;
                                    break;
                                }
                            }
                            if (!other_pads_held && _num_bridges == 0) {
                                _rst_tap = true;
                            }
                        }
                    }
                }
                _rest_scans[i] = 0;
            }
        }

        // Stereo Contact Mic Gestures
        float tap_vel_10 = 0.0f;
        _tracker_p10.Update(pressures[NodeIn], tap_vel_10);
        if (tap_vel_10 > 0.0f) {
            _contact_mic.TriggerTap(tap_vel_10, false);
        }
        _rub_speed_p10 = _tracker_p10.rub_speed;

        float tap_vel_11 = 0.0f;
        _tracker_p11.Update(pressures[NodeOut], tap_vel_11);
        if (tap_vel_11 > 0.0f) {
            _contact_mic.TriggerTap(tap_vel_11, true);
        }
        _rub_speed_p11 = _tracker_p11.rub_speed;
    }

    void SetActiveBridges(const ActiveBridge* bridges, uint8_t count) {
        _num_bridges = (count < kMaxActiveBridges) ? count : kMaxActiveBridges;
        for (uint8_t i = 0; i < _num_bridges; ++i) {
            _bridges[i] = bridges[i];
        }

        // Clear and rebuild fast pairwise conductance lookup table
        for (size_t i = 0; i < 12; ++i) {
            for (size_t j = 0; j < 12; ++j) {
                _bridge_g[i][j] = 0.0f;
            }
        }
        for (uint8_t k = 0; k < _num_bridges; ++k) {
            uint8_t a = _bridges[k].a;
            uint8_t b = _bridges[k].b;
            if (a < 12 && b < 12) {
                _bridge_g[a][b] = _bridges[k].g;
                _bridge_g[b][a] = _bridges[k].g;
            }
        }

        // Precompute ground shunts and standalone isolation weights for all 12 physical nodes
        for (size_t i = 0; i < 12; ++i) {
            _shunt_gnd_target[i] = 1.0f - _bridge_g[i][NodeGnd] * 0.98f;
            float g_sum = 0.0f;
            for (size_t j = 0; j < 12; ++j) {
                if (i != j) {
                    g_sum += _bridge_g[i][j];
                }
            }
            _bridged_weight[i] = std::clamp(g_sum * 1.5f, 0.0f, 1.0f);
        }
    }

    // Block-rate (750 Hz) 2-pass Jacobi DC relaxation solver
    void SolveDC(float vcc_rail) {
        float v_rest[12];
        float g_stiff[12];

        v_rest[NodeClk] = _knob_clk_pos;
        g_stiff[NodeClk] = 3.0f;

        v_rest[NodeRst] = 1.0f;
        g_stiff[NodeRst] = 0.3f;

        float g_vcc_gnd = _bridge_g[NodeVcc][NodeGnd];
        // Standalone touch on VCC alone has minimal effect (just subtle body load ~3%)
        // The real voltage drop happens when slightly grounding: bridging VCC (P02) to GND (P05)
        float vcc_target = vcc_rail * (1.0f - _pressures[NodeVcc] * 0.03f);
        if (g_vcc_gnd > 0.01f) {
            vcc_target *= (1.0f - g_vcc_gnd * 0.98f);
        }
        v_rest[NodeVcc] = vcc_target;
        g_stiff[NodeVcc] = (g_vcc_gnd > 0.01f) ? 0.05f : (6.0f * (vcc_target * vcc_target) + 0.1f);

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

        if (_num_bridges == 0) {
            for (size_t i = 0; i < 12; ++i) {
                _v_dc_target[i] = v_rest[i];
            }
            return;
        }

        // Seed rest voltages
        for (size_t i = 0; i < 12; ++i) {
            _v_dc_target[i] = v_rest[i];
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

                // Functional parameter bends (DLY, RST, MIX, OSC, CLK, FBK) operate via dedicated DSP algorithms
                // and must NOT drag each other's DC resting knob positions!
                if (a == NodeDly || b == NodeDly ||
                    a == NodeRst || b == NodeRst ||
                    a == NodeMix || b == NodeMix ||
                    a == NodeOsc || b == NodeOsc ||
                    a == NodeClk || b == NodeClk ||
                    a == NodeFbk || b == NodeFbk) {
                    continue;
                }

                // DC relaxation only applies to physical voltage rails (VCC, GND) and starve loads.
                bool a_is_dc_rail = (a == NodeVcc || a == NodeGnd || b == NodeVcc || b == NodeGnd);
                bool a_is_dc_load = ((a == NodeFlt && b == NodeAmp) || (a == NodeAmp && b == NodeFlt));
                if (!a_is_dc_rail && !a_is_dc_load) {
                    continue;
                }

                sum_gv[a] += g * _v_dc_target[b];
                sum_g[a]  += g;

                sum_gv[b] += g * _v_dc_target[a];
                sum_g[b]  += g;
            }

            for (size_t i = 0; i < 12; ++i) {
                _v_dc_target[i] = std::clamp(sum_gv[i] / sum_g[i], 0.0f, 1.2f);
            }
        }
    }

    // Per-sample audio tick (48 kHz) with stereo I/O
    inline void Tick(float ext_l, float ext_r, float vcc_rail, float& out_l, float& out_r) {
        // Continuous per-sample parameter smoothing (~3.5 ms slew) to eliminate zipper noise
        for (size_t i = 0; i < 12; ++i) {
            _v_dc_smooth[i]      += 0.006f * (_v_dc_target[i] - _v_dc_smooth[i]);
            _pressures_smooth[i] += 0.008f * (_pressures[i] - _pressures_smooth[i]);
            _shunt_gnd_smooth[i] += 0.008f * (_shunt_gnd_target[i] - _shunt_gnd_smooth[i]);
        }
        float g_osc_amp = _bridge_g[NodeOsc][NodeAmp];
        _g_osc_amp_smooth += 0.006f * (g_osc_amp - _g_osc_amp_smooth);

        // Universal Pad Bridging Isolation Rule:
        // Standalone touch effects only fire when a pad is unconnected.
        // When bridged to any other pad, the standalone touch action is smoothly bypassed.
        float pad_standalone_p[12];
        for (size_t i = 0; i < 12; ++i) {
            pad_standalone_p[i] = _pressures_smooth[i] * (1.0f - _bridged_weight[i]);
        }

        // Master rail voltage: check resolved Vcc and direct VCC-GND short
        float g_vcc_gnd = _bridge_g[NodeVcc][NodeGnd];
        float v_rail_eff = std::clamp(_v_dc_smooth[NodeVcc], 0.02f, 1.0f);
        if (g_vcc_gnd > 0.01f) {
            v_rail_eff = std::min(v_rail_eff, vcc_rail * (1.0f - g_vcc_gnd * 0.98f));
            v_rail_eff = std::max(0.02f, v_rail_eff);
        }
        float starve = 1.0f - v_rail_eff;

        // 1. Master Clock generator
        float s_clk = 0.0f;
        float clk_vcc_sag = 1.0f;
        TickMasterClock(v_rail_eff, clk_vcc_sag, s_clk);

        // 2. Probe Oscillator
        float dt = 1.0f / _sample_rate;
        float s_osc = 0.0f;
        TickOscillator(v_rail_eff, clk_vcc_sag, dt, s_osc);

        // 3. Ground Hum & Electrostatic generator
        float total_ground_hum = 0.0f;
        TickGroundHum(v_rail_eff, starve, pad_standalone_p[NodeGnd], total_ground_hum);

        // 4. Stereo Chassis Contact Mic Processing (Pad 10 IN / Pad 11 OUT)
        // Shapes external audio and synthesizes physical pad scratches and taps
        float mic_l = 0.0f;
        float mic_r = 0.0f;
        _contact_mic.Process(ext_l, ext_r,
                             _pressures_smooth[NodeIn], _rub_speed_p10,
                             _pressures_smooth[NodeOut], _rub_speed_p11,
                             mic_l, mic_r);

        // AC Bridge audio transmission
        // Each node carries a stereo signal pair [Left, Right]
        float ext_in_l = mic_l * _shunt_gnd_smooth[NodeIn];
        float ext_in_r = mic_r * _shunt_gnd_smooth[NodeOut];

        float s_node[12][2]{{0.0f}};
        s_node[NodeClk][0] = s_clk;
        s_node[NodeClk][1] = s_clk;
        s_node[NodeOsc][0] = s_osc;
        s_node[NodeOsc][1] = s_osc;
        s_node[NodeFlt][0] = _flt_low_l;
        s_node[NodeFlt][1] = _flt_low_r;
        s_node[NodeGnd][0] = total_ground_hum;
        s_node[NodeGnd][1] = total_ground_hum;
        s_node[NodeDly][0] = _prev_dly_read_l;
        s_node[NodeDly][1] = _prev_dly_read_r;
        s_node[NodeFbk][0] = _prev_dly_read_l;
        s_node[NodeFbk][1] = _prev_dly_read_r;
        s_node[NodeAmp][0] = _prev_amp_out_l;
        s_node[NodeAmp][1] = _prev_amp_out_r;
        s_node[NodeMix][0] = _prev_out_l;
        s_node[NodeMix][1] = _prev_out_r;
        s_node[NodeIn][0]  = ext_in_l;
        s_node[NodeIn][1]  = ext_in_r;
        s_node[NodeOut][0] = _prev_out_l;
        s_node[NodeOut][1] = _prev_out_r;

        float s_inj[12][2]{{0.0f}};
        for (uint8_t k = 0; k < _num_bridges; ++k) {
            uint8_t a = _bridges[k].a;
            uint8_t b = _bridges[k].b;
            float   g = _bridges[k].g;

            // OSC <-> AMP (P03 <-> P08) is handled as a dedicated smooth volume injection in ProcessPreamp
            if ((a == NodeOsc && b == NodeAmp) || (a == NodeAmp && b == NodeOsc)) {
                continue;
            }

            s_inj[a][0] += g * s_node[b][0];
            s_inj[a][1] += g * s_node[b][1];

            s_inj[b][0] += g * s_node[a][0];
            s_inj[b][1] += g * s_node[a][1];
        }

        _inj_clk = (s_inj[NodeClk][0] + s_inj[NodeClk][1]) * 0.5f;
        _inj_osc = (s_inj[NodeOsc][0] + s_inj[NodeOsc][1]) * 0.5f;

        // OSC <-> VCC bridge sways power rail
        float inj_vcc = (s_inj[NodeVcc][0] + s_inj[NodeVcc][1]) * 0.5f;
        if (inj_vcc != 0.0f) {
            v_rail_eff = std::clamp(v_rail_eff + inj_vcc * 0.35f, 0.01f, 1.0f);
            starve = 1.0f - v_rail_eff;
        }

        // Hardware Reset: P01 tap OR intentional active-low short to ground (P01 <-> P05 bridge)
        float g_rst_gnd = _bridge_g[NodeRst][NodeGnd];
        bool rst_active = _rst_tap || (g_rst_gnd > 0.15f);
        if (rst_active) {
            _rst_tap = false;
            _rst_gnd_hold++;
            // On bridge engagement edge: complete flush of memory buffers, feedback, and filters
            if (_rst_gnd_hold == 1) {
                if (_delay_buf_l && _delay_buf_r && _max_delay > 0) {
                    std::fill_n(_delay_buf_l, _max_delay, 0.0f);
                    std::fill_n(_delay_buf_r, _max_delay, 0.0f);
                }
                _flt_low_l = 0.0f;
                _flt_low_r = 0.0f;
                _flt_s1_l = 0.0f;
                _flt_s2_l = 0.0f;
                _flt_s1_r = 0.0f;
                _flt_s2_r = 0.0f;
                _flt_ping = 0.0f;
                _flt_ping_env = 0.0f;
                _smooth_delay_samples = 28.0f;
                _dly_knob_held = 0.50f;
                _delay_time_curr = 28.0f;
                _delay_time_prev = 28.0f;
                _delay_xfade = 1.0f;
                _delay_time_initialized = false;
                _g_osc_amp_smooth = 0.0f;
                _damp_l = 0.0f;
                _damp_r = 0.0f;
                _prev_dly_read_l = 0.0f;
                _prev_dly_read_r = 0.0f;
                _rev_phase_l = 0.0f;
                _rev_phase_r = 0.0f;
                _stutter_pos_l = 0.0f;
                _stutter_pos_r = 0.0f;
                _sh_step_ratio = 1.0f;
                _slip_timer = 0;
                _slip_offset_l = 0.0f;
                _slip_offset_r = 0.0f;
                _was_frozen = false;
                _freeze_phase = 0.0f;
                _saw_ramp = 0.0f;
                _saw_reset_pulse = 0.0f;
                _inj_clk = 0.0f;
                _inj_osc = 0.0f;
                _lofi_l.Reset();
                _lofi_r.Reset();
                _contact_mic.Reset();
                _dc_block_amp_l.Reset();
                _dc_block_amp_r.Reset();
                _dc_block_flt_l.Reset();
                _dc_block_flt_r.Reset();
                _dc_block_dly_l.Reset();
                _dc_block_dly_r.Reset();
            }
        } else {
            _rst_gnd_hold = 0;
        }

        // 5. Preamp Stage: receives external contact mic audio + NodeIn injections (e.g. OUT->IN feedback) + AMP injections
        float amp_out_l = 0.0f;
        float amp_out_r = 0.0f;
        ProcessPreamp(ext_in_l, ext_in_r, s_inj[NodeAmp], s_inj[NodeIn], s_osc, s_clk, total_ground_hum, v_rail_eff, starve, pad_standalone_p[NodeAmp], amp_out_l, amp_out_r);
        _prev_amp_out_l = amp_out_l;
        _prev_amp_out_r = amp_out_r;

        // 6. Filter Stage
        float flt_low_l = 0.0f;
        float flt_low_r = 0.0f;
        ProcessFilter(amp_out_l, amp_out_r, s_inj[NodeFlt], v_rail_eff, starve, pad_standalone_p[NodeFlt], flt_low_l, flt_low_r);

        // 7. Delay Stage & Digital Memory Bends
        float dly_dac_l = 0.0f;
        float dly_dac_r = 0.0f;
        ProcessDelay(flt_low_l, flt_low_r, amp_out_l, amp_out_r, s_osc, pad_standalone_p[NodeClk], pad_standalone_p[NodeFbk], v_rail_eff, starve, dly_dac_l, dly_dac_r);

        // 8. Mixer & Master Output
        ProcessMixer(flt_low_l, flt_low_r, dly_dac_l, dly_dac_r, ext_l, ext_r, s_inj, pad_standalone_p[NodeMix], out_l, out_r);
        _prev_out_l = out_l;
        _prev_out_r = out_r;
    }

    float GetNodeVoltage(size_t i) const {
        return (i < 12) ? _v_dc_smooth[i] : 0.0f;
    }

    bool ClkBeat() const {
        return _clk_beat;
    }

private:
    // Helper: effective air proximity when hovering (shunted to 0 upon skin contact)
    inline float PadAirHover(size_t node) const {
        return _proximities[node] * (1.0f - std::min(1.0f, _pressures_smooth[node] * 2.0f));
    }

    // 1. Master Clock generator
    inline void TickMasterClock(float v_rail_eff, float& clk_vcc_sag, float& s_clk) {
        float clk_dc = _v_dc_smooth[NodeClk];
        float clk_fm = _inj_clk * 0.35f;
        clk_vcc_sag = 0.60f + 0.40f * (v_rail_eff * v_rail_eff);
        float clk_rate = FastPow2(std::clamp(clk_dc * 11.23f + clk_fm, 0.0f, 11.5f)) * clk_vcc_sag;
        _clk_rate = clk_rate;
        _clk_phase += clk_rate / _sample_rate;
        _clk_beat = false;
        if (_clk_phase >= 1.0f) {
            _clk_phase -= 1.0f;
            _clk_beat = true;
        }
        s_clk = ((_clk_phase < 0.28f) ? 0.65f : -0.25f) * _shunt_gnd_smooth[NodeClk];
    }

    // 2. Probe Oscillator
    inline void TickOscillator(float v_rail_eff, float clk_vcc_sag, float dt, float& s_osc) {
        // High musical LFO / audio-rate modulator:
        // - 0.25 Hz at minimum (4.0s slow rhythmic sweep)
        // - ~8 Hz at 9 o'clock (vibrato/flutter LFO)
        // - ~110 Hz at 12 o'clock center (warm musical bass pitch)
        // - ~1.5 kHz at maximum (musical audio-rate FM / bell carrier without ultrasonic harshness)
        float u = _knob_osc_pos;
        float octaves = u * (22.57f - 10.02f * u);
        float osc_freq = std::clamp(0.25f * FastPow2(octaves) * clk_vcc_sag, 0.1f, 1600.0f);
        float osc_air = PadAirHover(NodeOsc);
        osc_freq *= (1.0f + osc_air * 0.08f);

        // CLK <-> OSC bridge injection: clock pulses frequency-modulate the probe oscillator
        if (_inj_osc != 0.0f) {
            osc_freq *= FastPow2(_inj_osc * 0.75f);
        }

        _osc_phase += osc_freq * dt;
        if (_osc_phase >= 1.0f) {
            _osc_phase -= 1.0f;
            _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
            _rnd_held = (static_cast<float>(_lfsr % 2001) - 1000.0f) / 1000.0f;

            // Musical semitone / scale intervals for Random S&H stepped arpeggiator / buffer scatter
            // 1.0 (unison), 1.1892 (minor 3rd), 1.3348 (4th), 1.4983 (5th), 2.0 (octave up), 0.75 (4th down)
            static constexpr float kShSteps[6] = { 1.0f, 1.1892f, 1.3348f, 1.4983f, 2.0f, 0.75f };
            _sh_step_ratio = kShSteps[_lfsr % 6];
        }

        _saw_reset_pulse = 0.0f;
        if (_osc_mode == OSC_SIN) {
            s_osc = std::sin(_osc_phase * kTwoPi) * std::min(1.0f, v_rail_eff * 1.5f);
            _saw_ramp = 0.0f;
        } else if (_osc_mode == OSC_SAW) {
            float v_th_hi = 0.65f * v_rail_eff;
            float v_th_lo = 0.30f * v_rail_eff;
            float tau = 1.4236f / std::max(0.1f, osc_freq);
            float k_charge = dt / tau;
            // Rapid relaxation discharge: sharp flyback edge creates authentic acoustic transients
            float k_discharge = dt / (0.012f * tau);

            if (v_rail_eff < 0.25f) {
                float choke = (v_rail_eff / 0.25f);
                k_charge *= (choke * choke);
            }

            if (_osc_charging) {
                _osc_vc += (v_rail_eff - _osc_vc) * k_charge;
                if (_osc_vc >= v_th_hi) {
                    _osc_charging = false;
                    _saw_reset_pulse = 1.0f; // Single-sample transient pulse on flyback!
                }
            } else {
                _osc_vc -= _osc_vc * k_discharge;
                if (_osc_vc <= v_th_lo) {
                    _osc_charging = true;
                }
            }

            float span = std::max(0.005f, v_th_hi - v_th_lo);
            float norm_vc = std::clamp((_osc_vc - v_th_lo) / span, 0.0f, 1.0f);
            _saw_ramp = norm_vc;
            s_osc = (2.0f * norm_vc - 1.0f) * std::min(1.0f, v_rail_eff * 1.5f);
        } else {
            s_osc = _rnd_held * std::min(1.0f, v_rail_eff * 1.5f);
            _saw_ramp = 0.0f;
        }
        s_osc *= _shunt_gnd_smooth[NodeOsc];
    }

    // 3. Ground Hum & Electrostatic generator
    inline void TickGroundHum(float v_rail_eff, float starve, float standalone_p_gnd, float& total_ground_hum) {
        float gnd_prox = _proximities[NodeGnd];
        float eff_prox = std::max(0.0f, (gnd_prox - 0.10f) / 0.90f);

        // Standalone touch on GND (P05) injects organic body ground hum (50Hz mains hum) and electrostatic buzz
        float gnd_touch_hum = standalone_p_gnd * 0.045f;
        float hover_hum = (eff_prox * eff_prox) * 0.015f;
        float v_starve_hum = std::clamp((0.72f - v_rail_eff) / 0.55f, 0.0f, 1.0f) * 0.020f;

        float hum_target = gnd_touch_hum + hover_hum + v_starve_hum;
        _smooth_gnd_hum += (hum_target - _smooth_gnd_hum) * 0.008f;

        _gnd_hum_phase += 50.0f * kTwoPi / _sample_rate;
        if (_gnd_hum_phase >= kTwoPi) _gnd_hum_phase -= kTwoPi;

        float sin1 = std::sin(_gnd_hum_phase);
        float cos1 = std::cos(_gnd_hum_phase);
        float sin2 = 2.0f * sin1 * cos1;
        float sin3 = sin1 * (3.0f - 4.0f * sin1 * sin1);

        float harmonic_brightness = 0.20f + 0.40f * (standalone_p_gnd + eff_prox);
        float hum_composite = sin1 * 0.70f + (sin2 * 0.22f + sin3 * 0.08f) * harmonic_brightness;
        float buzz = FastTanh(hum_composite * 1.0f);

        // Electrostatic crackle generator
        _gnd_crackle_rng = _gnd_crackle_rng * 1664525u + 1013904223u;
        float white = static_cast<float>(static_cast<int32_t>(_gnd_crackle_rng)) * 4.6566129e-10f;
        float crackle = (white > 0.96f) ? (white - 0.96f) * 4.0f : ((white < -0.97f) ? (white + 0.97f) * 3.0f : 0.0f);
        _gnd_crackle_filter += (crackle - _gnd_crackle_filter) * 0.20f;

        float starve_buzz = buzz + (buzz * buzz - 0.5f) * starve * 0.50f;
        float gnd_hum_audio = (starve_buzz * 0.90f + _gnd_crackle_filter * 0.10f) * _smooth_gnd_hum;

        float touched_signal_nodes = _pressures_smooth[NodeAmp] + _pressures_smooth[NodeFlt] + _pressures_smooth[NodeOsc] + _pressures_smooth[NodeDly] + _pressures_smooth[NodeIn];
        float body_antenna_mult = 1.0f + std::min(0.5f, touched_signal_nodes * 0.5f);

        total_ground_hum = gnd_hum_audio * body_antenna_mult * _shunt_gnd_smooth[NodeAmp];
    }

    // 4. Preamp Stage
    inline void ProcessPreamp(float ext_l, float ext_r, const float s_inj_amp[2], const float s_inj_in[2], float s_osc, float s_clk, float total_ground_hum, float v_rail_eff, float starve, float standalone_p_amp, float& amp_out_l, float& amp_out_r) {
        float u = _v_dc_smooth[NodeAmp];
        // Input volume: full mute below 0.04 for dead-quiet silent floor
        float in_vol = (u < 0.04f) ? 0.0f : std::clamp((u - 0.04f) / 0.96f, 0.0f, 1.0f);
        in_vol = in_vol * in_vol; // Audio taper
        _amp_in_vol = in_vol;

        float pluck_sig = 0.0f;
        if (_amp_pluck > 0.001f) {
            pluck_sig = _amp_pluck;
            _amp_pluck *= 0.995f;
        }

        float mb_sig = 0.0f;
        if (v_rail_eff < 0.35f) {
            float starve_depth = (0.35f - v_rail_eff) / 0.35f;
            float mb_rate = 2.5f + 5.5f * (1.0f - starve_depth);
            _motorboat_phase += mb_rate * kTwoPi / _sample_rate;
            if (_motorboat_phase >= kTwoPi) _motorboat_phase -= kTwoPi;
            float mb_pulse = (std::sin(_motorboat_phase) > 0.65f) ? 0.8f : -0.15f;
            mb_sig = 0.040f * starve_depth * mb_pulse * in_vol;
        }

        float bias_drift = 0.18f * (starve * starve);
        float deadzone = 0.07f * (starve * starve);
        float touch_drive = standalone_p_amp * 3.0f;
        // Pushes drive up to 7.5x (and up to 10.5x with standalone Pad 8 pressure)
        float drive = (0.35f + 7.2f * in_vol + touch_drive) * (1.0f + 1.4f * starve);
        float bias_null = FastTanh(bias_drift * drive);

        // Loudness compensation: scales output down as drive increases so it doesn't get painfully loud
        float comp = 1.35f / (1.0f + 0.44f * drive + 0.08f * drive * drive);
        float out_gain = comp * (1.0f - 0.25f * starve) * _shunt_gnd_smooth[NodeAmp];

        // OSC <-> AMP bridge (P03 <-> P08): smooth, touch-sensitive oscillator volume swell into preamp (zero zipper noise)
        float osc_amp_inject = s_osc * _g_osc_amp_smooth * 0.85f;

        auto process_channel = [&](float ext_in, float inj_amp, float inj_in, DcBlocker& dc_block) -> float {
            float amp_in = ext_in * in_vol * 0.40f + inj_in * 0.85f + inj_amp * 0.90f + osc_amp_inject + total_ground_hum * in_vol + pluck_sig + mb_sig;
            float biased_in = amp_in + bias_drift;
            float gated_in = biased_in;
            if (std::abs(biased_in) < deadzone) {
                float sgn = (biased_in >= 0.0f) ? 1.0f : -1.0f;
                float mag = std::abs(biased_in);
                gated_in = sgn * (mag * mag) / std::max(0.001f, deadzone);
            }
            float clipped = FastTanh(gated_in * drive);
            float out = (clipped - bias_null) * out_gain;
            return dc_block.Process(out);
        };

        amp_out_l = process_channel(ext_l * _shunt_gnd_smooth[NodeIn], s_inj_amp[0], s_inj_in[0], _dc_block_amp_l);
        amp_out_r = process_channel(ext_r * _shunt_gnd_smooth[NodeOut], s_inj_amp[1], s_inj_in[1], _dc_block_amp_r);
    }

    // 5. Filter Stage
    inline void ProcessFilter(float amp_out_l, float amp_out_r, const float s_inj_flt[2], float v_rail_eff, float starve, float standalone_p_flt, float& flt_out_l, float& flt_out_r) {
        float flt_air = PadAirHover(NodeFlt);
        float flt_dc = _v_dc_smooth[NodeFlt] + 0.15f * standalone_p_flt + flt_air * 0.12f;

        // Cutoff scaling: remains full and open down to 0.50V, then gently sags under deep starve
        float flt_v_scale;
        if (v_rail_eff < 0.50f) {
            float s = v_rail_eff / 0.50f;
            flt_v_scale = 0.25f + 0.75f * (s * s);
        } else {
            flt_v_scale = 0.88f + 0.12f * ((v_rail_eff - 0.50f) / 0.50f);
        }

        float inj_flt_avg = (s_inj_flt[0] + s_inj_flt[1]) * 0.5f;
        float flt_freq = (35.0f * FastPow2(flt_dc * 8.6f)) * flt_v_scale;

        // P03 (OSC) <-> P04 (FLT) Bending:
        float g_osc_flt = _bridge_g[NodeOsc][NodeFlt];
        if (g_osc_flt > 0.015f && _osc_mode == OSC_SAW) {
            // Saw Ramp: unipolar upward envelope sweep sweeping cutoff up by up to +4.5 octaves!
            float saw_flt_ramp = _saw_ramp * 4.5f * g_osc_flt;
            flt_freq *= FastPow2(saw_flt_ramp);
        } else if (inj_flt_avg != 0.0f) {
            flt_freq *= FastPow2(inj_flt_avg * 1.5f);
        }
        flt_freq = std::clamp(flt_freq, 25.0f, 19000.0f);

        // Starve overdrive into filter input only kicks in below 0.45V
        float starve_drive = std::max(0.0f, starve - 0.55f) * 0.70f;
        float flt_drive = 1.0f + _bridge_g[NodeAmp][NodeFlt] * 2.2f + starve_drive;
        float flt_in_l = (amp_out_l * flt_drive + s_inj_flt[0] * 0.70f);
        float flt_in_r = (amp_out_r * flt_drive + s_inj_flt[1] * 0.70f);

        // P03 (OSC in Saw) <-> P04 (FLT): sharp saw flyback edge pings the resonant filter core!
        if (g_osc_flt > 0.015f && _osc_mode == OSC_SAW && _saw_reset_pulse > 0.0f) {
            float saw_ping = _saw_reset_pulse * g_osc_flt * 0.85f;
            _flt_s1_l += saw_ping;
            _flt_s1_r += saw_ping;
            flt_in_l += saw_ping * 1.5f;
            flt_in_r += saw_ping * 1.5f;
            _flt_ping_env = std::max(_flt_ping_env, g_osc_flt * 0.95f);
        }

        // Ping impulse injection & resonance decay envelope
        if (_flt_ping > 0.001f) {
            _flt_s1_l += _flt_ping;
            _flt_s1_r += _flt_ping;
            flt_in_l += _flt_ping * 1.5f;
            flt_in_r += _flt_ping * 1.5f;
            _flt_ping_env = 1.0f;
            _flt_ping = 0.0f;
        } else {
            _flt_ping_env *= 0.9997f; // ~250ms smooth acoustic ping ring tail
        }

        // Resonance calculation:
        // Base resonance: juicy, bubbly Q ~ 2.8 at rest (R ~ 0.18) so cutoff sweeps bubble and sing!
        // Standalone Pad 4 touch swells into screaming howl (R down to 0.02)
        // Ping envelope ensures striking Pad 4 rings out like a water droplet / resonant bell
        float res_raw = 0.62f + standalone_p_flt * 0.35f + _flt_ping_env * 0.32f + _bridge_g[NodeFlt][NodeFbk] * 0.45f;
        res_raw *= (1.0f - _bridge_g[NodeFlt][NodeGnd] * 0.95f);
        res_raw = std::clamp(res_raw, 0.0f, 0.98f);

        // Starve Self-Resonance Bending Paths:
        // 1. VCC <-> FLT (P02 <-> P04): injecting starved rail instability directly into filter core!
        float vcc_flt_bend = _bridge_g[NodeVcc][NodeFlt];
        // 2. FLT <-> FBK (P04 <-> P07) feedback runaway
        float flt_fbk_bend = _bridge_g[NodeFlt][NodeFbk];
        // 3. Pad 4 touch pressure under starve
        float touch_flt = standalone_p_flt;

        // Combined intentional bend intensity:
        float bend_howl = std::max({vcc_flt_bend * 1.5f, flt_fbk_bend * 1.3f, touch_flt * 1.2f});

        // Transconductance choke: default behavior stabilizes filter so starving never runs away
        float starve_choke = std::clamp((v_rail_eff - 0.15f) / 0.45f, 0.0f, 1.0f);
        // BUT when the bending path is engaged, starve choke is defeated!
        starve_choke = starve_choke + (1.0f - starve_choke) * std::min(1.0f, bend_howl * 1.4f);

        float res_eff = res_raw * starve_choke;

        // Damping factor R: R = 1 / (2Q)
        // At rest (res_eff = 0.62): R = 0.45 * (1 - 0.62) + 0.02 = 0.191 -> Q = 2.62 (juicy bubble & vocal sweep!)
        // On ping (res_eff = 0.94): R = 0.45 * 0.06 + 0.02 = 0.047 -> Q = 10.6 (singing bell / water ping!)
        // On Pad 4 hard press (res_eff = 0.97): R = 0.45 * 0.03 + 0.02 = 0.033 -> Q = 15.0 (screaming howl!)
        float R = 0.45f * (1.0f - std::clamp(res_eff, 0.0f, 0.98f)) + 0.02f;

        // Default low-voltage damping reinforcement keeps the filter stable and musical
        float starve_damping = (v_rail_eff < 0.50f) ? (0.50f - v_rail_eff) * 1.6f : 0.0f;
        // The bend cancels the extra damping and pushes it into screaming starve distortion!
        starve_damping *= (1.0f - std::min(1.0f, bend_howl * 1.5f));
        R += starve_damping;

        if (bend_howl > 0.08f && starve > 0.35f) {
            // Player is actively bending the starved filter:
            // Unleashes the savage screaming self-resonant breakdown distortion!
            R = std::max(0.015f, R - bend_howl * 0.45f);
            flt_in_l += starve * bend_howl * 0.35f;
            flt_in_r += starve * bend_howl * 0.35f;
        }

        // Bilinear transform pre-warping parameter g = tan(pi * f_c / (2 * f_s)) with 2x oversampling
        // Unconditionally stable from 20 Hz to 19 kHz: zero undersampling hash or high-cutoff ringing
        float g = std::tan(kPi * flt_freq / (2.0f * _sample_rate));
        g = std::clamp(g, 0.001f, 1.25f);
        float h = 1.0f / (1.0f + 2.0f * R * g + g * g);

        // Headroom stays high until deep starve (below 0.45V)
        float flt_headroom = (v_rail_eff > 0.45f) ? (0.80f + 0.20f * ((v_rail_eff - 0.45f) / 0.55f))
                                                  : (0.25f + 0.55f * (v_rail_eff / 0.45f));

        // 2x oversampled TPT SVF trapezoidal integration for both Left and Right channels
        for (int step = 0; step < 2; ++step) {
            // Left
            float high_l = (flt_in_l - (2.0f * R + g) * _flt_s1_l - _flt_s2_l) * h;
            high_l = SaturateFilter(high_l, flt_headroom, starve);
            float v1_l = g * high_l;
            float band_l = v1_l + _flt_s1_l;
            _flt_s1_l = 2.0f * v1_l + _flt_s1_l;
            float v2_l = g * band_l;
            _flt_low_l = v2_l + _flt_s2_l;
            _flt_s2_l = 2.0f * v2_l + _flt_s2_l;

            // Right
            float high_r = (flt_in_r - (2.0f * R + g) * _flt_s1_r - _flt_s2_r) * h;
            high_r = SaturateFilter(high_r, flt_headroom, starve);
            float v1_r = g * high_r;
            float band_r = v1_r + _flt_s1_r;
            _flt_s1_r = 2.0f * v1_r + _flt_s1_r;
            float v2_r = g * band_r;
            _flt_low_r = v2_r + _flt_s2_r;
            _flt_s2_r = 2.0f * v2_r + _flt_s2_r;
        }

        if (std::isnan(_flt_low_l) || std::isinf(_flt_low_l) || std::isnan(_flt_s1_l) || std::isinf(_flt_s1_l)) {
            _flt_low_l = 0.0f;
            _flt_s1_l = 0.0f;
            _flt_s2_l = 0.0f;
        }
        if (std::isnan(_flt_low_r) || std::isinf(_flt_low_r) || std::isnan(_flt_s1_r) || std::isinf(_flt_s1_r)) {
            _flt_low_r = 0.0f;
            _flt_s1_r = 0.0f;
            _flt_s2_r = 0.0f;
        }

        _flt_low_l = SaturateFilter(_flt_low_l, flt_headroom, starve);
        _flt_low_r = SaturateFilter(_flt_low_r, flt_headroom, starve);

        float flt_makeup = std::min(1.25f, 1.0f / std::sqrt(0.35f + 0.65f * v_rail_eff));
        _flt_low_l = _flt_low_l * flt_makeup * _shunt_gnd_smooth[NodeFlt];
        _flt_low_r = _flt_low_r * flt_makeup * _shunt_gnd_smooth[NodeFlt];

        _flt_low_l = _dc_block_flt_l.Process(_flt_low_l);
        _flt_low_r = _dc_block_flt_r.Process(_flt_low_r);

        flt_out_l = _flt_low_l;
        flt_out_r = _flt_low_r;
    }

    // 6. Delay Stage & Digital Memory Bends
    inline void ProcessDelay(float flt_low_l, float flt_low_r, float amp_out_l, float amp_out_r, float s_osc, float standalone_p_clk, float standalone_p_fbk, float v_rail_eff, float starve, float& dly_dac_l, float& dly_dac_r) {
        float u_raw = std::clamp(_v_dc_smooth[NodeDly], 0.0f, 1.0f);
        if (!_delay_time_initialized || std::abs(u_raw - _dly_knob_held) > 0.005f) {
            _dly_knob_held = u_raw;
        }
        float u = _dly_knob_held;

        float base_time;
        if (u < 0.20f) {
            // Zone 1 (0% to 20%): Sub-millisecond Karplus-Strong string tuning & comb resonance
            // 0.60ms at 0% (29 samples / 1667 Hz) -> 25ms at 20% (1200 samples / 40 Hz)
            float k = u / 0.20f;
            base_time = 0.0006f * FastPow2(k * 5.3808218f);
        } else {
            // Zone 2 (20% to 100%): Echo, Ambient Space Delay, and Looper
            // 25ms at 20% -> 106ms at 40% -> 312ms at 55% -> 920ms at 70% -> 2.7s at 85% -> 8.0s at 100%
            float w = (u - 0.20f) / 0.80f;
            base_time = 0.025f * FastPow2(w * 8.321928f);
        }
        base_time = std::clamp(base_time, 0.0006f, 8.0f);

        // PT2399 Digital Variclock Model:
        // S36 Clock Fader directly controls delay tape speed and converter clock:
        // Top of fader (1.0) = 48 kHz (pristine clean, 1.0x normal pitch)
        // Sliding down smoothly lowers speed down to 0.20x (~2.3 octaves down, deep & crunchy lo-fi aliasing)
        float clk_dc = std::clamp(_v_dc_smooth[NodeClk], 0.0f, 1.0f);
        float clk_fader_speed = 0.20f + 0.80f * (clk_dc * clk_dc);
        float pad_clk_drag = 1.0f - standalone_p_clk * 0.20f;

        // Ground shunt: grounding CLK (P00 <-> P05) stalls the delay motor completely (tape stop)
        float clk_shunt = _shunt_gnd_smooth[NodeClk];

        // Overclocking: P00 (CLK) <-> P02 (VCC) boosts speed up to 1.35x
        float g_vcc_clk = _bridge_g[NodeVcc][NodeClk];
        float clk_overclock = 1.0f + g_vcc_clk * 0.35f;

        // Fast clock bridge (P00 <-> P06 when >= 35 Hz) drives digital clock decimation
        float g_clk_dly = _bridge_g[NodeClk][NodeDly];
        bool clk_is_slow = (_clk_rate < 35.0f);
        float fast_clk_decim = (!clk_is_slow && g_clk_dly > 0.015f) ? (g_clk_dly * 0.45f) : 0.0f;

        // OSC <-> CLK (P03 <-> P00) bridge: audio-rate FM on the delay converter clock!
        // Directly frequency-modulates the delay clock rate, creating genuine PT2399 heterodyne bell sidebands
        float g_osc_clk = _bridge_g[NodeOsc][NodeClk];
        float osc_clk_fm = g_osc_clk * s_osc * 0.35f;

        // Power rail sag (brownout / starve) also drags delay clock motor
        float clk_rail_sag = 0.50f + 0.50f * (v_rail_eff * v_rail_eff);

        float eff_clk_speed = (clk_fader_speed * pad_clk_drag * clk_overclock - fast_clk_decim + osc_clk_fm) * clk_shunt * clk_rail_sag;
        eff_clk_speed = std::clamp(eff_clk_speed, 0.0f, 1.45f);

        // Delay Time (S33) sets the buffer size / readhead distance directly.
        // In Karplus zone (< 20%): responsive one-pole filter for instant string pitch tracking.
        // In Echo / Looper zone (>= 20%): dual-head raised-cosine crossfade completely eliminates
        // Doppler tape pitch dive, providing instant buffer length changes with rock-solid pitch!
        float target_samples = std::clamp(base_time * _sample_rate, 28.0f, static_cast<float>(_max_delay - 2));
        if (!_delay_time_initialized) {
            _smooth_delay_samples = target_samples;
            _delay_time_curr = target_samples;
            _delay_time_prev = target_samples;
            _delay_xfade = 1.0f;
            _delay_time_initialized = true;
        } else if (u < 0.20f) {
            _smooth_delay_samples += 0.025f * (target_samples - _smooth_delay_samples);
            _delay_time_curr = _smooth_delay_samples;
            _delay_time_prev = _smooth_delay_samples;
            _delay_xfade = 1.0f;
        } else {
            if (_delay_xfade >= 1.0f) {
                if (std::abs(target_samples - _delay_time_curr) > 36.0f) {
                    _delay_time_prev = _delay_time_curr;
                    _delay_time_curr = target_samples;
                    _delay_xfade = 0.0f;
                }
            } else {
                _delay_xfade += 0.00085f; // ~25ms crossfade at 48 kHz
                if (_delay_xfade >= 1.0f) {
                    _delay_xfade = 1.0f;
                    if (std::abs(target_samples - _delay_time_curr) > 36.0f) {
                        _delay_time_prev = _delay_time_curr;
                        _delay_time_curr = target_samples;
                        _delay_xfade = 0.0f;
                    }
                }
            }
            _smooth_delay_samples = _delay_time_curr;
        }

        // Manage Freeze state:
        // Freezing captures the delay buffer loop without overwriting it.
        // The Delay Time knob dynamically expands or contracts the loop window across the 16-second recorded history!
        // The Clock Fader controls the playback speed and pitch of the frozen loop!
        if (_is_frozen) {
            if (!_was_frozen) {
                _was_frozen = true;
                _freeze_anchor = _delay_write_pos;
                _freeze_len = std::clamp(_smooth_delay_samples, 28.0f, static_cast<float>(_max_delay - 2));
                _freeze_phase = 0.0f;
            }
            // Dynamically track the Delay Time knob during Freeze!
            float target_freeze_len = std::clamp(_smooth_delay_samples, 28.0f, static_cast<float>(_max_delay - 2));
            _freeze_len += 0.008f * (target_freeze_len - _freeze_len);

            if (_freeze_len > 1.0f) {
                _freeze_phase += eff_clk_speed;
                if (_freeze_phase >= _freeze_len) {
                    _freeze_phase = fmodf(_freeze_phase, _freeze_len);
                } else if (_freeze_phase < 0.0f) {
                    _freeze_phase = fmodf(_freeze_phase, _freeze_len) + _freeze_len;
                }
            } else {
                _freeze_phase = 0.0f;
            }
        } else {
            _was_frozen = false;
        }

        // Base read delay samples (drum-tight identical timing across both channels)
        float base_read_samples = _is_frozen ? (_freeze_len - _freeze_phase) : _smooth_delay_samples;
        float read_pos_l = base_read_samples;
        float read_pos_r = base_read_samples;

        // PT2399 Variclock FM Modulation (P03 OSC <-> P00 CLK):
        // Audio-rate FM stereo widening on readheads
        if (g_osc_clk > 0.015f) {
            float clk_fm_depth = std::clamp(base_read_samples * 0.08f, 1.0f, 250.0f) * g_osc_clk;
            read_pos_l += s_osc * clk_fm_depth;
            read_pos_r -= s_osc * clk_fm_depth;
        }

        // 1. P03 (OSC) <-> P06 (DLY): 3-Mode Digital Memory Mangle
        float g_osc_dly = _bridge_g[NodeOsc][NodeDly];
        if (g_osc_dly > 0.015f) {
            if (_osc_mode == OSC_SIN) {
                // Mode 1: Sine - Quadrature stereo chorus / metallic audio-rate FM bells
                float fm_depth = (1.5f + 8.0f * u) * g_osc_dly;
                read_pos_l += s_osc * fm_depth;
                read_pos_r -= s_osc * fm_depth;
            } else if (_osc_mode == OSC_SAW) {
                // Mode 2: Saw - Linear ramp pitch sweeps / barberpole frequency shifts
                float ramp_span = (u < 0.20f) ? (base_read_samples * 0.40f) : std::min(4800.0f, base_read_samples * 0.25f);
                float ramp_l = (s_osc * 0.5f + 0.5f) * ramp_span * g_osc_dly;
                float ramp_r = (1.0f - (s_osc * 0.5f + 0.5f)) * ramp_span * g_osc_dly;
                read_pos_l = std::max(1.0f, read_pos_l - ramp_l);
                read_pos_r = std::max(1.0f, read_pos_r - ramp_r);
            } else {
                // Mode 3: Random - S&H Stepped Pitch Arpeggiator (Karplus) or Buffer Slice Scatter (Echo)
                if (u < 0.20f) {
                    read_pos_l = base_read_samples * (1.0f + (_sh_step_ratio - 1.0f) * g_osc_dly);
                    read_pos_r = base_read_samples * (1.0f + (_sh_step_ratio - 1.0f) * g_osc_dly * 0.98f);
                } else {
                    float scatter = _rnd_held * std::min(24000.0f, base_read_samples * 0.5f) * g_osc_dly;
                    read_pos_l = std::max(1.0f, read_pos_l + scatter);
                    read_pos_r = std::max(1.0f, read_pos_r - scatter * 0.85f);
                }
            }
        }

        // 2. P00 (CLK) <-> P06 (DLY): Rhythmic Glitch Stutter (< 35 Hz)
        if (g_clk_dly > 0.015f && clk_is_slow) {
            float stutter_len = std::clamp(base_read_samples * 0.15f, 300.0f, 6000.0f);
            bool stutter_active_l = (_clk_phase < 0.28f);
            bool stutter_active_r = (_clk_phase >= 0.50f && _clk_phase < 0.78f);

            if (stutter_active_l) {
                _stutter_pos_l += 1.0f;
                if (_stutter_pos_l >= stutter_len) _stutter_pos_l = 0.0f;
                read_pos_l = std::max(1.0f, read_pos_l - _stutter_pos_l * g_clk_dly);
            } else {
                _stutter_pos_l = 0.0f;
            }

            if (stutter_active_r) {
                _stutter_pos_r += 1.0f;
                if (_stutter_pos_r >= stutter_len) _stutter_pos_r = 0.0f;
                read_pos_r = std::max(1.0f, read_pos_r - _stutter_pos_r * g_clk_dly);
            } else {
                _stutter_pos_r = 0.0f;
            }
        }

        // 3. P02 (VCC) <-> P06 (DLY) & Lower-Third Voltage Starve: Rhythmic CD Skips & Address-Bus Bends
        // Musical windowed micro-repeats and harmonic power-of-two address jumps without harsh static crackle.
        float g_vcc_dly = _bridge_g[NodeVcc][NodeDly];
        float starve_slip = (v_rail_eff < 0.33f) ? ((0.33f - v_rail_eff) / 0.31f) : 0.0f;
        float glitch_intensity = std::max(g_vcc_dly, starve_slip);
        float mod_l = read_pos_l - base_read_samples;
        float mod_r = read_pos_r - base_read_samples;

        float raw_dly_l;
        float raw_dly_r;
        if (!_is_frozen && u >= 0.20f && _delay_xfade < 1.0f) {
            float fade_win = 0.5f * (1.0f - std::cos(_delay_xfade * kPi));
            float prev_pos_l = std::clamp(_delay_time_prev + mod_l, 1.0f, static_cast<float>(_max_delay - 2));
            float curr_pos_l = std::clamp(_delay_time_curr + mod_l, 1.0f, static_cast<float>(_max_delay - 2));
            raw_dly_l = ReadDelay_L(prev_pos_l) * (1.0f - fade_win) + ReadDelay_L(curr_pos_l) * fade_win;

            float prev_pos_r = std::clamp(_delay_time_prev + mod_r, 1.0f, static_cast<float>(_max_delay - 2));
            float curr_pos_r = std::clamp(_delay_time_curr + mod_r, 1.0f, static_cast<float>(_max_delay - 2));
            raw_dly_r = ReadDelay_R(prev_pos_r) * (1.0f - fade_win) + ReadDelay_R(curr_pos_r) * fade_win;
        } else {
            raw_dly_l = ReadDelay_L(read_pos_l);
            raw_dly_r = ReadDelay_R(read_pos_r);
        }

        if (glitch_intensity > 0.015f) {
            _slip_timer++;
            if (_slip_timer >= _slip_len) {
                _slip_timer = 0;
                // Rhythmic grain lengths: 25ms (1200 samples) to 120ms (5760 samples), subdividing with pressure
                uint32_t base_len = 1200 + (_lfsr % 4560);
                _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
                _slip_len = static_cast<uint16_t>(base_len / (1.0f + glitch_intensity * 1.5f));
                if (_slip_len < 240) _slip_len = 240;

                // Power-of-two address bus jump (512, 1024, 2048, 4096, 8192, 16384 samples)
                // Creates harmonic octave repeats and rhythmic sub-delays
                static const float kAddressJumps[6] = {512.0f, 1024.0f, 2048.0f, 4096.0f, 8192.0f, 16384.0f};
                uint8_t jump_idx = (_lfsr >> 3) % 6;
                _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
                _slip_offset_l = kAddressJumps[jump_idx];
                _slip_offset_r = kAddressJumps[(jump_idx + 1) % 6];
            }

            // Smooth 2.5ms (120 samples) anti-click raised-cosine window on grain edges
            float grain_win = 1.0f;
            constexpr uint16_t kFadeLen = 120;
            if (_slip_timer < kFadeLen) {
                grain_win = static_cast<float>(_slip_timer) / static_cast<float>(kFadeLen);
            } else if (_slip_len - _slip_timer < kFadeLen) {
                grain_win = static_cast<float>(_slip_len - _slip_timer) / static_cast<float>(kFadeLen);
            }
            grain_win = 0.5f * (1.0f - std::cos(grain_win * kPi)); // raised-cosine smoothing

            // Read the slipped address grain
            float micro_pos = static_cast<float>(_slip_timer);
            float slip_read_pos_l = std::clamp(read_pos_l - micro_pos + _slip_offset_l, 1.0f, static_cast<float>(_max_delay - 2));
            float slip_read_pos_r = std::clamp(read_pos_r - micro_pos + _slip_offset_r, 1.0f, static_cast<float>(_max_delay - 2));

            float slipped_l = ReadDelay_L(slip_read_pos_l);
            float slipped_r = ReadDelay_R(slip_read_pos_r);

            // Crossfade smoothly into the micro-stutter grain without DC clicks or bacon crackle
            float g_blend = glitch_intensity * grain_win;
            raw_dly_l = raw_dly_l * (1.0f - g_blend) + slipped_l * g_blend;
            raw_dly_r = raw_dly_r * (1.0f - g_blend) + slipped_r * g_blend;
        }

        // 3. P01 (RST) <-> P06 (DLY): 90° Quadrature Stereo Reverse Pitch Cloud
        float g_rst_dly = _bridge_g[NodeRst][NodeDly];
        if (g_rst_dly > 0.015f) {
            float rev_window = std::clamp(base_read_samples * 0.85f, 1200.0f, 48000.0f);
            float inv_window = 1.0f / rev_window;

            _rev_phase_l += inv_window;
            if (_rev_phase_l >= 1.0f) _rev_phase_l -= 1.0f;

            // Head 1 & Head 2 (180 deg out of phase) for Left
            float p1_l = _rev_phase_l;
            float p2_l = p1_l + 0.5f;
            if (p2_l >= 1.0f) p2_l -= 1.0f;

            float d1_l = read_pos_l + (1.0f - p1_l) * rev_window;
            float d2_l = read_pos_l + (1.0f - p2_l) * rev_window;
            float s1_l = ReadDelay_L(d1_l);
            float s2_l = ReadDelay_L(d2_l);
            float w1_l = 4.0f * p1_l * (1.0f - p1_l);
            float w2_l = 4.0f * p2_l * (1.0f - p2_l);
            float rev_read_l = (s1_l * w1_l + s2_l * w2_l) / (w1_l + w2_l + 1e-6f);

            // Head 1 & Head 2 for Right (90 deg quadrature phase offset)
            float p1_r = _rev_phase_l + 0.25f;
            if (p1_r >= 1.0f) p1_r -= 1.0f;
            float p2_r = p1_r + 0.5f;
            if (p2_r >= 1.0f) p2_r -= 1.0f;

            float d1_r = read_pos_r + (1.0f - p1_r) * rev_window;
            float d2_r = read_pos_r + (1.0f - p2_r) * rev_window;
            float s1_r = ReadDelay_R(d1_r);
            float s2_r = ReadDelay_R(d2_r);
            float w1_r = 4.0f * p1_r * (1.0f - p1_r);
            float w2_r = 4.0f * p2_r * (1.0f - p2_r);
            float rev_read_r = (s1_r * w1_r + s2_r * w2_r) / (w1_r + w2_r + 1e-6f);

            raw_dly_l += g_rst_dly * (rev_read_l - raw_dly_l);
            raw_dly_r += g_rst_dly * (rev_read_r - raw_dly_r);
        }

        // Loop damping filter (controlled by S32 Filter knob and rail voltage)
        float string_damp = daisysp::fclamp((0.25f + 0.72f * _v_dc_smooth[NodeFlt]) * (0.65f + 0.35f * v_rail_eff), 0.05f, 0.98f);
        _damp_l += string_damp * (raw_dly_l - _damp_l);
        _damp_r += string_damp * (raw_dly_r - _damp_r);

        // 4. P02 (VCC) <-> P06 (DLY): Digital DAC ladder starve & bit crushing
        // (g_vcc_dly and glitch_intensity resolved above)

        // Lo-Fi processor blend:
        // Comb/phasing zone (< 10ms): clean floating point unless bent or clock downsampled
        // Echo zone (> 30ms): digital converter model
        float lofi_blend = daisysp::fclamp((base_time - 0.010f) / 0.020f, 0.0f, 1.0f);
        if (eff_clk_speed < 0.96f || g_vcc_dly > 0.02f || starve > 0.10f || std::abs(osc_clk_fm) > 0.01f) {
            lofi_blend = 1.0f;
        }

        float dly_proc_l, dly_proc_r;
        if (lofi_blend < 0.001f) {
            dly_proc_l = _damp_l;
            dly_proc_r = _damp_r;
        } else {
            _lofi_l.Prepare(base_time, eff_clk_speed, v_rail_eff, _sample_rate);
            float proc_l = _lofi_l.Process(_damp_l);

            _lofi_r.Prepare(base_time, eff_clk_speed, v_rail_eff, _sample_rate);
            float proc_r = _lofi_r.Process(_damp_r);

            // DAC quantization & R-2R ladder starve
            if (eff_clk_speed >= 0.96f && v_rail_eff >= 0.95f && g_vcc_dly < 0.02f) {
                dly_proc_l = _damp_l + lofi_blend * (proc_l - _damp_l);
                dly_proc_r = _damp_r + lofi_blend * (proc_r - _damp_r);
            } else {
                float bit_depth = 16.0f - g_vcc_dly * 4.0f - starve * 8.0f - (1.0f - eff_clk_speed) * 6.0f;
                bit_depth = std::clamp(bit_depth, 8.0f, 16.0f);
                float levels = FastPow2(bit_depth);
                float inv_levels = 1.0f / levels;
                float dac_dnl = 0.05f + g_vcc_dly * 0.15f + starve * 0.35f + (1.0f - eff_clk_speed) * 0.15f;

                float out_lofi_l = QuantizeDac(proc_l, levels, inv_levels, dac_dnl);
                float out_lofi_r = QuantizeDac(proc_r, levels, inv_levels, dac_dnl);

                dly_proc_l = _damp_l + lofi_blend * (out_lofi_l - _damp_l);
                dly_proc_r = _damp_r + lofi_blend * (out_lofi_r - _damp_r);
            }
        }

        dly_dac_l = dly_proc_l * _shunt_gnd_smooth[NodeDly];
        dly_dac_r = dly_proc_r * _shunt_gnd_smooth[NodeDly];

        // Stereo feedback calculation:
        // Allows infinite sustain (= 1.00) at top of feedback fader for sound-on-sound looping
        // Bridging P04 (FLT) <-> P07 (FBK) drives feedback into howling analog dub oscillation
        float max_fb = (base_time < 0.025f) ? 0.992f : 1.10f;
        float base_fb = _v_dc_smooth[NodeFbk] * (max_fb + 0.05f * starve);
        base_fb += standalone_p_fbk * 0.30f;
        float flt_fb_boost = _bridge_g[NodeFlt][NodeFbk] * 0.65f;
        float eff_fb = (base_fb + flt_fb_boost) * _shunt_gnd_smooth[NodeFbk];

        // Karplus-Strong string resonance: elevated feedback when exciting string with saw
        if (u < 0.20f && g_osc_dly > 0.015f && _osc_mode == OSC_SAW) {
            float karplus_fb = 0.965f + 0.026f * g_osc_dly;
            eff_fb = std::max(eff_fb, karplus_fb * g_osc_dly);
        }
        eff_fb = std::clamp(eff_fb, 0.0f, 1.40f);

        // Drum-tight rhythmic ping-pong cross-coupling:
        // Left and Right share identical delay time T (base_read_samples),
        // guaranteeing that percussive transients land 100% on the beat with zero flamming.
        // Nominal unbridged mode provides a tight, musical cross-coupling (68% direct, 32% cross).
        // Bridging P06 (DLY) <-> P07 (FBK) elevates cross-coupling to 100% full ping-pong rotation!
        float g_dly_fbk = _bridge_g[NodeDly][NodeFbk];
        float cross = 0.32f + 0.68f * g_dly_fbk;
        float direct = 1.0f - cross;
        float fb_tot_l = (_damp_l * direct + _damp_r * cross) * eff_fb;
        float fb_tot_r = (_damp_r * direct + _damp_l * cross) * eff_fb;

        // P04 (FLT) <-> P07 (FBK) Dub Delay Loop:
        // Routes the delay feedback path through the resonant SVF filter!
        float g_flt_fbk = _bridge_g[NodeFlt][NodeFbk];
        if (g_flt_fbk > 0.015f) {
            fb_tot_l = fb_tot_l * (1.0f - g_flt_fbk) + flt_low_l * eff_fb * g_flt_fbk * 1.05f;
            fb_tot_r = fb_tot_r * (1.0f - g_flt_fbk) + flt_low_r * eff_fb * g_flt_fbk * 1.05f;
        }

        // P03 (OSC) <-> P07 (FBK): Chaotic Delayed Self-FM & Recirculating Modulator
        // Closed-loop dynamical system: the probe oscillator cross-modulates the delay feedback loop,
        // while the delay output audio simultaneously frequency-modulates the probe oscillator via _inj_osc!
        // Generates morphing alien sidebands, multiphonic whistling, and bubbling self-FM textures.
        float g_osc_fbk = _bridge_g[NodeOsc][NodeFbk];
        if (g_osc_fbk > 0.015f) {
            float mod_l = 1.0f + s_osc * 0.95f * g_osc_fbk;
            float mod_r = 1.0f - s_osc * 0.95f * g_osc_fbk; // quadrature phase spread
            fb_tot_l = fb_tot_l * mod_l + s_osc * 0.40f * g_osc_fbk;
            fb_tot_r = fb_tot_r * mod_r - s_osc * 0.40f * g_osc_fbk;
        }

        // Bridge inputs directly into delay buffer
        float amp_to_dly_l = _bridge_g[NodeAmp][NodeDly] * amp_out_l * 1.5f;
        float amp_to_dly_r = _bridge_g[NodeAmp][NodeDly] * amp_out_r * 1.5f;
        float flt_to_dly_l = _bridge_g[NodeFlt][NodeDly] * flt_low_l * 1.2f;
        float flt_to_dly_r = _bridge_g[NodeFlt][NodeDly] * flt_low_r * 1.2f;

        // P03 (OSC) <-> P06 (DLY) buffer excitation:
        // In Karplus zone (< 25ms): saw flyback edge injects sharp string pluck excitation!
        // In Echo zone: injects rhythmic saw echoes into loop
        float osc_to_dly_l = 0.0f;
        float osc_to_dly_r = 0.0f;
        if (g_osc_dly > 0.015f) {
            if (_osc_mode == OSC_SAW) {
                if (u < 0.20f) {
                    float saw_pluck = _saw_reset_pulse * g_osc_dly * 0.92f;
                    float saw_continuous = s_osc * g_osc_dly * 0.22f;
                    osc_to_dly_l = saw_pluck + saw_continuous;
                    osc_to_dly_r = saw_pluck + saw_continuous;
                } else {
                    float saw_echo = (s_osc * 0.35f + _saw_reset_pulse * 0.50f) * g_osc_dly;
                    osc_to_dly_l = saw_echo;
                    osc_to_dly_r = saw_echo;
                }
            } else {
                osc_to_dly_l = s_osc * g_osc_dly * 0.45f;
                osc_to_dly_r = s_osc * g_osc_dly * 0.45f;
            }
        }


        // Non-destructive Freeze & Low-Voltage Memory Glitch:
        // When _is_frozen is true OR under deep starve / VCC<->DLY glitch OR during RST<->GND clear,
        // write operations to SDRAM buffer are completely bypassed!
        // All bends operate live on readhead pointers and DAC, preserving the captured audio loop forever.
        bool delay_write_inhibited = _is_frozen || (glitch_intensity > 0.15f) || (_rst_gnd_hold > 0);
        if (!delay_write_inhibited) {
            float eff_fb_in_l = FastTanh(fb_tot_l);
            float eff_fb_in_r = FastTanh(fb_tot_r);
            float dly_in_l = flt_low_l + amp_to_dly_l + flt_to_dly_l + osc_to_dly_l + eff_fb_in_l;
            float dly_in_r = flt_low_r + amp_to_dly_r + flt_to_dly_r + osc_to_dly_r + eff_fb_in_r;
            dly_in_l = _dc_block_dly_l.Process(dly_in_l);
            dly_in_r = _dc_block_dly_r.Process(dly_in_r);
            WriteDelay_Stereo(dly_in_l, dly_in_r, eff_clk_speed);
        } else if (!_is_frozen) {
            AdvanceDelayWrite(eff_clk_speed);
        }

        _prev_dly_read_l = dly_dac_l;
        _prev_dly_read_r = dly_dac_r;
    }

    // 7. Mixer Stage
    inline void ProcessMixer(float flt_low_l, float flt_low_r, float dly_dac_l, float dly_dac_r, float ext_l, float ext_r, const float s_inj[12][2], float standalone_p_mix, float& out_l, float& out_r) {
        // Direct tap into master output (P11 OUT probe point)
        float out_direct_tap_l = s_inj[NodeOut][0] * 0.85f;
        float out_direct_tap_r = s_inj[NodeOut][1] * 0.85f;

        // Direct tap into master mix (P09 MIX probe point)
        float mix_direct_tap_l = s_inj[NodeMix][0] * 0.85f;
        float mix_direct_tap_r = s_inj[NodeMix][1] * 0.85f;

        // External inputs (scaled by input volume and shunted if P10 IN is grounded)
        float ext_l_gated = ext_l * _amp_in_vol * 0.25f * _shunt_gnd_smooth[NodeIn];
        float ext_r_gated = ext_r * _amp_in_vol * 0.25f * _shunt_gnd_smooth[NodeOut];

        float dry_l = (flt_low_l + ext_l_gated + mix_direct_tap_l + out_direct_tap_l) * _shunt_gnd_smooth[NodeAmp];
        float dry_r = (flt_low_r + ext_r_gated + mix_direct_tap_r + out_direct_tap_r) * _shunt_gnd_smooth[NodeAmp];

        float mix_wet = std::clamp(_v_dc_smooth[NodeMix] + standalone_p_mix * 0.40f, 0.0f, 1.0f);

        out_l = ((1.0f - mix_wet) * dry_l + mix_wet * dly_dac_l + (mix_direct_tap_l + out_direct_tap_l) * 0.45f) * _shunt_gnd_smooth[NodeMix] * _shunt_gnd_smooth[NodeOut] * 0.75f;
        out_r = ((1.0f - mix_wet) * dry_r + mix_wet * dly_dac_r + (mix_direct_tap_r + out_direct_tap_r) * 0.45f) * _shunt_gnd_smooth[NodeMix] * _shunt_gnd_smooth[NodeOut] * 0.75f;

        // P09 (MIX) <-> P04 (FLT) Master Output Filter Isolator:
        // When MIX is bridged to FLT, the master mixed audio (wet delay + dry mix) is routed through the Filter!
        float g_flt_mix = _bridge_g[NodeFlt][NodeMix];
        if (g_flt_mix > 0.015f) {
            float flt_master_l = flt_low_l * _shunt_gnd_smooth[NodeMix] * _shunt_gnd_smooth[NodeOut] * 0.85f;
            float flt_master_r = flt_low_r * _shunt_gnd_smooth[NodeMix] * _shunt_gnd_smooth[NodeOut] * 0.85f;
            out_l = out_l * (1.0f - g_flt_mix) + flt_master_l * g_flt_mix;
            out_r = out_r * (1.0f - g_flt_mix) + flt_master_r * g_flt_mix;
        }
    }

    inline float QuantizeDac(float in, float levels, float inv, float dnl) const {
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

    inline float SaturateFilter(float x, float headroom, float starve) const {
        // Asymmetric diode clipping: positive clips slightly earlier under starvation, but with zero DC shift at rest
        float h_pos = headroom * (1.0f - 0.20f * starve);
        float h_neg = headroom * (1.0f + 0.08f * starve);
        float sat;
        if (x >= 0.0f) {
            sat = h_pos * FastTanh(x / h_pos);
        } else {
            sat = h_neg * FastTanh(x / h_neg);
        }
        if (starve > 0.70f) {
            float deadzone = 0.04f * (starve - 0.70f) / 0.30f;
            if (std::abs(sat) < deadzone) {
                float sgn = (sat >= 0.0f) ? 1.0f : -1.0f;
                sat = sgn * (sat * sat) / deadzone;
            }
        }
        return sat;
    }

    inline float ReadDelay_L(float delay_samples) const {
        if (!_delay_buf_l || _max_delay == 0) return 0.0f;
        if (std::isnan(delay_samples) || delay_samples < 1.0f) delay_samples = 1.0f;
        else if (delay_samples >= static_cast<float>(_max_delay - 2)) delay_samples = static_cast<float>(_max_delay - 2);

        float max_d = static_cast<float>(_max_delay);
        float read_pos = _delay_write_pos - delay_samples;
        while (read_pos < 0.0f) read_pos += max_d;
        while (read_pos >= max_d) read_pos -= max_d;

        size_t idx0 = static_cast<size_t>(read_pos);
        if (idx0 >= _max_delay - 1) idx0 = _max_delay - 2;
        size_t idx1 = idx0 + 1;

        float frac = read_pos - static_cast<float>(idx0);
        return _delay_buf_l[idx0] + frac * (_delay_buf_l[idx1] - _delay_buf_l[idx0]);
    }

    inline float ReadDelay_R(float delay_samples) const {
        if (!_delay_buf_r || _max_delay == 0) return 0.0f;
        if (std::isnan(delay_samples) || delay_samples < 1.0f) delay_samples = 1.0f;
        else if (delay_samples >= static_cast<float>(_max_delay - 2)) delay_samples = static_cast<float>(_max_delay - 2);

        float max_d = static_cast<float>(_max_delay);
        float read_pos = _delay_write_pos - delay_samples;
        while (read_pos < 0.0f) read_pos += max_d;
        while (read_pos >= max_d) read_pos -= max_d;

        size_t idx0 = static_cast<size_t>(read_pos);
        if (idx0 >= _max_delay - 1) idx0 = _max_delay - 2;
        size_t idx1 = idx0 + 1;

        float frac = read_pos - static_cast<float>(idx0);
        return _delay_buf_r[idx0] + frac * (_delay_buf_r[idx1] - _delay_buf_r[idx0]);
    }

    inline void WriteDelay_Stereo(float sample_l, float sample_r, float speed) {
        if (!_delay_buf_l || !_delay_buf_r || _max_delay == 0) return;
        size_t idx = static_cast<size_t>(_delay_write_pos);
        if (idx >= _max_delay) idx = 0;
        _delay_buf_l[idx] = sample_l;
        _delay_buf_r[idx] = sample_r;
        _delay_write_pos += speed;
        if (_delay_write_pos >= static_cast<float>(_max_delay)) {
            _delay_write_pos -= static_cast<float>(_max_delay);
        } else if (_delay_write_pos < 0.0f) {
            _delay_write_pos += static_cast<float>(_max_delay);
        }
    }

    inline void AdvanceDelayWrite(float speed) {
        if (_max_delay == 0) return;
        _delay_write_pos += speed;
        if (_delay_write_pos >= static_cast<float>(_max_delay)) {
            _delay_write_pos -= static_cast<float>(_max_delay);
        } else if (_delay_write_pos < 0.0f) {
            _delay_write_pos += static_cast<float>(_max_delay);
        }
    }

    float _sample_rate{48000.0f};

    // Master Clock
    float _knob_clk_pos{0.15f};
    float _clk_phase{0.0f};
    float _clk_rate{1.0f};
    float _inj_clk{0.0f};
    bool  _clk_beat{false};

    // Probe Oscillator
    uint8_t _osc_mode{OSC_SAW};
    float   _knob_osc_pos{0.25f};
    float   _osc_phase{0.0f};
    float   _osc_vc{0.0f};
    bool    _osc_charging{true};
    float   _saw_ramp{0.0f};
    float   _saw_reset_pulse{0.0f};
    uint16_t _lfsr{0xACE1u};
    float   _rnd_held{0.0f};
    float   _inj_osc{0.0f};

    // Preamp Drive & Pluck
    float _knob_drive_pos{0.35f};
    float _prev_amp_out_l{0.0f};
    float _prev_amp_out_r{0.0f};
    float _amp_pluck{0.0f};
    float _motorboat_phase{0.0f};
    float _amp_in_vol{1.0f};
    float _g_osc_amp_smooth{0.0f};

    // Resonant Filter (TPT SVF Stereo)
    float _knob_flt_pos{0.65f};
    float _flt_low_l{0.0f};
    float _flt_low_r{0.0f};
    float _flt_s1_l{0.0f};
    float _flt_s2_l{0.0f};
    float _flt_s1_r{0.0f};
    float _flt_s2_r{0.0f};
    float _flt_ping{0.0f};
    float _flt_ping_env{0.0f};

    // Delay & Lo-Fi Stereo Processor (Dual SDRAM buffers)
    float _knob_dly_pos{0.50f};
    float _knob_fbk_pos{0.40f};
    float _smooth_delay_samples{28.0f};
    float _dly_knob_held{0.50f};
    float _delay_time_curr{28.0f};
    float _delay_time_prev{28.0f};
    float _delay_xfade{1.0f};
    bool  _delay_time_initialized{false};
    float _damp_l{0.0f};
    float _damp_r{0.0f};
    float _prev_dly_read_l{0.0f};
    float _prev_dly_read_r{0.0f};
    float* _delay_buf_l{nullptr};
    float* _delay_buf_r{nullptr};
    size_t _max_delay{0};
    float _delay_write_pos{0.0f};
    LoFiDelayModel _lofi_l;
    LoFiDelayModel _lofi_r;
    bool  _is_frozen{false};
    bool  _was_frozen{false};
    float _freeze_len{48000.0f};
    float _freeze_phase{0.0f};
    float _freeze_anchor{0.0f};
    bool  _rst_tap{false};
    float _rev_phase_l{0.0f};
    float _rev_phase_r{0.0f};
    float _stutter_pos_l{0.0f};
    float _stutter_pos_r{0.0f};
    float _sh_step_ratio{1.0f};
    uint32_t _rst_gnd_hold{0};
    uint16_t _slip_timer{0};
    uint16_t _slip_len{2400};
    float    _slip_offset_l{0.0f};
    float    _slip_offset_r{0.0f};

    // Mix & Master Out
    float _knob_mix_pos{0.50f};
    float _prev_out_l{0.0f};
    float _prev_out_r{0.0f};

    // Active bridges for sparse relaxation
    ActiveBridge _bridges[kMaxActiveBridges];
    uint8_t      _num_bridges{0};
    float        _bridge_g[12][12];

    // Ground shunts (target and per-sample smoothed)
    float        _shunt_gnd_target[12];
    float        _shunt_gnd_smooth[12];
    float        _bridged_weight[12]{0.0f};

    // Resolved DC node voltages (target and per-sample smoothed)
    float _v_dc_target[12]{0.5f};
    float _v_dc_smooth[12]{0.5f};

    // Pad pressures & proximities
    float _pressures[12]{0.0f};
    float _pressures_smooth[12]{0.0f};
    float _prev_pressures[12]{0.0f};
    float _proximities[12]{0.0f};

    // Ground hum state
    float _smooth_gnd_hum{0.0f};
    float _gnd_hum_phase{0.0f};
    uint32_t _gnd_crackle_rng{13579u};
    float _gnd_crackle_filter{0.0f};

    // DC Blockers (Stereo)
    DcBlocker _dc_block_amp_l;
    DcBlocker _dc_block_amp_r;
    DcBlocker _dc_block_flt_l;
    DcBlocker _dc_block_flt_r;
    DcBlocker _dc_block_dly_l;
    DcBlocker _dc_block_dly_r;

    // Stereo Chassis Contact Mic (Pad 10 IN / Pad 11 OUT)
    synthux::ContactMicModel _contact_mic;
    PadGestureTracker _tracker_p10;
    PadGestureTracker _tracker_p11;
    float _rub_speed_p10{0.0f};
    float _rub_speed_p11{0.0f};
    uint8_t _rest_scans[12]{10};
};

} // namespace bends
