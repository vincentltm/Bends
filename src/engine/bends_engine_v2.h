#pragma once

#include "circuit/node_table.h"
#include "circuit/network.h"
#include "circuit/readers.h"
#include "circuit/gestures.h"
#include "circuit/rail.h"
#include "circuit/clock.h"
#include "circuit/audition.h"
#include "daisysp.h"
#include <algorithm>

namespace bends {

class BendsEngineV2 {
public:
    void Init(float sample_rate, uint16_t block_size) {
        _sample_rate = sample_rate;
        _block_size = block_size;

        _network.Reset();
        _readers.Init(sample_rate);
        _gestures.Reset();
        _rail.Init(sample_rate);
        _clock.Init(sample_rate);
        _audition.Init(sample_rate);

        _dc_blocker_l.Init(sample_rate);
        _dc_blocker_r.Init(sample_rate);

        _limiter.Init();

        // Boot-up fade in (0.8s)
        _boot_fade = 0.0f;
        _boot_fade_inc = 1.0f / (0.8f * sample_rate);

        _audition_mode = true; // In Phase 1, start in Audition Mode to verify the network
    }

    void SetAuditionMode(bool enable) {
        _audition_mode = enable;
    }

    bool IsAuditionMode() const { return _audition_mode; }

    // Direct access to core components
    Network&         Net()      { return _network; }
    NodeReader&      Readers()  { return _readers; }
    Gestures&        Gest()     { return _gestures; }
    Rail&            GetRail()  { return _rail; }
    MasterClock&     Clk()      { return _clock; }

    // Audio block callback (64 samples)
    void ProcessBlock(const float* in_l, const float* in_r,
                      float* out_l, float* out_r,
                      size_t size) {
        float node_s_in[kSolverNodes]  = {0.0f};
        float node_s_out[kSolverNodes] = {0.0f};
        float node_inj[kSolverNodes]   = {0.0f};

        for (size_t n = 0; n < size; ++n) {
            float inl = in_l ? in_l[n] : 0.0f;
            float inr = in_r ? in_r[n] : 0.0f;
            float ext_in = (inl + inr) * 0.5f;

            // 1. Gather generator signals for each node
            if (_audition_mode) {
                // Test tone audition mode: verify network coupling acoustically
                _audition.Generate(node_s_in);
                // Also mix in real external audio into node IN (P10)
                node_s_in[NodeIn] = ext_in;
            } else {
                // Clean passthrough in early skeleton
                node_s_in[NodeIn]  = ext_in;
                node_s_in[NodeAmp] = ext_in;
            }

            // 2. Advance network solver per-sample interpolation
            _network.Tick(node_s_in, node_s_out, node_inj);

            // 3. Process reader crossovers (LF vs HF separation)
            _readers.Process(node_inj);

            // 4. Update power rail per-sample ripple
            float vcc = _rail.Tick(_readers.InjHF(NodeVcc));

            // 5. Update master clock per-sample phase and divider
            float clk_pulse = _clock.Tick(_readers.InjHF(NodeClk));
            (void)clk_pulse;

            // 6. Output audition mixing or passthrough
            float sig_l = 0.0f;
            float sig_r = 0.0f;

            if (_audition_mode) {
                // In audition mode, the circuit is silent until pads are touched.
                // Each touched pad auditions its potential v_i (own signal + injected mix from other pads)
                for (size_t i = 0; i < kPhysicalNodes; ++i) {
                    float p = _network.Pressure(i);
                    if (p > 0.02f) {
                        float v = node_s_out[i];
                        sig_l += v * p * 0.40f;
                        sig_r += v * p * 0.40f;
                    }
                }
            } else {
                sig_l = ext_in * _network.Rho(NodeAmp);
                sig_r = ext_in * _network.Rho(NodeAmp);
            }

            // DC blocking
            sig_l = _dc_blocker_l.Process(sig_l);
            sig_r = _dc_blocker_r.Process(sig_r);

            // Boot fade in ramp
            if (_boot_fade < 1.0f) {
                sig_l *= _boot_fade;
                sig_r *= _boot_fade;
                _boot_fade += _boot_fade_inc;
            }

            // Starvation voltage scaling
            float starve_amp = std::clamp(vcc * 1.2f, 0.0f, 1.0f);
            sig_l *= starve_amp;
            sig_r *= starve_amp;

            // Strict NaN and runaway protection
            if (std::isnan(sig_l) || std::isinf(sig_l)) sig_l = 0.0f;
            if (std::isnan(sig_r) || std::isinf(sig_r)) sig_r = 0.0f;
            sig_l = std::clamp(sig_l, -1.0f, 1.0f);
            sig_r = std::clamp(sig_r, -1.0f, 1.0f);

            // Safety soft limiter
            sig_l = daisysp::SoftLimit(sig_l);
            sig_r = daisysp::SoftLimit(sig_r);

            out_l[n] = sig_l;
            out_r[n] = sig_r;
        }
    }

private:
    float _sample_rate{48000.0f};
    uint16_t _block_size{64};

    Network         _network;
    NodeReader      _readers;
    Gestures        _gestures;
    Rail            _rail;
    MasterClock     _clock;
    AuditionGenerator _audition;

    daisysp::DcBlock _dc_blocker_l;
    daisysp::DcBlock _dc_blocker_r;
    daisysp::Limiter _limiter;

    float _boot_fade{0.0f};
    float _boot_fade_inc{0.0f};
    bool  _audition_mode{true};
};

} // namespace bends
