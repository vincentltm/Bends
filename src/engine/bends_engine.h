#pragma once

#include "circuit/node_table.h"
#include "circuit/network.h"
#include "circuit/rail.h"
#include "circuit/synth_circuit.h"
#include "daisysp.h"
#include <algorithm>

namespace bends {

class BendsEngine {
public:
    void Init(float sample_rate, uint16_t block_size,
              float* delay_buf_l = nullptr, float* delay_buf_r = nullptr,
              size_t max_delay = SynthCircuit::kMaxDelay) {
        _sample_rate = sample_rate;
        _block_size = block_size;

        _network.Reset();
        _rail.Init(sample_rate);
        _synth.Init(sample_rate, delay_buf_l, delay_buf_r, max_delay);

        _dc_blocker_l.Init(sample_rate);
        _dc_blocker_r.Init(sample_rate);
        _limiter.Init();
    }

    void SetKnobs(float s30_amp, float s31_osc, float s32_flt,
                  float s33_dly, float s34_fbk, float s35_mix,
                  float s36_clk) {
        _synth.SetKnobs(s30_amp, s31_osc, s32_flt, s33_dly, s34_fbk, s35_mix, s36_clk);
    }

    void SetSwitches(uint8_t osc_mode, bool is_frozen) {
        _synth.SetSwitches(osc_mode, is_frozen);
    }

    void SetPressures(const float pressures[12], bool new_scan = true) {
        _synth.SetPressures(pressures, new_scan);
    }

    void SetProximities(const float proximities[12]) {
        _synth.SetProximities(proximities);
    }

    Network&      Net()     { return _network; }
    Rail&         GetRail() { return _rail; }
    SynthCircuit& Synth()   { return _synth; }
    const SynthCircuit& Synth() const { return _synth; }

    // Audio block callback (64 samples)
    void ProcessBlock(const float* in_l, const float* in_r,
                      float* out_l, float* out_r,
                      size_t size) {
        // 1. Extract active bridges for the sparse relaxation solver
        ActiveBridge bridges[SynthCircuit::kMaxActiveBridges];
        uint8_t num_bridges = 0;
        for (size_t i = 0; i < kPhysicalNodes; ++i) {
            for (size_t j = i + 1; j < kPhysicalNodes; ++j) {
                float g = _network.Conductance(i, j);
                if (g > 0.015f && num_bridges < SynthCircuit::kMaxActiveBridges) {
                    bridges[num_bridges++] = { static_cast<uint8_t>(i), static_cast<uint8_t>(j), g };
                }
            }
        }
        _synth.SetActiveBridges(bridges, num_bridges);

        // 2. Resolve block-rate DC operating points
        float vcc = _rail.EffectiveVcc();
        _synth.SolveDC(vcc);

        // 3. Process per-sample audio
        for (size_t n = 0; n < size; ++n) {
            float inl = in_l ? in_l[n] : 0.0f;
            float inr = in_r ? in_r[n] : 0.0f;

            vcc = _rail.Tick(0.0f);

            float sig_l = 0.0f;
            float sig_r = 0.0f;
            _synth.Tick(inl, inr, vcc, sig_l, sig_r);

            // DC blocking
            sig_l = _dc_blocker_l.Process(sig_l);
            sig_r = _dc_blocker_r.Process(sig_r);

            // Soft limiting & NaN guard
            if (std::isnan(sig_l) || std::isinf(sig_l)) sig_l = 0.0f;
            if (std::isnan(sig_r) || std::isinf(sig_r)) sig_r = 0.0f;
            sig_l = daisysp::SoftLimit(sig_l);
            sig_r = daisysp::SoftLimit(sig_r);

            out_l[n] = sig_l;
            out_r[n] = sig_r;
        }
    }

private:
    float _sample_rate{48000.0f};
    uint16_t _block_size{64};

    Network           _network;
    Rail              _rail;
    SynthCircuit      _synth;

    daisysp::DcBlock  _dc_blocker_l;
    daisysp::DcBlock  _dc_blocker_r;
    daisysp::Limiter  _limiter;
};

} // namespace bends
