#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>

namespace bends {

class MasterClock {
public:
    void Init(float sample_rate) {
        _sample_rate = sample_rate;
        Reset();
    }

    void Reset() {
        _clock_freq = 48000.0f;
        _phase_accum = 0;
        _prev_bit14 = false;
        _beat_trigger = false;
        _lfsr = 0xACE1u;
    }

    // Call at control rate (~750 Hz)
    void ProcessControl(float fader_pos, float vcc_eff, float clk_pull, float clk_load) {
        // Fader sweeps base rate: from 2 kHz up to 48 kHz
        float fader_curved = fader_pos * fader_pos;
        float base_f = 2000.0f + 46000.0f * fader_curved;

        // VCC starve slows the clock and drops sample rate
        float v_scale = std::pow(std::clamp(vcc_eff, 0.05f, 1.0f), 1.4f);

        // Capacitive load on pad P00 drags clock speed downward
        float load_drag = 1.0f / (1.0f + clk_load * 0.40f);

        // DC pull from network shifts clock frequency (e.g. pulled up or down)
        float pull_mult = std::pow(2.0f, clk_pull * 1.5f);

        _clock_freq = std::clamp(base_f * v_scale * load_drag * pull_mult, 1500.0f, 48000.0f);
        _sample_rate_ratio = _clock_freq / _sample_rate;
    }

    // Call per sample (48 kHz)
    inline float Tick(float inj_sync) {
        // Starvation jitter when VCC is low
        uint32_t step = static_cast<uint32_t>(_sample_rate_ratio * 65536.0f);
        if (_sample_rate_ratio < 0.35f) {
            // LFSR jitter
            _lfsr = (_lfsr >> 1) ^ (-(_lfsr & 1u) & 0xB400u);
            step += (_lfsr & 0x0Fu);
        }

        // External sync pull: if audio injected into CLK node crosses threshold, sync edge
        if (inj_sync > 0.45f && _prev_inj_sync <= 0.45f) {
            _phase_accum += 0x1000u; // Phase nudge
        }
        _prev_inj_sync = inj_sync;

        _phase_accum += step;

        bool bit10 = (_phase_accum & (1u << 10)) != 0;
        bool bit12 = (_phase_accum & (1u << 12)) != 0;
        bool bit14 = (_phase_accum & (1u << 14)) != 0;

        // Beat trigger for LED blinking (quarter note division)
        _beat_trigger = (bit14 && !_prev_bit14);
        _prev_bit14 = bit14;

        // Weighted R-2R divider output: provides accent pattern to NodeClk
        float val = (bit14 ? 0.50f : 0.0f) +
                    (bit12 ? 0.25f : 0.0f) +
                    (bit10 ? 0.125f : 0.0f);

        // Center around zero AC (-0.44 to +0.44)
        return val - 0.4375f;
    }

    float ClockFreq() const        { return _clock_freq; }
    float SampleRateRatio() const  { return _sample_rate_ratio; }
    bool  BeatTrigger() const      { return _beat_trigger; }

private:
    float _sample_rate{48000.0f};
    float _clock_freq{48000.0f};
    float _sample_rate_ratio{1.0f};
    float _prev_inj_sync{0.0f};

    uint32_t _phase_accum{0};
    uint16_t _lfsr{0xACE1u};
    bool     _prev_bit14{false};
    bool     _beat_trigger{false};
};

} // namespace bends
