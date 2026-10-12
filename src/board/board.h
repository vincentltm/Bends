#pragma once

#include "board/nodes.h"
#include "board/contacts.h"
#include "board/solver.h"
#include "board/rail.h"
#include "bends/bends.h"
#include "surface/frame.h"
#include "parts/clock.h"
#include "parts/oscillator.h"
#include "parts/hum.h"
#include "parts/contact_mic.h"
#include "parts/preamp.h"
#include "parts/filter.h"
#include "parts/delay.h"
#include "parts/diffuser.h"
#include "parts/mixer.h"
#include "dsp/filters.h"
#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

class Board {
public:
    void Init(float sample_rate, float* delay_buf_l, float* delay_buf_r, size_t max_delay = Delay::kMaxDelay) {
        _sample_rate = sample_rate;

        _contacts.Reset();
        _solver.Reset();
        _rail.Init(sample_rate);

        _clock.Init(sample_rate);
        _osc.Init(sample_rate);
        _hum.Init(sample_rate);
        _contact_mic.Init(sample_rate);
        _preamp.Init(sample_rate);
        _filter.Init(sample_rate);
        _delay.Init(sample_rate, delay_buf_l, delay_buf_r, max_delay);
        _diffuser.Init();

        _dc_block_out_l.Init(sample_rate);
        _dc_block_out_r.Init(sample_rate);

        for (size_t i = 0; i < kSolverNodes; ++i) {
            _prev_node_sig[i][0] = 0.0f;
            _prev_node_sig[i][1] = 0.0f;
        }

        _flush_samples_remaining = 0;
    }

    // Process 64-sample audio block
    void ProcessBlock(const float* in_l, const float* in_r,
                      float* out_l, float* out_r,
                      size_t size, const ControlFrame& frame) {
        // 1. Update contact conductances and body coupling
        _contacts.Process(frame);

        // 2. Solve passive nodal equations
        float vcc = _rail.EffectiveVcc();
        _solver.Solve(_contacts, vcc);

        // 3. Compile declarative bends against active condition flags
        uint8_t when_flags = 0;
        if (frame.osc_mode == Oscillator::ModeSaw) when_flags |= When::Saw;
        else if (frame.osc_mode == Oscillator::ModeSin) when_flags |= When::Sin;
        else when_flags |= When::Rnd;

        if (_clock.Rate() < 35.0f) when_flags |= When::ClkSlow;
        else when_flags |= When::ClkFast;

        if (vcc < 0.40f) when_flags |= When::Starved;
        if (frame.is_latched) when_flags |= When::Latched;

        _bend_engine.Compile(kBends, kNumBends, _contacts, when_flags);

        // 4. Memory flush handling (edge-triggered + non-blocking incremental SDRAM zeroing)
        bool flush_active = _bend_engine.FlushRequested() || frame.tap_rst;
        if (flush_active) {
            if (!_prev_flush_active) {
                _delay.Reset();
                _filter.Reset();
                _preamp.Reset();
            }
            _prev_flush_active = true;
        } else {
            _prev_flush_active = false;
        }
        _delay.ServiceIncrementalClear();

        // 5. Update power rail
        float total_load = _contacts.TotalCurrentLoad();
        float vcc_pull   = _solver.Pull(NodeVcc);
        _rail.ProcessControl(frame.s37_vcc, total_load, vcc_pull);

        // Pad gesture triggers
        if (frame.tap_vel_p10 > 0.0f) _contact_mic.TriggerTap(frame.tap_vel_p10, false);
        if (frame.tap_vel_p11 > 0.0f) _contact_mic.TriggerTap(frame.tap_vel_p11, true);
        if (frame.tap_amp) _preamp.TriggerPluck(0.85f * (frame.s30_amp + 0.25f));
        if (frame.tap_flt) _filter.TriggerPing(0.75f);

        // 6. Block-rate invariant calculations hoisted before 64-sample audio loop
        float air_osc = frame.proximities[NodeOsc] * (1.0f - std::min(1.0f, frame.pressures[NodeOsc] * 2.0f));
        float air_flt = frame.proximities[NodeFlt] * (1.0f - std::min(1.0f, frame.pressures[NodeFlt] * 2.0f));
        float touched_sum = frame.pressures[NodeAmp] + frame.pressures[NodeFlt] + frame.pressures[NodeOsc] + frame.pressures[NodeDly];
        float howl_amount = std::max({_contacts.Conductance(NodeVcc, NodeFlt) * 1.5f,
                                      _contacts.Conductance(NodeFlt, NodeFbk) * 1.3f,
                                      frame.pressures[NodeFlt] * _solver.Retention(NodeFlt) * 1.2f});
        float gnd_touch = frame.pressures[NodeGnd] * _solver.Retention(NodeGnd);
        float string_damp_base = 0.25f + 0.72f * (frame.s32_flt + _solver.Pull(NodeFlt) * 0.2f);
        float max_fb = (frame.s33_dly < 0.20f) ? 0.992f : 1.10f;
        float eff_fb_base = (frame.s34_fbk + _solver.Pull(NodeFbk) * 0.2f);
        float eff_fb_press = frame.pressures[NodeFbk] * 0.30f;
        float press_amp_ret = frame.pressures[NodeAmp] * _solver.Retention(NodeAmp);
        float press_flt_ret = frame.pressures[NodeFlt] * _solver.Retention(NodeFlt);
        float press_mix_ret = frame.pressures[NodeMix] * _solver.Retention(NodeMix);
        bool is_frozen_or_inhibit = frame.is_frozen || _bend_engine.WriteInhibited();
        bool is_clk_slow = (_clock.Rate() < 35.0f);

        // Submodule block-rate preparation
        _clock.PrepareBlock(frame.s36_clk, _solver.Pull(NodeClk));
        _osc.PrepareBlock(frame.s31_osc, air_osc);
        _filter.PrepareBlock(frame.s32_flt, _solver.Pull(NodeFlt), press_flt_ret, air_flt);
        _delay.PrepareBlock(frame.s33_dly, frame.s36_clk, _solver.Pull(NodeDly), _solver.Pull(NodeClk));
        _preamp.PrepareBlock(frame.s30_amp, _solver.Pull(NodeAmp));

        float dry_in_scale = _preamp.InputVolume() * 0.25f * _solver.Retention(NodeIn);
        float dry_out_scale = _preamp.InputVolume() * 0.25f * _solver.Retention(NodeOut);

        bool has_dyn = (_bend_engine.NumDynamicRoutes() > 0);
        BoardMods static_mods = _bend_engine.BaseMods();

        uint8_t num_br = _contacts.NumActiveBridges();
        const auto* br = _contacts.ActiveBridges();
        float g_osc_amp = _contacts.Conductance(NodeOsc, NodeAmp);
        float g_osc_dly = _contacts.Conductance(NodeOsc, NodeDly);

        for (size_t n = 0; n < size; ++n) {
            float inl = in_l ? in_l[n] : 0.0f;
            float inr = in_r ? in_r[n] : 0.0f;

            // Compute AC cross-injections from active contact bridges
            float s_inj[kSolverNodes][2]{{0.0f}};
            for (uint8_t k = 0; k < num_br; ++k) {
                uint8_t a = br[k].a;
                uint8_t b = br[k].b;
                float g = br[k].g;
                s_inj[a][0] += g * _prev_node_sig[b][0];
                s_inj[a][1] += g * _prev_node_sig[b][1];
                s_inj[b][0] += g * _prev_node_sig[a][0];
                s_inj[b][1] += g * _prev_node_sig[a][1];
            }

            vcc = _rail.Tick(_prev_node_sig[NodeVcc][0]);
            float inj_vcc = (s_inj[NodeVcc][0] + s_inj[NodeVcc][1]) * 0.5f;
            if (inj_vcc != 0.0f) {
                vcc = std::clamp(vcc + inj_vcc * 0.35f, 0.01f, 1.0f);
            }
            float starve = 1.0f - vcc;

            // Route dynamic audio-rate bends into typed BoardMods
            BoardMods dyn_mods;
            if (has_dyn) {
                dyn_mods = static_mods;
                _bend_engine.AccumulateDynamic(dyn_mods, _prev_node_sig, _prev_osc_ramp, _prev_osc_reset, _prev_osc_sub, _prev_node_sig[NodeClk][0]);
            }
            const BoardMods& mods = has_dyn ? dyn_mods : static_mods;

            // Tick Clock
            float s_clk = 0.0f;
            _clock.Tick(vcc, s_inj[NodeClk][0], s_clk);
            s_clk *= _solver.Retention(NodeClk);

            // Tick Oscillator
            float s_osc = 0.0f, osc_ramp = 0.0f, osc_reset = 0.0f, osc_sub = 0.0f;
            _osc.Tick(frame.osc_mode, vcc,
                      0.60f + 0.40f * (vcc * vcc), s_inj[NodeOsc][0], mods.osc,
                      s_osc, osc_ramp, osc_reset, osc_sub);
            s_osc *= _solver.Retention(NodeOsc);
            _prev_osc_ramp = osc_ramp;
            _prev_osc_reset = osc_reset;
            _prev_osc_sub = osc_sub;

            // Tick Hum
            float s_hum = 0.0f;
            _hum.Tick(vcc, starve, gnd_touch, frame.proximities[NodeGnd], touched_sum, s_hum);
            s_hum *= _solver.Retention(NodeGnd);

            // Contact Mic
            float mic_l = 0.0f, mic_r = 0.0f;
            _contact_mic.Process(inl, inr, frame.pressures[NodeIn], frame.rub_speed_p10,
                                 frame.pressures[NodeOut], frame.rub_speed_p11, mic_l, mic_r);
            mic_l *= _solver.Retention(NodeIn);
            mic_r *= _solver.Retention(NodeOut);

            // Preamp: receives mic + direct oscillator bridge injection + AC cross-injections
            float amp_l = 0.0f, amp_r = 0.0f;
            float osc_inject = s_osc * g_osc_amp * 0.85f + s_inj[NodeAmp][0];
            _preamp.Process(mic_l + s_inj[NodeIn][0], mic_r + s_inj[NodeIn][1],
                            press_amp_ret,
                            mods.preamp,
                            osc_inject, s_hum, vcc, starve, _solver.Retention(NodeAmp),
                            amp_l, amp_r);

            // Filter
            float flt_l = 0.0f, flt_r = 0.0f;
            FilterMods flt_mods = mods.filter;
            flt_mods.clock_step += s_inj[NodeFlt][0];
            flt_mods.howl = howl_amount;
            _filter.Process(amp_l + s_inj[NodeFlt][0], amp_r + s_inj[NodeFlt][1],
                            press_flt_ret,
                            flt_mods,
                            vcc, starve, _solver.Retention(NodeFlt),
                            flt_l, flt_r);

            // Delay Feedback Loop routes & Oscillator Injection into Delay
            float fbk_route = mods.delay.fbk_filter_loop;
            float osc_to_dly = 0.0f;
            if (g_osc_dly > 0.015f) {
                if (frame.osc_mode == Oscillator::ModeSaw) {
                    if (frame.s33_dly < 0.20f) {
                        osc_to_dly = (osc_reset * 0.92f + s_osc * 0.22f) * g_osc_dly;
                    } else {
                        osc_to_dly = (s_osc * 0.35f + osc_reset * 0.50f) * g_osc_dly;
                    }
                } else {
                    osc_to_dly = s_osc * g_osc_dly * 0.45f;
                }
            }

            float dly_in_l = flt_l * (1.0f - fbk_route) + flt_l * fbk_route * 1.05f + osc_to_dly + s_inj[NodeDly][0];
            float dly_in_r = flt_r * (1.0f - fbk_route) + flt_r * fbk_route * 1.05f + osc_to_dly + s_inj[NodeDly][1];

            // Recirculating Modulator (OSC <-> FBK)
            float osc_fbk = mods.delay.fbk_recirc_fm;
            if (osc_fbk > 0.015f) {
                dly_in_l = dly_in_l * (1.0f + s_osc * 0.95f * osc_fbk) + s_osc * 0.40f * osc_fbk;
                dly_in_r = dly_in_r * (1.0f - s_osc * 0.95f * osc_fbk) - s_osc * 0.40f * osc_fbk;
            }

            float dly_l = 0.0f, dly_r = 0.0f;
            float string_damp = daisysp::fclamp(string_damp_base * (0.65f + 0.35f * vcc), 0.05f, 0.98f);
            float eff_fb = std::clamp(eff_fb_base * (max_fb + 0.05f * starve) + eff_fb_press, 0.0f, 1.40f);

            _delay.Process(dly_in_l, dly_in_r,
                           mods.delay,
                           is_frozen_or_inhibit,
                           is_clk_slow, _clock.Phase(),
                           string_damp, eff_fb,
                           vcc, starve, _solver.Retention(NodeDly), _solver.Retention(NodeFbk),
                           dly_l, dly_r);

            // Diffuser
            float diff_l = 0.0f, diff_r = 0.0f;
            if (mods.mixer.diffuser > 0.005f) {
                _diffuser.Process(dly_l, dly_r, diff_l, diff_r);
            }

            // Mixer & Master Output
            float sig_l = 0.0f, sig_r = 0.0f;
            float dry_l = flt_l + inl * dry_in_scale;
            float dry_r = flt_r + inr * dry_out_scale;

            _mixer.Process(dry_l, dry_r, dly_l, dly_r,
                           frame.s35_mix, _solver.Pull(NodeMix),
                           press_mix_ret,
                           s_inj[NodeMix][0], s_inj[NodeMix][1],
                           s_inj[NodeOut][0], s_inj[NodeOut][1],
                           mods.mixer, flt_l, flt_r,
                           diff_l, diff_r,
                           _solver.Retention(NodeMix), _solver.Retention(NodeOut),
                           sig_l, sig_r);

            // Store node signals for z^-1 cross-injection
            _prev_node_sig[NodeClk][0] = s_clk;  _prev_node_sig[NodeClk][1] = s_clk;
            _prev_node_sig[NodeOsc][0] = s_osc;  _prev_node_sig[NodeOsc][1] = s_osc;
            _prev_node_sig[NodeFlt][0] = flt_l;  _prev_node_sig[NodeFlt][1] = flt_r;
            _prev_node_sig[NodeGnd][0] = s_hum;  _prev_node_sig[NodeGnd][1] = s_hum;
            _prev_node_sig[NodeDly][0] = dly_l;  _prev_node_sig[NodeDly][1] = dly_r;
            _prev_node_sig[NodeFbk][0] = dly_l;  _prev_node_sig[NodeFbk][1] = dly_r;
            _prev_node_sig[NodeAmp][0] = amp_l;  _prev_node_sig[NodeAmp][1] = amp_r;
            _prev_node_sig[NodeMix][0] = sig_l;  _prev_node_sig[NodeMix][1] = sig_r;
            _prev_node_sig[NodeIn][0]  = mic_l;  _prev_node_sig[NodeIn][1]  = mic_r;
            _prev_node_sig[NodeOut][0] = sig_l;  _prev_node_sig[NodeOut][1] = sig_r;
            _prev_node_sig[NodeVcc][0] = s_inj[NodeVcc][0];
            _prev_node_sig[NodeVcc][1] = s_inj[NodeVcc][1];

            // DC Blocking & Limiting
            sig_l = _dc_block_out_l.Process(sig_l);
            sig_r = _dc_block_out_r.Process(sig_r);

            if (std::isnan(sig_l) || std::isinf(sig_l)) sig_l = 0.0f;
            if (std::isnan(sig_r) || std::isinf(sig_r)) sig_r = 0.0f;

            sig_l = daisysp::SoftLimit(sig_l);
            sig_r = daisysp::SoftLimit(sig_r);

            out_l[n] = sig_l;
            out_r[n] = sig_r;
        }
    }

    const Rail& GetRail() const { return _rail; }
    const Solver& GetSolver() const { return _solver; }
    const Contacts& GetContacts() const { return _contacts; }
    bool ClkBeat() const { return _clock.Beat(); }

    Preamp& GetPreamp() { return _preamp; }
    const Preamp& GetPreamp() const { return _preamp; }
    Filter& GetFilter() { return _filter; }
    const Filter& GetFilter() const { return _filter; }
    ContactMic& GetContactMic() { return _contact_mic; }
    const ContactMic& GetContactMic() const { return _contact_mic; }

private:
    float _sample_rate{48000.0f};

    Contacts    _contacts;
    Solver      _solver;
    Rail        _rail;
    BendEngine  _bend_engine;

    Clock       _clock;
    Oscillator  _osc;
    Hum         _hum;
    ContactMic  _contact_mic;
    Preamp      _preamp;
    Filter      _filter;
    Delay       _delay;
    DiffuserTank _diffuser;
    Mixer       _mixer;

    daisysp::DcBlock _dc_block_out_l;
    daisysp::DcBlock _dc_block_out_r;

    float _prev_node_sig[kSolverNodes][2];
    float _prev_osc_ramp{0.0f};
    float _prev_osc_reset{0.0f};
    float _prev_osc_sub{0.0f};

    bool     _prev_flush_active{false};
    uint32_t _flush_samples_remaining{0};
};

} // namespace bends
