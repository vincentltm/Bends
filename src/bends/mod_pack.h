#pragma once

#include "bends/slots.h"
#include <algorithm>

namespace bends {

struct OscillatorMods {
    float freq_mod{0.0f};
    float sub_octave{0.0f};
    bool  sync{false};
};

struct PreampMods {
    float drive{0.0f};
    float tremolo{0.0f};
    float ring_mod{0.0f};
    float bias{0.0f};
};

struct FilterMods {
    float cutoff_ramp{0.0f};
    float clock_step{0.0f};
    float resonance{0.0f};
    float drive{0.0f};
    float ping{0.0f};
    float howl{0.0f};
};

struct DelayMods {
    float time_mod{0.0f};
    float clock_fm{0.0f};
    float overclock{0.0f};
    float stutter{0.0f};
    float reverse{0.0f};
    float address_slip{0.0f};
    float xor_glitch{0.0f};
    float decimate{0.0f};
    float ping_pong{0.0f};
    float fbk_filter_loop{0.0f};
    float fbk_recirc_fm{0.0f};
};

struct MixerMods {
    float filter_isolate{0.0f};
    float diffuser{0.0f};
};

struct BoardMods {
    PreampMods     preamp;
    FilterMods     filter;
    DelayMods      delay;
    MixerMods      mixer;
    OscillatorMods osc;

    void Reset() {
        preamp = PreampMods{};
        filter = FilterMods{};
        delay  = DelayMods{};
        mixer  = MixerMods{};
        osc    = OscillatorMods{};
    }

    inline void Apply(Slot slot, Verb verb, float amount, float src_val) {
        float eff = (verb == Verb::Set) ? amount : (amount * src_val);
        switch (slot) {
            case Slot::PreampDrive:    preamp.drive += eff; break;
            case Slot::AmpTremolo:     preamp.tremolo = std::max(preamp.tremolo, eff); break;
            case Slot::PreampRingMod:  preamp.ring_mod += eff; break;
            case Slot::PreampBias:     preamp.bias += eff; break;

            case Slot::FltCutoffRamp:  filter.cutoff_ramp += eff; break;
            case Slot::FltClockStep:   filter.clock_step += eff; break;
            case Slot::FltRes:         filter.resonance += eff; break;
            case Slot::FltDrive:       filter.drive += eff; break;
            case Slot::FltPing:        filter.ping = std::max(filter.ping, eff); break;

            case Slot::DlyTimeMod:     delay.time_mod += eff; break;
            case Slot::DlyClockFm:     delay.clock_fm += eff; break;
            case Slot::DlyOverclock:   delay.overclock += eff; break;
            case Slot::DlyStutter:     delay.stutter = std::max(delay.stutter, eff); break;
            case Slot::DlyReverse:     delay.reverse = std::max(delay.reverse, eff); break;
            case Slot::DlyAddressSlip: delay.address_slip = std::max(delay.address_slip, eff); break;
            case Slot::DlyXorGlitch:   delay.xor_glitch = std::max(delay.xor_glitch, eff); break;
            case Slot::DlyDecimate:    delay.decimate += eff; break;
            case Slot::DlyPingPong:    delay.ping_pong = std::max(delay.ping_pong, eff); break;

            case Slot::FbkFilterLoop:  delay.fbk_filter_loop = std::max(delay.fbk_filter_loop, eff); break;
            case Slot::FbkRecircFm:    delay.fbk_recirc_fm += eff; break;

            case Slot::MixFilterIsol:  mixer.filter_isolate = std::max(mixer.filter_isolate, eff); break;
            case Slot::MixDiffuser:    mixer.diffuser = std::max(mixer.diffuser, eff); break;

            case Slot::OscPhase:       if (eff > 0.5f) osc.sync = true; break;
            case Slot::OscFreqMod:     osc.freq_mod += eff; break;
            case Slot::OscSubOctave:   osc.sub_octave = std::max(osc.sub_octave, eff); break;
            default: break;
        }
    }
};

} // namespace bends
