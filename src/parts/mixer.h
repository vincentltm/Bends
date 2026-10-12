#pragma once

#include "dsp/fast_math.h"
#include "bends/mod_pack.h"
#include <algorithm>

namespace bends {

class Mixer {
public:
    inline void Process(float dry_l, float dry_r,
                        float wet_l, float wet_r,
                        float knob_mix, float node_pull_mix, float touch_mix,
                        float tap_mix_l, float tap_mix_r,
                        float tap_out_l, float tap_out_r,
                        const MixerMods& mods,
                        float flt_l, float flt_r,
                        float diff_l, float diff_r,
                        float shunt_mix, float shunt_out,
                        float& out_l, float& out_r) {
        float mix_wet = std::clamp(knob_mix + node_pull_mix * 0.20f + touch_mix * 0.40f, 0.0f, 1.0f);

        float eff_dry_l = dry_l + tap_mix_l + tap_out_l;
        float eff_dry_r = dry_r + tap_mix_r + tap_out_r;

        float eff_wet_l = wet_l;
        float eff_wet_r = wet_r;

        // Diffuser reverb wash blend
        if (mods.diffuser > 0.0f) {
            float d_amt = std::min(1.0f, mods.diffuser);
            eff_wet_l = eff_wet_l * (1.0f - d_amt * 0.5f) + diff_l * d_amt * 0.65f;
            eff_wet_r = eff_wet_r * (1.0f - d_amt * 0.5f) + diff_r * d_amt * 0.65f;
        }

        float mixed_l = ((1.0f - mix_wet) * eff_dry_l + mix_wet * eff_wet_l + (tap_mix_l + tap_out_l) * 0.45f) * shunt_mix * shunt_out * 0.75f;
        float mixed_r = ((1.0f - mix_wet) * eff_dry_r + mix_wet * eff_wet_r + (tap_mix_r + tap_out_r) * 0.45f) * shunt_mix * shunt_out * 0.75f;

        // Master output filter isolator (P04 FLT <-> P09 MIX)
        if (mods.filter_isolate > 0.015f) {
            float flt_master_l = flt_l * shunt_mix * shunt_out * 0.85f;
            float flt_master_r = flt_r * shunt_mix * shunt_out * 0.85f;
            float f_amt = std::min(1.0f, mods.filter_isolate);
            mixed_l = mixed_l * (1.0f - f_amt) + flt_master_l * f_amt;
            mixed_r = mixed_r * (1.0f - f_amt) + flt_master_r * f_amt;
        }

        out_l = mixed_l;
        out_r = mixed_r;
    }
};

} // namespace bends
