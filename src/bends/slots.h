#pragma once

#include <cstdint>

namespace bends {

enum class Slot : uint8_t {
    None = 0,

    // Preamp
    PreampDrive,
    PreampBias,
    AmpTremolo,
    PreampRingMod,

    // Filter
    FltCutoff,
    FltCutoffRamp,
    FltRes,
    FltDrive,
    FltPing,
    FltClockStep,

    // Delay
    DlyTimeMod,
    DlyClockFm,
    DlyOverclock,
    DlyStutter,
    DlyReverse,
    DlyAddressSlip,
    DlyXorGlitch,
    DlyPingPong,
    DlyDecimate,
    DlyBeatRepeat,
    DlyTapeStop,
    DlyTapeWarpLoop,

    // Feedback
    FbkAmount,
    FbkFilterLoop,
    FbkRecircFm,
    FbkClockBurst,

    // Mixer
    MixWet,
    MixFilterIsol,
    MixDiffuser,

    // Oscillator
    OscPhase,
    OscFreqMod,
    OscSubOctave,

    // Clock
    ClkRateMod,
    ClkSync,
    ClkSubDiv,

    Count
};

enum class Sig : uint8_t {
    One = 0,         // Constant 1.0 (control-rate static amounts)
    Osc,             // Continuous oscillator waveform
    OscRamp,         // Unipolar saw ramp (0.0 to 1.0)
    OscReset,        // Single-sample flyback reset transient
    OscSub,          // Sub-octave square wave (f/2)
    ClkPulse,        // Clock divider pulse train
    PreampOut,       // Preamp output audio
    FltOut,          // SVF lowpass output audio
    DlyOut,          // Delay wet output audio
    Hum,             // 50Hz mains hum / crackle
    ContactMic,      // Contact mic tap/rub audio
    MasterOut        // Final master mixed output audio
};

enum class Verb : uint8_t {
    Modulate,  // Continuous additive modulation: slot += g * depth * source
    Set,       // Parameter amount override: slot = max(slot, g * depth)
    Route,     // Audio crossfade into stage: in = in*(1-g) + source*g*depth
    Ring,      // Four-quadrant ring modulation: in += source * in * g * depth
    Sync,      // Phase hard sync on source transient edge
    Trigger,   // Impulse excitation trigger (pings, plucks)
    Flush,     // Memory buffer clear on edge
    Inhibit    // Inhibit writes to delay buffer (freeze-like)
};

namespace When {
    constexpr uint8_t Always  = 0;
    constexpr uint8_t Saw     = (1 << 0);
    constexpr uint8_t Sin     = (1 << 1);
    constexpr uint8_t Rnd     = (1 << 2);
    constexpr uint8_t ClkSlow = (1 << 3);
    constexpr uint8_t ClkFast = (1 << 4);
    constexpr uint8_t Starved = (1 << 5);
    constexpr uint8_t Latched = (1 << 6);
}

} // namespace bends
