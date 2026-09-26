#pragma once

#include "daisy_seed.h"
#include "knobs.h"
#include "pads.h"
#include "switches.h"
#include "mvalue.h"
#include "latch.h"
#include "log.h"
#include "air.h"

namespace synthux {

/**
 * Unified hardware wrapper for Bends / Synthux Touch 2.
 * Bundles the 12 capacitive pads with continuous pressure, 8 analog inputs (knobs/faders),
 * 2 toggle switches, and 2D/3D air proximity tracking.
 */
class Touch {
public:
    void Init(daisy::DaisySeed& hw) {
        _knobs.Init(hw);
        _pads.Init(hw);
        _switches.Init();
        hw.adc.Start();
    }

    void Process() {
        _pads.Process();
        _knobs.Process();
        if (_air.IsEnabled()) {
            _air.Process(_pads);
        }
    }

    Knobs& knobs() { return _knobs; }
    Pads& pads() { return _pads; }
    Switches& switches() { return _switches; }
    AirTracker& air() { return _air; }

private:
    Knobs _knobs;
    Pads _pads;
    Switches _switches;
    AirTracker _air;
};

} // namespace synthux
