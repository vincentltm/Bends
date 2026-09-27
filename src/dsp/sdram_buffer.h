#pragma once

#include "dev/sdram.h"
#include <cstddef>

namespace bends {

// 524,288 samples (2^19) = ~10.92s stereo buffer at 48kHz (4 MB total in external SDRAM)
// Power-of-two size allows single-cycle bitwise masking (& kMaxDelayMask) instead of integer modulo.
static constexpr size_t kMaxDelaySamples = 524288;
static constexpr size_t kMaxDelayMask = kMaxDelaySamples - 1;

extern float DSY_SDRAM_BSS g_sdram_delay_l[kMaxDelaySamples];
extern float DSY_SDRAM_BSS g_sdram_delay_r[kMaxDelaySamples];

} // namespace bends
