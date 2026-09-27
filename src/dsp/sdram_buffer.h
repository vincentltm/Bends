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

// Dedicated historical memory snapshot slots for Pad 1 (4 slots x 262,144 samples = ~5.46s each = ~21.8s total in SDRAM)
static constexpr size_t kNumCorruptSlots = 4;
static constexpr size_t kCorruptSlotSamples = 262144;
static constexpr size_t kCorruptSlotMask = kCorruptSlotSamples - 1;

extern float DSY_SDRAM_BSS g_sdram_corrupt_l[kNumCorruptSlots][kCorruptSlotSamples];
extern float DSY_SDRAM_BSS g_sdram_corrupt_r[kNumCorruptSlots][kCorruptSlotSamples];

} // namespace bends
