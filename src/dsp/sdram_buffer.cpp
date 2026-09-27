#include "sdram_buffer.h"

namespace bends {

float DSY_SDRAM_BSS g_sdram_delay_l[kMaxDelaySamples];
float DSY_SDRAM_BSS g_sdram_delay_r[kMaxDelaySamples];

float DSY_SDRAM_BSS g_sdram_corrupt_l[kNumCorruptSlots][kCorruptSlotSamples];
float DSY_SDRAM_BSS g_sdram_corrupt_r[kNumCorruptSlots][kCorruptSlotSamples];

} // namespace bends
