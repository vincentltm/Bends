#include "daisy_seed.h"
#include "surface/touch.h"
#include "app/instrument.h"

using namespace daisy;
using namespace synthux;
using namespace bends;

static DaisySeed hw;
static Touch touch;
static Instrument instrument;

// External SDRAM stereo delay buffers (16.0s at 48kHz each = 6.14 MB in 64MB SDRAM)
static float DSY_SDRAM_BSS g_sdram_delay_buffer_l[Delay::kMaxDelay];
static float DSY_SDRAM_BSS g_sdram_delay_buffer_r[Delay::kMaxDelay];

void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size) {
    instrument.ProcessBlock(in[0], in[1], out[0], out[1], size);
    instrument.UpdateLedPwm(hw);
}

int main(void) {
    hw.Init(true);
    // Enable Cortex-M7 FPU Flush-to-Zero (FZ) and Default NaN (DN) modes
    __set_FPSCR(__get_FPSCR() | (1 << 24) | (1 << 25));

    // Zero out SDRAM delay buffers after SDRAM initialization
    memset(g_sdram_delay_buffer_l, 0, sizeof(g_sdram_delay_buffer_l));
    memset(g_sdram_delay_buffer_r, 0, sizeof(g_sdram_delay_buffer_r));

    // 64-sample block size (1.33 ms @ 48 kHz)
    hw.SetAudioBlockSize(64);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    touch.Init(hw);
    instrument.Init(hw.AudioSampleRate(), g_sdram_delay_buffer_l, g_sdram_delay_buffer_r, Delay::kMaxDelay);

    // Calibrate baseline tracking on startup
    for (int i = 0; i < 40; i++) {
        touch.Process();
        System::Delay(3);
    }
    System::Delay(120);
    touch.pads().LockBaseline(true);

    touch.air().Enable(true);
    touch.air().SetNoiseGate(1.1f);
    touch.air().SetMaxProximity(26.0f);

    // Initial surface read to populate frame buffer with physical knob/switch positions
    touch.Process();
    instrument.ProcessSurface(touch, true);

    // Startup LED sequence
    for (int b = 0; b < 3; ++b) {
        hw.SetLed(true);
        System::Delay(50);
        hw.SetLed(false);
        System::Delay(50);
    }

    hw.StartAudio(AudioCallback);

    uint32_t rst_hold_start = 0;

    while (1) {
        bool new_scan = touch.Process();
        instrument.ProcessSurface(touch, new_scan);

        // Touch DFU Bootloader Gesture:
        // Hold Pad 01 (RST) alone for 1.8 seconds to reboot into STM DFU bootloader without touching physical PCB buttons.
        bool rst_touched = touch.pads().IsTouched(1) && (touch.pads().Pressure(1) > 0.35f);
        if (rst_touched) {
            for (size_t p = 0; p < 12; ++p) {
                if (p != 1 && touch.pads().Pressure(p) > 0.15f) {
                    rst_touched = false;
                    break;
                }
            }
        }

        if (rst_touched) {
            if (rst_hold_start == 0) {
                rst_hold_start = System::GetNow();
            } else {
                uint32_t elapsed = System::GetNow() - rst_hold_start;
                if (elapsed >= 1800) {
                    // Confirmed: enter DFU bootloader
                    instrument.SetLedOverride(0.0f);
                    hw.StopAudio();
                    for (int i = 0; i < 8; ++i) {
                        hw.SetLed(true);
                        System::Delay(30);
                        hw.SetLed(false);
                        System::Delay(30);
                    }
                    daisy::System::ResetToBootloader(daisy::System::BootloaderMode::STM);
                } else if (elapsed > 1100) {
                    // Fast strobe warning
                    instrument.SetLedOverride(((elapsed / 50) % 2 == 0) ? 1.0f : 0.0f);
                } else if (elapsed > 500) {
                    // Slow pulse warning
                    instrument.SetLedOverride(((elapsed / 160) % 2 == 0) ? 0.9f : 0.05f);
                }
            }
        } else {
            if (rst_hold_start != 0) {
                rst_hold_start = 0;
                instrument.SetLedOverride(-1.0f);
            }
        }

        System::Delay(1);
    }
}
