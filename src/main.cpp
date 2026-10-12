#include "daisy_seed.h"
#include "touch/touch.h"
#include "engine/bends_engine.h"
#include "engine/bends_controller.h"

using namespace daisy;
using namespace synthux;
using namespace bends;

static DaisySeed hw;
static Touch touch;
static BendsEngine engine;
static BendsController controller;

// External SDRAM stereo delay buffers: 16.0s at 48kHz each (2 x 768,000 samples = 6.14 MB in 64MB SDRAM)
static float DSY_SDRAM_BSS g_sdram_delay_buffer_l[SynthCircuit::kMaxDelay];
static float DSY_SDRAM_BSS g_sdram_delay_buffer_r[SynthCircuit::kMaxDelay];

void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size) {
    engine.ProcessBlock(in[0], in[1], out[0], out[1], size);
    controller.UpdateLedPwm(hw);
}

static void SendTelemetry(DaisySeed& hw, Touch& touch, BendsEngine& engine, uint32_t frame) {
    uint32_t slot = frame % 4;
    if (slot == 0) {
        hw.PrintLine("P %d %d %d %d %d %d %d %d %d %d %d %d",
                     static_cast<int>(touch.pads().Pressure(0) * 100.0f),
                     static_cast<int>(touch.pads().Pressure(1) * 100.0f),
                     static_cast<int>(touch.pads().Pressure(2) * 100.0f),
                     static_cast<int>(touch.pads().Pressure(3) * 100.0f),
                     static_cast<int>(touch.pads().Pressure(4) * 100.0f),
                     static_cast<int>(touch.pads().Pressure(5) * 100.0f),
                     static_cast<int>(touch.pads().Pressure(6) * 100.0f),
                     static_cast<int>(touch.pads().Pressure(7) * 100.0f),
                     static_cast<int>(touch.pads().Pressure(8) * 100.0f),
                     static_cast<int>(touch.pads().Pressure(9) * 100.0f),
                     static_cast<int>(touch.pads().Pressure(10) * 100.0f),
                     static_cast<int>(touch.pads().Pressure(11) * 100.0f));
    } else if (slot == 1) {
        hw.PrintLine("R %d %d %d %d %d %d %d %d %d %d",
                     static_cast<int>(engine.GetRail().EffectiveVcc() * 100.0f),
                     engine.GetRail().IsBrownout() ? 1 : 0,
                     static_cast<int>(touch.knobs().s30() * 100.0f),
                     static_cast<int>(touch.knobs().s31() * 100.0f),
                     static_cast<int>(touch.knobs().s32() * 100.0f),
                     static_cast<int>(touch.knobs().s33() * 100.0f),
                     static_cast<int>(touch.knobs().s34() * 100.0f),
                     static_cast<int>(touch.knobs().s35() * 100.0f),
                     static_cast<int>(touch.knobs().s36() * 100.0f),
                     static_cast<int>(touch.knobs().s37() * 100.0f));
    } else if (slot == 2) {
        hw.PrintLine("D %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld",
                     static_cast<long>(touch.pads().RawDelta(0)),
                     static_cast<long>(touch.pads().RawDelta(1)),
                     static_cast<long>(touch.pads().RawDelta(2)),
                     static_cast<long>(touch.pads().RawDelta(3)),
                     static_cast<long>(touch.pads().RawDelta(4)),
                     static_cast<long>(touch.pads().RawDelta(5)),
                     static_cast<long>(touch.pads().RawDelta(6)),
                     static_cast<long>(touch.pads().RawDelta(7)),
                     static_cast<long>(touch.pads().RawDelta(8)),
                     static_cast<long>(touch.pads().RawDelta(9)),
                     static_cast<long>(touch.pads().RawDelta(10)),
                     static_cast<long>(touch.pads().RawDelta(11)));
    } else {
        hw.PrintLine("U %d %d %d %d %d %d %d %d %d %d %d %d",
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeClk) * 100.0f),
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeRst) * 100.0f),
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeVcc) * 100.0f),
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeOsc) * 100.0f),
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeFlt) * 100.0f),
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeGnd) * 100.0f),
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeDly) * 100.0f),
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeFbk) * 100.0f),
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeAmp) * 100.0f),
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeMix) * 100.0f),
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeIn) * 100.0f),
                     static_cast<int>(engine.Synth().GetNodeVoltage(NodeOut) * 100.0f));
    }
}

static bool g_telemetry_enabled = false;

static void UsbRxCallback(uint8_t* buff, uint32_t* length) {
    if (buff && length && *length > 0) {
        for (uint32_t i = 0; i < *length; ++i) {
            uint8_t c = buff[i];
            if (c == 'b' || c == 'B' || c == 'd' || c == 'D') {
                System::ResetToBootloader(System::BootloaderMode::STM);
            } else if (c == 't' || c == 'T' || c == '1') {
                g_telemetry_enabled = true;
            } else if (c == 'x' || c == 'X' || c == '0') {
                g_telemetry_enabled = false;
            }
        }
    }
}

int main(void) {
    hw.Init(true);
    // Enable Cortex-M7 FPU Flush-to-Zero (FZ) and Default NaN (DN) modes to handle denormals.
    __set_FPSCR(__get_FPSCR() | (1 << 24) | (1 << 25));

    // Zero out SDRAM stereo delay buffers after SDRAM controller initialization
    memset(g_sdram_delay_buffer_l, 0, sizeof(g_sdram_delay_buffer_l));
    memset(g_sdram_delay_buffer_r, 0, sizeof(g_sdram_delay_buffer_r));

    // 64-sample block size (1.33 ms)
    hw.SetAudioBlockSize(64);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    touch.Init(hw);
    engine.Init(hw.AudioSampleRate(), 64, g_sdram_delay_buffer_l, g_sdram_delay_buffer_r, SynthCircuit::kMaxDelay);
    controller.Init(&engine);

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

    // Startup LED sequence
    for (int b = 0; b < 3; ++b) {
        hw.SetLed(true);
        System::Delay(50);
        hw.SetLed(false);
        System::Delay(50);
    }

    hw.StartAudio(AudioCallback);

    uint32_t loop_counter = 0;

    hw.StartLog(false);
    hw.usb_handle.SetReceiveCallback(UsbRxCallback, UsbHandle::UsbPeriph::FS_INTERNAL);

    while (1) {
        bool new_scan = touch.Process();
        controller.Process(hw, touch, loop_counter, new_scan);

        // Only stream telemetry if an external visualizer/host explicitly enabled it with 't'
        if (g_telemetry_enabled && (loop_counter % 16) == 0) {
            SendTelemetry(hw, touch, engine, (loop_counter / 16) % 4);
        }

        loop_counter++;
        System::Delay(1);
    }
}
