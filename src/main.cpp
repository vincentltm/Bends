#include "daisy_seed.h"
#include "touch/touch.h"
#include "engine/bends_engine_v2.h"
#include "engine/bends_controller_v2.h"

using namespace daisy;
using namespace synthux;
using namespace bends;

static DaisySeed hw;
static Touch touch;
static BendsEngineV2 engine;
static BendsControllerV2 controller;

void AudioCallback(AudioHandle::InputBuffer in,
                   AudioHandle::OutputBuffer out,
                   size_t size) {
    engine.ProcessBlock(in[0], in[1], out[0], out[1], size);
    controller.UpdateLedPwm(hw);
}

int main(void) {
    hw.Init(true);
    // Enable Cortex-M7 FPU Flush-to-Zero (FZ) and Default NaN (DN) modes to handle denormals.
    __set_FPSCR(__get_FPSCR() | (1 << 24) | (1 << 25));

    // 64-sample block size (1.33 ms)
    hw.SetAudioBlockSize(64);
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);

    touch.Init(hw);
    engine.Init(hw.AudioSampleRate(), 64);
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

    while (1) {
        touch.Process();
        controller.Process(hw, touch, loop_counter);

        loop_counter++;
        System::Delay(1);
    }
}
