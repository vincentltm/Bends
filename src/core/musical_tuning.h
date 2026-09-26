#pragma once

#include "daisysp.h"
#include <cmath>
#include <algorithm>

namespace bends {

class MusicalTuning {
public:
    static constexpr float kRootD0 = 18.354f;
    static constexpr float kRootD1 = 36.708f;

    // D Pentatonic / Modal scale in semitones (Root, 2nd, m3rd, 4th, 5th, 6th, m7th, 8ve)
    static constexpr float kDScale[] = {
        0.0f, 2.0f, 3.0f, 5.0f, 7.0f, 9.0f, 10.0f, 12.0f
    };
    static constexpr size_t kScaleLen = sizeof(kDScale) / sizeof(kDScale[0]);

    static inline float NearestScaleSemitone(float semi) {
        float oct = std::floor(semi / 12.0f);
        float pc = semi - oct * 12.0f;

        float best_d = 999.0f;
        float best_note = 0.0f;
        for (size_t i = 0; i < kScaleLen; ++i) {
            float d = std::abs(pc - kDScale[i]);
            if (d < best_d) {
                best_d = d;
                best_note = kDScale[i];
            }
        }
        return oct * 12.0f + best_note;
    }

    // Pull pitch towards the nearest note in the scale
    static inline float SnapToScale(float semi, float strength) {
        if (strength <= 0.001f) return semi;
        float target = NearestScaleSemitone(semi);
        float delta = daisysp::fclamp(semi - target, -1.0f, 1.0f);
        return semi - (strength * 0.5f / 3.14159265f) * std::sin(3.14159265f * delta);
    }

    // Glides when moving fast, snaps in tune when stopping
    struct ScaleSnapper {
        float current_hz{110.0f};
        float prev_target{110.0f};
        float velocity{0.0f};
        float snap{0.0f};

        void Init(float hz) {
            current_hz = prev_target = hz;
            velocity = snap = 0.0f;
        }

        float Process(float target_hz, float max_snap = 0.72f, float slew = 0.05f) {
            float speed = std::abs(target_hz - prev_target) / std::max(1.0f, target_hz);
            prev_target = target_hz;
            velocity += 0.08f * (speed - velocity);

            float speed_factor = daisysp::fclamp(velocity * 120.0f, 0.0f, 1.0f);
            snap += 0.03f * (max_snap * (1.0f - speed_factor) - snap);

            float semi = 12.0f * std::log2(std::max(10.0f, target_hz) / kRootD0);
            float tuned_hz = kRootD0 * std::pow(2.0f, SnapToScale(semi, snap) / 12.0f);

            current_hz += slew * (tuned_hz - current_hz);
            return current_hz;
        }
    };

    // Tunes short delay times (< 35ms) to musical scale notes
    struct CombTuner {
        float current_sec{0.010f};
        float prev_target{0.010f};
        float velocity{0.0f};
        float snap{0.0f};

        void Init(float sec) {
            current_sec = prev_target = sec;
            velocity = snap = 0.0f;
        }

        float Process(float target_sec, float max_snap = 0.75f) {
            if (target_sec >= 0.035f) {
                current_sec += 0.025f * (target_sec - current_sec);
                prev_target = target_sec;
                return current_sec;
            }

            float speed = std::abs(target_sec - prev_target) / std::max(0.0001f, target_sec);
            prev_target = target_sec;
            velocity += 0.08f * (speed - velocity);

            float speed_factor = daisysp::fclamp(velocity * 100.0f, 0.0f, 1.0f);
            snap += 0.04f * (max_snap * (1.0f - speed_factor) - snap);

            float comb_hz = 1.0f / target_sec;
            float semi = 12.0f * std::log2(std::max(10.0f, comb_hz) / kRootD1);
            float tuned_sec = 1.0f / (kRootD1 * std::pow(2.0f, SnapToScale(semi, snap) / 12.0f));

            current_sec += 0.035f * (tuned_sec - current_sec);
            return current_sec;
        }
    };

    // Snaps S&H random walk wow into harmonic scale steps (-12, -7, -5, 0, +5, +7, +12 st)
    static inline float QuantizeSHToHarmonic(float raw_bipolar) {
        static constexpr float kHarmonics[] = { -12.0f, -7.0f, -5.0f, 0.0f, 0.0f, 5.0f, 7.0f, 12.0f };
        static constexpr size_t kLen = sizeof(kHarmonics) / sizeof(kHarmonics[0]);
        float norm = daisysp::fclamp((raw_bipolar + 1.0f) * 0.5f * (kLen - 1), 0.0f, static_cast<float>(kLen - 1));
        return kHarmonics[static_cast<size_t>(norm)] / 12.0f;
    }
};

} // namespace bends
