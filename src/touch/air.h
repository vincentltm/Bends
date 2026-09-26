#pragma once

#include "daisy_seed.h"
#include "pads.h"
#include <cmath>
#include <array>

namespace synthux {

/**
 * Capacitive proximity and position tracker for Synthux Touch 2.
 * Tracks hand proximity above pads P00–P09.
 */
class AirTracker {
public:
    struct PadCoord {
        float x; // 0.0 (Far Left) .. 1.0 (Far Right)
        float y; // 0.0 (Bottom) .. 1.0 (Top)
    };

    // Pad positions on the board
    static constexpr std::array<PadCoord, 10> kPadCoords = {{
        { 0.22f, 0.82f }, // P00: Upper Left
        { 0.50f, 0.88f }, // P01: Upper Center
        { 0.78f, 0.82f }, // P02: Upper Right
        { 0.00f, 0.38f }, // P03: Far Left
        { 0.33f, 0.48f }, // P04: Mid-Left
        { 0.50f, 0.36f }, // P05: Center
        { 0.67f, 0.48f }, // P06: Mid-Right
        { 1.00f, 0.38f }, // P07: Far Right
        { 0.15f, 0.08f }, // P08: Lower Left
        { 0.85f, 0.08f }  // P09: Lower Right
    }};

    struct OneEuro {
        float min_cutoff{1.0f}; // Cutoff at rest (Hz) - rock solid pitch
        float beta{0.08f};      // Speed sensitivity - fast tracking when moving
        float d_cutoff{1.0f};   // Derivative cutoff (Hz)
        float x{0.5f};
        float dx{0.0f};
        bool  init{false};

        void Init(float min_fc, float b, float d_fc = 1.0f) {
            min_cutoff = min_fc;
            beta       = b;
            d_cutoff   = d_fc;
            x          = 0.5f;
            dx         = 0.0f;
            init       = false;
        }

        float Process(float val, float dt) {
            if (!init) {
                x = val;
                dx = 0.0f;
                init = true;
                return x;
            }
            float raw_dx = (val - x) / dt;
            float a_d = Alpha(d_cutoff, dt);
            dx += (raw_dx - dx) * a_d;

            float cutoff = min_cutoff + beta * std::abs(dx);
            float a = Alpha(cutoff, dt);
            x += (val - x) * a;
            return x;
        }

        static float Alpha(float cutoff, float dt) {
            float tau = 1.0f / (6.2831853f * cutoff);
            return 1.0f / (1.0f + tau / dt);
        }
    };

    AirTracker() 
        : _enabled(false),
          _has_air(false),
          _x(0.5f), _y(0.5f), _z(0.0f),
          _target_x(0.5f), _target_y(0.5f), _target_z(0.0f),
          _peak_delta(0.0f), _strongest_pad(-1),
          _noise_gate(1.0f),
          _power_exp(1.0f),
          _max_proximity(18.0f) {
        _filter_x.Init(1.2f, 0.12f, 1.0f); // Fast yet rock-solid pitch
        _filter_z.Init(1.5f, 0.08f, 1.0f); // Smooth dynamic volume
        for (size_t i = 0; i < 12; i++) {
            _pad_delta[i] = 0.0f;
            _smooth_prox[i] = 0.0f;
        }
    }

    void Enable(bool enable = true) { _enabled = enable; }
    bool IsEnabled() const { return _enabled; }

    void Process(const Pads& pads) {
        if (!_enabled) return;

        constexpr float kDt = 0.0045f; // ~220Hz main loop rate

        float sum_w = 0.0f;
        float sum_wx = 0.0f;
        float total_prox = 0.0f;
        float max_d = 0.0f;
        int best_pad = -1;

        // 1. Per-pad low-pass filter, continuous soft-knee noise gate & smooth temporal slewer
        for (size_t i = 0; i < 12; i++) {
            float raw_d = static_cast<float>(pads.RawDelta(i));
            // Stage 1: Pre-filter raw delta (eliminates single-sample ADC spikes)
            _pad_delta[i] += (raw_d - _pad_delta[i]) * 0.40f;

            float d = _pad_delta[i];
            float target_p = 0.0f;
            if (d > _noise_gate) {
                float excess = d - _noise_gate;
                // Soft-knee curve with dynamic span scaled by _max_proximity
                float eff = (excess * excess) / (excess + 0.35f);
                float span = _max_proximity * 0.25f;
                if (span < 2.5f) span = 2.5f;
                float norm = eff / span;
                if (norm > 1.0f) norm = 1.0f;
                target_p = norm;
            }

            // Continuous temporal smoothing: fast attack (~18ms), gentle release (~50ms)
            // Completely eliminates any binary step, pop, or click
            float slew = (target_p > _smooth_prox[i]) ? 0.25f : 0.08f;
            _smooth_prox[i] += (target_p - _smooth_prox[i]) * slew;
            if (_smooth_prox[i] < 0.001f) _smooth_prox[i] = 0.0f;

            float eff_prox = _smooth_prox[i];
            total_prox += eff_prox;

            // Pads P01, P03-P09 drive horizontal centroid and air height.
            // P00 and P02 are excluded so touching them does not jump position.
            if (i < 10 && i != 0 && i != 2) {
                if (eff_prox > max_d) {
                    max_d = eff_prox;
                    best_pad = static_cast<int>(i);
                }
                sum_w += eff_prox;
                sum_wx += eff_prox * kPadCoords[i].x;
            }
        }

        _peak_delta = max_d;
        _strongest_pad = best_pad;
        _has_air = (_peak_delta > 0.02f);

        if (_has_air) {
            // Proximity (Height Z)
            float norm_z = _peak_delta;
            if (norm_z > 1.0f) norm_z = 1.0f;
            _target_z = norm_z;

            // Horizontal position (X Centroid across pads P03..P07)
            if (sum_w > 0.05f) {
                float target_x_raw = sum_wx / sum_w;
                float norm_x = target_x_raw;
                if (norm_x > 1.0f) norm_x = 1.0f;
                if (norm_x < 0.0f) norm_x = 0.0f;
                _target_x = norm_x;
                _x = _filter_x.Process(_target_x, kDt);
            }
        } else {
            // Hands fully away: force target Z to silence
            _target_z = 0.0f;
        }

        // Stage 2: One-Euro Adaptive Filtering on Z (Volume)
        _z = _filter_z.Process(_target_z, kDt);
        if (_z < 0.001f) _z = 0.0f;
    }

    // Coordinates
    float X() const { return _x; } // 0.0 (Left) .. 1.0 (Right)
    float Y() const { return 0.5f; } // Fixed mid-plane
    float Z() const { return _z; } // 0.0 (Far away) .. 1.0 (Very close / touch)
    
    // State
    bool HasAir() const { return _has_air; }
    float PeakDelta() const { return _peak_delta; }
    int StrongestPad() const { return _strongest_pad; }

    // Per-pad proximity: 0.0 (hand away) to 1.0 (close/touch)
    // 100% continuous, slewed analog glide with zero binary step
    float PadProximity(size_t i) const {
        return (i < 12) ? _smooth_prox[i] : 0.0f;
    }

    // Tuning
    void SetNoiseGate(float gate) { _noise_gate = gate; }
    float NoiseGate() const { return _noise_gate; }

    void SetPower(float power) { _power_exp = power; }
    void SetMaxProximity(float max_prox) { _max_proximity = max_prox; }

private:
    bool  _enabled;
    bool  _has_air;
    float _x, _y, _z;
    float _target_x, _target_y, _target_z;
    float _peak_delta;
    int   _strongest_pad;

    float _noise_gate;
    float _power_exp;
    float _max_proximity;

    std::array<float, 12> _pad_delta{};
    std::array<float, 12> _smooth_prox{};
    OneEuro _filter_x;
    OneEuro _filter_z;
};

} // namespace synthux
