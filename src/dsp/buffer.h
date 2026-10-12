#pragma once

#include <cstddef>
#include <cmath>
#include <algorithm>

namespace bends {

class StereoDelayBuffer {
public:
    void Init(float* buf_l, float* buf_r, size_t max_size) {
        _buf_l = buf_l;
        _buf_r = buf_r;
        _max_size = max_size;
        _write_pos = 0.0f;
        _clear_idx = 0;
        _clear_remaining = 0;
    }

    void Clear() {
        _write_pos = 0.0f;
        _clear_idx = 0;
        _clear_remaining = _max_size;
        // Fast synchronous mute of immediate window to kill audio tail instantaneously
        if (_buf_l && _buf_r && _max_size > 0) {
            size_t wipe = std::min(_max_size, static_cast<size_t>(512));
            std::fill_n(_buf_l, wipe, 0.0f);
            std::fill_n(_buf_r, wipe, 0.0f);
        }
    }

    inline void ServiceIncrementalClear(size_t chunk_size = 4096) {
        if (_clear_remaining > 0 && _buf_l && _buf_r) {
            size_t to_clear = std::min(chunk_size, _clear_remaining);
            if (_clear_idx + to_clear <= _max_size) {
                std::fill_n(&_buf_l[_clear_idx], to_clear, 0.0f);
                std::fill_n(&_buf_r[_clear_idx], to_clear, 0.0f);
                _clear_idx += to_clear;
            } else {
                size_t part1 = _max_size - _clear_idx;
                std::fill_n(&_buf_l[_clear_idx], part1, 0.0f);
                std::fill_n(&_buf_r[_clear_idx], part1, 0.0f);
                size_t part2 = to_clear - part1;
                std::fill_n(&_buf_l[0], part2, 0.0f);
                std::fill_n(&_buf_r[0], part2, 0.0f);
                _clear_idx = part2;
            }
            _clear_remaining -= to_clear;
        }
    }

    inline float ReadL(float delay_samples) const {
        if (!_buf_l || _max_size == 0) return 0.0f;
        float d = std::clamp(delay_samples, 1.0f, static_cast<float>(_max_size - 2));
        float max_d = static_cast<float>(_max_size);

        float read_pos = _write_pos - d;
        while (read_pos < 0.0f) read_pos += max_d;
        while (read_pos >= max_d) read_pos -= max_d;

        size_t idx0 = static_cast<size_t>(read_pos);
        if (idx0 >= _max_size - 1) idx0 = _max_size - 2;
        size_t idx1 = idx0 + 1;
        float frac = read_pos - static_cast<float>(idx0);

        return _buf_l[idx0] + frac * (_buf_l[idx1] - _buf_l[idx0]);
    }

    inline float ReadR(float delay_samples) const {
        if (!_buf_r || _max_size == 0) return 0.0f;
        float d = std::clamp(delay_samples, 1.0f, static_cast<float>(_max_size - 2));
        float max_d = static_cast<float>(_max_size);

        float read_pos = _write_pos - d;
        while (read_pos < 0.0f) read_pos += max_d;
        while (read_pos >= max_d) read_pos -= max_d;

        size_t idx0 = static_cast<size_t>(read_pos);
        if (idx0 >= _max_size - 1) idx0 = _max_size - 2;
        size_t idx1 = idx0 + 1;
        float frac = read_pos - static_cast<float>(idx0);

        return _buf_r[idx0] + frac * (_buf_r[idx1] - _buf_r[idx0]);
    }

    inline void Write(float sample_l, float sample_r, float speed) {
        if (!_buf_l || !_buf_r || _max_size == 0) return;
        size_t idx = static_cast<size_t>(_write_pos);
        if (idx >= _max_size) idx = 0;
        _buf_l[idx] = sample_l;
        _buf_r[idx] = sample_r;

        Advance(speed);
    }

    inline void Advance(float speed) {
        if (_max_size == 0) return;
        _write_pos += speed;
        float max_d = static_cast<float>(_max_size);
        if (_write_pos >= max_d) {
            _write_pos -= max_d;
        } else if (_write_pos < 0.0f) {
            _write_pos += max_d;
        }
    }

    float WritePos() const { return _write_pos; }
    size_t MaxSize() const { return _max_size; }

private:
    float* _buf_l{nullptr};
    float* _buf_r{nullptr};
    size_t _max_size{0};
    float  _write_pos{0.0f};
    size_t _clear_idx{0};
    size_t _clear_remaining{0};
};

} // namespace bends
