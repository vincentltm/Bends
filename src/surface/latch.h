#pragma once

#include <cstdint>
#include <functional>
#include <bitset>

namespace synthux {

/**
 * Chord / Note Latch Manager
 * 
 * Manages sustain pedal and chord latching behavior:
 * - When latch is ON: played notes sustain. Touching a new set of notes releases
 *   the previous chord and latches the new one.
 * - When latch is turned OFF: releases any notes that are not physically held.
 */
template <uint8_t note_count = 12>
class Latch {
public:
    Latch() : _on(false) {
        _note_on.reset();
        _note_hold.reset();
    }
    ~Latch() = default;

    bool IsOn() const { return _on; }

    void SetOn(const bool on) {
        bool was_on = _on;
        _on = on;
        if (was_on && !on) {
            for (uint8_t i = 0; i < note_count; i++) {
                if (_note_hold.test(i) && !_note_on.test(i)) {
                    if (_on_note_off) _on_note_off(i);
                    _note_hold.reset(i);
                }
            }
        }
    }

    void Toggle() {
        SetOn(!_on);
    }

    void NoteOn(const uint8_t num) {
        if (num >= note_count) return;
        _note_on.set(num);

        if (_note_on != _note_hold) {
            for (uint8_t note = 0; note < note_count; note++) {
                if (_note_hold.test(note) && !_note_on.test(note)) {
                    if (_on_note_off) _on_note_off(note);
                    _note_hold.reset(note);
                }
            }
        }

        if (!_note_hold.test(num)) {
            if (_on_note_on) _on_note_on(num);
            _note_hold.set(num);
        }
    }

    void NoteOff(const uint8_t num) {
        if (num >= note_count) return;
        if (!_on) {
            if (_on_note_off) _on_note_off(num);
            _note_hold.reset(num);
        }
        _note_on.reset(num);
    }

    void SetOnNoteOn(std::function<void(uint8_t)> cb) { _on_note_on = cb; }
    void SetOnNoteOff(std::function<void(uint8_t)> cb) { _on_note_off = cb; }

    bool IsHolding(uint8_t num) const { return (num < note_count) ? _note_hold.test(num) : false; }
    bool HasHoldingNotes() const { return _note_hold.any(); }

private:
    std::function<void(uint8_t)> _on_note_on;
    std::function<void(uint8_t)> _on_note_off;

    std::bitset<note_count> _note_hold;
    std::bitset<note_count> _note_on;
    bool _on;
};

} // namespace synthux
