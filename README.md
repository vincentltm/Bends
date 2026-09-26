# Bends

Bends is a virtual circuit-bent synthesizer and effects processor for the Synthux Touch 2 and Daisy Seed.

Inspired by 90s circuit-bent delays and samplers, Bends lets you short virtual circuit traces with your fingers, coins, or alligator clips to starve power, bleed feedback, bend clocks, and corrupt audio buffers.

<img src="assets/touch.jpeg" width="400"/>

## Origins

Bends started as a card idea for the [Music Thing Modular Workshop Computer](https://www.youtube.com/watch?v=-rACWw0J_zI&t=6s). During the Synthux Academy Residency, it grew into a standalone instrument and effects unit running on the Daisy Seed with the Touch 2 surface, exploring circuit bending without the risk of letting out the magic smoke.

Drawing on Reed Ghazala's concept of body contacts, the hardware uses touch pads and conductive objects (fingers, coins, clips) to short virtual circuits, clock lines, and audio buffers.

---

## Controls

### Faders
- **S36 (Left Fader)**: Starve (Vcc). Pulling down starves the power rail into brownout, adding distortion, motorboating sputters, and unstable pitch.
- **S37 (Right Fader)**: Clock / Pitch. Master pitch and clock over 5 octaves (D0 up to D5). Controls drone pitch, click rates, and glitch timing.

### Knobs
- **S30 (Drive)**: Saturation and input gain for the drone oscillator and hum generators.
- **S31 (Cutoff)**: 2-pole state-variable filter cutoff (55 Hz to 15 kHz).
- **S32 (Resonance)**: Filter resonance and self-oscillation feedback.
- **S33 (Delay Time)**: Delay time from short comb filtering (0.8 ms) up to 2.25 seconds. Turning past halfway drops the clock speed towards 17 kHz for a crunchy, lo-fi digital sound.
- **S34 (Delay Feedback)**: Delay repeats, with soft clipping for singing resonant tails.
- **S35 (Delay Mix)**: Wet / dry balance.

---

## Touch Surface

Touching pads triggers sounds or modulates parameters. Bridging two pads with a finger, coin, or clip shorts circuit connections between them:

- **P00 (Drone Oscillator)**: Pressure opens volume and adds a sub-octave. Air proximity gently opens the filter.
- **P01 (Filter Ping)**: Tapping strikes the resonant filter like a drum; holding adds resonance modulation.
- **P02 (Heartbeat / Click)**: Tapping triggers an acoustic transient kick; holding plays a slow pulse train.
- **P03 (Hum Antenna)**: 50 Hz mains buzz and static crackle.
- **P04 (Starve Sag)**: Draws extra current from the power rail to deepen brownout.
- **P05 (Resonant Feedback)**: Bleeds delay repeats back into the filter.
- **P06 (Delay Clock)**: Bends delay clock time. Bridging P06 to Ground (P08) freezes the delay buffer.
- **P07 (Radio Whine)**: Heterodyne RF whistle that beats against the master clock.
- **P08 (Ground)**: Reference ground plane. Bridging pins to ground shunts signals, drops pitch, or freezes audio.
- **P09 (Stutter)**: Repeats audio slices in rhythmic subdivisions based on pressure. Under starve, triggers digital dropouts.
- **P10 (Pitch Ribbon)**: Continuous upward pitch bend.
- **P11 (Utility Bridge)**: Sends delay feedback directly into the power rail.

---

## Tuning & Scale Snapping

Bends is tuned in the key of **D** ($D_0 = 18.35\text{ Hz}$).

When turning knobs or faders quickly, the pitch glides smoothly. When you stop, it snaps to the nearest note in the scale.

The delay also snaps to musical notes at short delay times ($S33 < 35\text{ ms}$), turning comb resonance into playable notes.

---

## Building & Flashing

```bash
# Build firmware
make -j4

# Flash via USB DFU
# (Hold BOOT, tap RESET, release BOOT)
make program-dfu
```

## License

MIT
