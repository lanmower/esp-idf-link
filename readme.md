# ESP-IDF Link

A MIDI synthesizer controller firmware for ESP32 with Ableton Link synchronization, touch-sensitive pads and potentiometer control.

## Quick Build & Flash

**Requirements**: Docker and a USB cable for flashing.

```bash
bash setup.sh
bash build.sh
```

### Flash from Windows

```bash
node flash-ticker.js COM3
```

`flash-ticker.js` drives a real COM port and needs the BOOT/IO0 button held.

```bash
python tools/flash-usbip.py
```

`tools/flash-usbip.py` talks CH341 over USB/IP, so it enters download mode and
flashes hands-free.

### Flash from Linux/WSL2 (`/dev/ttyUSB0`)

```bash
docker run --rm --device=/dev/ttyUSB0 -v "$(pwd)":/project -w /project \
  esp-idf-link:latest \
  bash -c "source /opt/esp/idf/export.sh && idf.py -p /dev/ttyUSB0 flash"
```

Do not use `docker-compose run --rm flash`. Compose builds on this project's
WSL2 + Docker Desktop setup have been observed to hang indefinitely (>30 min)
when only sources changed; `bash build.sh` and the `docker run` form above go
directly to the Docker daemon.

### Flash without a local build

CI builds every push and, on a green `main`, commits `build/link-idf-example.bin`,
`build/bootloader/bootloader.bin` and `build/partition_table/partition-table.bin`
to the branch (and uploads them as the `ticker-firmware-<sha>` artifact). A clean
clone therefore carries flashable images and needs no Docker at all --
`node flash-ticker.js` reads the app offset out of the committed
partition-table binary rather than hardcoding it.

The SPIFFS image that holds the MIDI files is separate: `flash_midi_data.sh`
builds it from `./data` and writes it at `0x317000`, the `storage` partition
offset from `partitions_large.csv`.

See [BUILD_GUIDE.md](BUILD_GUIDE.md) for detailed build instructions and [QUICK_BUILD.txt](QUICK_BUILD.txt) for quick reference.

---

## User Operation Guide

The onboard bassline interpreter (`main/bass_engine.cpp` +
`main/bassline_interpreter.{h,cpp}`) generates a continuously-evolving
64-bar bassline and sends it out as MIDI. It replaced an earlier 8-genre
tap/double-tap lookup-table scheme; there is no discrete genre selection
any more, only continuous interpreted generation. The control surface is
intentionally small -- 4 touch pads + 2 potentiometers, all the hardware
has -- reshaped as 4 dial "banks" of 2 dials each (see
`main/bassline_interpreter.h` for exactly what each dial controls; the design
was validated by audio experiments in the sibling `DawDreamer` repo before it
was ported here).

General Operation
- Pads (0-3): tap a pad to select which bank the 2 pots address. Switches
  *instantly* on press -- no tap/double-tap disambiguation delay. The very
  first pad tap after boot also starts playback, using default parameters
  until you dial something in.
  - Pad 0 -> **Harmony** bank: pot 0 = chord-tone gravity, pot 1 = scale
    colour (banded across 9 modes) / tension tolerance.
  - Pad 1 -> **Groove** bank: pot 0 = energy (groove-template select +
    density, sparse -> busy), pot 1 = swing/syncopation looseness.
  - Pad 2 -> **Motion** bank: pot 0 = pitch contour shape (level, climb,
    arch, wave, zigzag, ...), pot 1 = variation (leap size, octave-jump
    appetite, randomness).
  - Pad 3 -> **Voice** bank: pot 0 = filter-sweep automation depth, pot 1
    = articulation energy (accent/ghost-note/slide density).
- Long-press a pad (>=500ms without releasing): nudges the bassline --
  rerolls the current bar with fresh randomness while keeping the same
  dial-driven character (same "reinterpret while keeping this vibe" idea
  as SUBSTRATE's Nudge button). Takes effect at the next bar boundary, so
  it never clicks or interrupts a note.
- Hold all four pads together: panic-stops playback (fires the instant
  all four are simultaneously held, not a timed hold) and sends MIDI
  CC123 (All Notes Off). Tap any pad afterward to start again.
- No buzzer/LED confirms a bank switch or nudge -- the buzzer is already
  the Link-quantum metronome click (fires every beat via `link_sync.cpp`),
  so a bank switch is a silent context change (the next pot turn shows
  which bank you're in by what changes) and a nudge is audible through
  the bassline itself.

Potentiometer Behaviour Detail
- Smoothing: Uses Exponential Moving Average (EMA) for smoothness.
- Jitter Rejection: Uses a secondary, slower EMA to establish a "stable center". Movement is only registered if the primary smoothed value deviates significantly from this stable center, preventing noise from causing parameter changes while allowing slow, deliberate movements.