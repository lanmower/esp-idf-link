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

`bash flash.sh` is the current WSL2/Linux script route: it auto-detects
`/dev/ttyUSB*`, derives the app offset from the built partition table, and
runs the user-installed esptool under a PYTHONPATH override. It needs sudo
for the serial device.

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
builds it from `./data` and writes it at `0x317000`. Offsets come from the built
partition table (`flash.sh` derives the app offset with `app_offset()`);
`partitions_large.csv` leaves every offset blank except `nvs`.

See [BUILD_GUIDE.md](BUILD_GUIDE.md) for detailed build instructions and [QUICK_BUILD.txt](QUICK_BUILD.txt) for quick reference.

---

## User Operation Guide

The sequencer is an eight-beat (two-bar) bass loop built from
`main/bass_engine.cpp`, `main/state_machine.cpp` and
`main/control_model.{h,cpp}`. Four touch pads and two potentiometers address
16 settings (`ctl::controlIndex`: eight single-tap settings and eight
double-tap settings). The loop is the phrase; the earlier long-form bassline
and the dial "banks" are gone.

General Operation
- Single tap a pad: select that pad's two single-tap settings and start the
  sequencer (`selectSingle` sets `m_active = true`).
- Double-tap a pad and hold: route both pots to that pad's two double-tap
  settings (`setDoubleHeld`); releasing the pad returns the pots to the
  single-tap settings.
- Dials act only while a pad is held. `state_machine.cpp` calls `setDial`
  only when a pot moved and at least one pad is held, and `setTouchHeld`
  gates recording. With no pad held the pots do nothing.
- A dial move starts a touch (`LoopTrack::startRecording` opens a clean
  line). While a pad is held, `valueAt` writes the live dial value at every
  step the loop passes, including steps where the dial is no longer moving.
  Releasing the pad ends the touch; the last value stays as the fallback for
  steps never recorded.
- Two pads held: fill (`MARK_ADD` on each step passed). Three pads held:
  unfill (`MARK_REMOVE`). Four pads held at once: `resetAll()` clears every
  recording and fill mark, restores every setting's baseline/default,
  releases active notes and stops the sequencer.
- The loop window setting (pad 3 double-tap, pot 0) chooses one or two bars
  (4 or 8 beats) and is read LIVE, never from the track; a shortened window
  takes effect at the next turnaround of the new length. `ctl::kLoopBars = 2`
  and `ctl::kLoopMaxBeats = 8` cap the loop.

Pads are numbered 0-3 as in `ctl::controlIndex`; the print control card
labels them 1-4.

| Pad | Single tap: pot 0 / pot 1 | Double-tap, hold: pot 0 / pot 1 |
|---|---|---|
| 0 | scale / width | gate width / arp speed |
| 1 | pitch offset / 32-step rhythm | random octave / random drop-out |
| 2 | LFO type / LFO speed | LFO width / filter shift (CC 74) |
| 3 | trance-gate series / octave | loop window / chromatic aberration |

Potentiometer Behaviour Detail
- Smoothing: Uses Exponential Moving Average (EMA) for smoothness.
- Jitter Rejection: Uses a secondary, slower EMA to establish a "stable center". Movement is only registered if the primary smoothed value deviates significantly from this stable center, preventing noise from causing parameter changes while allowing slow, deliberate movements.

## Network ports

All ports are UDP. The Ableton Link discovery group is `224.76.78.75`. Each
port carries exactly one protocol, so a request/reply poll can never be read
as a beat message.

| Port  | Direction             | Protocol and payload                                                         | Source                          |
|-------|-----------------------|------------------------------------------------------------------------------|---------------------------------|
| 20808 | Link peers            | Ableton Link discovery                                                       | `main/wifi_config.h`            |
| 20810 | ESP -> group          | `LCLK` clock: 4-byte tag + int64 Link micros. 50 Hz while a status subscriber is active, 1 Hz idle | `LINK_PHASE_PORT`, `broadcast_link_clock` |
| 20811 | group or unicast -> ESP | `LTMP` tempo/phase request, `LRST` session reset. Bound on any address, joins the group | `LINK_TEMPO_PORT`, `tempo_listener_task` |
| 20812 | ESP -> group          | `TTMP` beat origin, 28 bytes: `TTMP` + 3 little-endian int64 (micros per beat, beat origin in microbeats, time origin micros). Nothing replies. 10 Hz while subscribed, 5 s idle. Subscribed by a datagram to 20813 or 20811 within 15 s; a datagram to 20812 does not subscribe | `LINK_TTMP_PORT`, `broadcast_ticker_timeline` |
| 20813 | unicast request/reply | Status: send any datagram of 1 to 64 bytes, get one JSON line back to the sender (`link`, `peers`, `bpm`, `playing`, `beat`, `phase`, `quantum`, `ap`, `metro`, `pot0`, `pot1`) | `LINK_STATUS_PORT`, `status_responder_task` |
| 34999 | ESP -> `g_link_send_dst_ip` | TX probe (diagnostic, rate limited)                                    | `TX_PROBE_PORT`, `main/wifi_config.cpp` |

- A status request also marks a telemetry subscriber for 15 s, which raises
  the `LCLK` and `TTMP` rates in the table above.
- Status used to share port 20812 with `TTMP`. It moved to 20813 so a status
  poll cannot be read as a beat message. Any client that polls status must
  send to 20813.
- Do not assign 20808, 20810, 20811, 20812 or 20813 to a second protocol.