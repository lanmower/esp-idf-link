# WiFi Provisioning Fix

## Saved But Not Connecting Issue
After provisioning saves credentials, device says "Saved" but never actually connects - hotspot restarts instead.

## Root Causes Fixed
1. **Missing WiFi state reset**: After stopping hotspot, WiFi driver needs time to stabilize before re-initialization
2. **No error checking**: `wifi_config_set_credentials()` wasn't being validated - could fail silently
3. **Short connection timeout**: Only waited 20s for connection, needed 30s for ESP32

## Solutions Applied
- `provisioning.cpp`: Added error checking on credential save, returns 500 if save fails
- `main.cpp`: Added 2s delay after `wifi_stop_hotspot()` for WiFi driver stability, extended timeout to 30s, added logs
- `wifi_config.cpp`: Added detailed error logging at each step (get creds, set mode, set config, start)

## Build Performance Improvement
- `build.sh`: Changed to incremental builds by default (removed unconditional `rm -rf build`)
- Use `./build.sh clean` for full rebuild
- Default builds now leverage Docker layer caching - 10-100x faster for code-only changes

## Files Changed
- `build.sh`: Incremental build support
- `main/provisioning.cpp`: Error handling on credential save
- `main/main.cpp`: WiFi state reset timing, extended timeout, improved logging
- `main/wifi_config.cpp`: Detailed error logging in connect flow

---

# MIDI Compatibility Notes

## NRPN Sequence
`send_midi_nrpn` sends CC99, CC98, CC6, CC38=0, CC101=127, CC100=127. The null reset (CC101/100=127) is mandatory — without it, subsequent CC6 messages are misread as NRPN data on MicroKorg, Micron, and RC-505.

## Note-Off Velocity
Always send velocity 0 for note-off. Passing note-on velocity in note-off byte causes stuck notes on RC-505 MK2 and KO2.

## MIDI File Player Loop Sync
`syncToBpm` is disabled (`false`). Link's `beatAtTime()` already provides tempo-independent beats — enabling `syncToBpm` caused double-scaling of `playbackRate` which corrupted `endBeatAbsolute` on any tempo change, leaving notes permanently stuck. At loop boundary, `sendAllNotesOff()` + `activeNotes.clear()` fires instead of remapping note end times.

## UART TX Buffer
`uart_driver_install` uses 256-byte TX ring buffer (not 0). Without it, burst NRPN sends (e.g. `setSidechainPattern` with 12 NRPNs = 108 bytes) would block the FreeRTOS main loop for ~35ms.

---

# WSL2 CH340 Driver Setup

## Issue
In WSL2 with usbipd, CH340 USB-serial devices are detected by the kernel but don't automatically create `/dev/ttyUSB*` device nodes.

## Solution
The CH341 kernel module must be loaded. This is now configured system-wide:
- Config file: `/etc/modprobe.d/wsl-ch340.conf`
- Loads both `usbserial` and `ch341` modules automatically
- Applies to all WSL2 terminal sessions

## Flash Script Updates
- `flash.sh`: Auto-detects `/dev/ttyUSB*` device if not specified
- `flash.sh`: Uses Python module invocation with PYTHONPATH override to access user-installed esptool (5.1.0)
- `flash.sh`: Requires sudo for /dev/ttyUSB* access
- `setup-ch340.sh`: Guides manual setup via `sudo bash setup-ch340.sh`

## Known Issues & Workarounds
- System esptool 4.7.0 has missing stub_flasher_32.json. Fixed by upgrading to user-installed 5.1.0
- esptool binary doesn't use PYTHONPATH, so script uses `python3.12 -m esptool` instead
- System requires `sudo modprobe ch341` before first use (set up in `/etc/modprobe.d/wsl-ch340.conf`)

## Build Performance Issue (Critical)
- Docker Compose builds hang indefinitely when rebuilding only source files (>30min timeout)
- `docker run` directly also hangs (timeout after 3min)
- Appears to be Docker Desktop / volume mount issue in WSL2
- Workaround: Use pre-built binary and iterate via serial testing
- Root cause: Unknown - possibly Docker Desktop WSL2 integration issue with build caching/volumes
