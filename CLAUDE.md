@AGENTS.md
# WiFi Self-Healing Mesh ('ticker' network)

There is no credential-provisioning flow. Devices form an ad-hoc single-AP mesh around
the open SSID `ticker` so Ableton Link peers can discover each other:

## Boot decision (`main.cpp` app_main)
1. Stagger scan by last STA-MAC byte (0-3825ms) so co-booting devices don't race.
2. `wifi_scan_best_bssid("ticker", ...)` -- active scan filtered to the SSID, reads
   `esp_wifi_scan_get_ap_records` for the lowest BSSID (tie-break key).
3. Match found -> join as STA (`wifi_connect_sta`), wait up to 30s; on failure, host.
   No match -> host the AP (`wifi_start_link_ap`) + Link multicast relay.

## Self-healing supervisor (`wifi_config.cpp` wifi_supervisor_task)
Started by `wifi_start_supervisor("ticker")`, runs forever, 2s cadence:
- STA role: a dropped STA reconnects up to 6 times (~12s); if the host stays gone it
  re-hosts the AP so the mesh survives the host powering off.
- AP role: re-scans; if another `ticker` AP with a strictly-lower BSSID appears (both
  ended up hosting), it drops its AP and joins the lower one -- exactly one host wins.
- `ensure_sta_started()` prevents leaking a default-STA netif across rescans.

## MIDI master-clock sync (`link_sync.cpp`)
Emits continuous 24ppqn clock (capped per tick + hard-resync past 24 clocks behind to
avoid post-stall bursts). SPP + Start/Continue fire ONLY at the 16-bar phrase boundary
(`PHRASE_BEATS=64`, distinct from `LINK_QUANTUM=16`). Stop + All-Notes-Off (CC123) on
transport stop / peer loss. See the per-target compatibility table at the top of
`link_sync.cpp` (KO2, Volca Drum, MicroKorg, Micron, MiniNova, RC-505 MK2).

## Build Performance Improvement
- `build.sh`: Changed to incremental builds by default (removed unconditional `rm -rf build`)
- Use `./build.sh clean` for full rebuild
- Default builds now leverage Docker layer caching - 10-100x faster for code-only changes

---

# MIDI Compatibility Notes

## NRPN Sequence
`send_midi_nrpn` sends CC99, CC98, CC6, CC38=0, CC101=127, CC100=127. The null reset (CC101/100=127) is mandatory -- without it, subsequent CC6 messages are misread as NRPN data on MicroKorg, Micron, and RC-505.

## Note-Off Velocity
Always send velocity 0 for note-off. Passing note-on velocity in note-off byte causes stuck notes on RC-505 MK2 and KO2.

## MIDI File Player Loop Sync
`syncToBpm` is disabled (`false`). Link's `beatAtTime()` already provides tempo-independent beats -- enabling `syncToBpm` caused double-scaling of `playbackRate` which corrupted `endBeatAbsolute` on any tempo change, leaving notes permanently stuck. At loop boundary, `sendAllNotesOff()` + `activeNotes.clear()` fires instead of remapping note end times.

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
