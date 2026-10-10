# esp-idf-link — agent notes

`CLAUDE.md` `@`-includes this file. Durable cross-session facts live here, not in
code. Do not extend this by editing `CLAUDE.md`. Compacted 2026-10-10 (was 31 kb;
that version and the memories it superseded are folded in here).

## Mesh with `../aloopprime`: paired invariants (change BOTH or the mesh splits)

One ad-hoc single-AP mesh so Link multicast reaches every device. No credentials:
exactly one device hosts open SSID `ticker`, the rest join as stations. Every value
below exists in BOTH trees; changing one alone splits the mesh silently.

| Invariant | esp-idf-link | aloopprime |
|---|---|---|
| SSID | `main.cpp` `wifi_scan_best_bssid/sta/link_ap("ticker")` | `hostapd.conf`, `wpa_supplicant.conf` `ssid=ticker` |
| Auth | `wifi_connect_sta("ticker","")` open, empty password | open (`key_mgmt=NONE`; `wpa=` commented out) |
| AP addr / DHCP | `wifi_config.cpp` `esp_netif_set_ip_info` `192.168.4.1/255.255.255.0` | `192.168.4.1/24`, dnsmasq `.2-.20` |
| Channel | SoftAP ch6 (`wifi_config.cpp:229`) | `hostapd.conf` `channel=6` |
| Link multicast | `224.76.78.75:20808` | same (hardcoded in Link) |
| Quantum | `main.h:55` `LINK_QUANTUM 16.0` | `link_bridge.cpp` `quantum = 16.0` |
| Start/stop sync | `main.cpp:43` `enableStartStopSync(false)` | same, `false` |
| Host election | lowest MAC/BSSID wins | lowest MAC/BSSID wins |

`PHRASE_BEATS 64.0` (`main.h:56`) is NOT the quantum — it is this project's SPP /
transport-correction boundary (16 bars). Deliberately different; do not "align".

### Host election

Two devices cold-booting can each scan before the other's AP exists, so "nothing
found -> host" makes BOTH host: two isolated L2 domains Link cannot cross. So:
hold `HOLD_MAX_MS = 6000`, strictly monotonic in own STA MAC, rescan 1/s, scan
staggered `mac[5] * 15` ms. `../aloopprime/src/net/autoap.sh` mirrors it (0-6 s,
lowest-wins). Invert the convention here and invert it there in the same change.

Supervisor (`wifi_config.cpp:446`, 2 s cadence, forever):
- STA: dropped link reconnects up to 30 times (~60 s) before re-hosting.
- AP: re-scans; yields to a strictly-lower `ticker` BSSID and drops its AP.
- **Yields only while it has ZERO stations** (`ap_has_associated_stations()`).
  A host that ever accepts a station becomes sticky — so a peer must host and let
  this box join; it must not join this box's SoftAP expecting it to then yield.
- **The comparison MAC is the STA MAC** (`wifi_get_sta_mac()`, here
  `e4:65:b8:77:0d:14`), not the SoftAP MAC (`...:15`). Pi is `e4:5f:01:53:f0:f1`,
  lower, so this box yields to the Pi.
- **Self-BSSID guard reads the interface** (`esp_wifi_get_mac(WIFI_IF_AP)`), never
  an inferred MAC — `esp_read_mac()` reads eFuse and may differ. Without it the box
  yields to itself and flaps AP->STA->AP every tick.

DHCP is on in both roles: AP runs `esp_netif_dhcps_start` on 192.168.4.1 (IDF
default pool starts at .2, so .2-.20 fits); STA netif comes from
`esp_netif_create_default_wifi_sta()` and no `esp_netif_dhcpc_stop` exists in the
tree. Proof of a lease on the ESP side is the `WIFI: IP: 192.168.4.x` line
(`IP_EVENT_STA_GOT_IP`).

**The hosting scan is pinned to the mesh channel.** The election scan needs
`ensure_sta_started()` -> `WIFI_MODE_APSTA`, and an all-channel scan dwells
off-channel, interrupting SoftAP beaconing: a station associating inside a scan
window can sit with no RX and never finish DHCP (Pi report: "associated, rx
packets 0, no lease"). So the supervisor passes `kTickerChannel` — the STA stays
on the channel the AP already beacons on, so there is no dwell. Boot and hold
scans stay all-channel (`kScanAllChannels`): they run before the AP exists.
Cost: a host that left channel 6 would never be seen, so we would never yield to
it. That is the same ch6 pairing the table already rests on.

### By-the-book Link checklist (both trees)

- `captureAppSessionState()` / `commitAppSessionState()` off the audio thread; the
  `AudioSessionState` variants only on it. This tree uses the App variants.
- **Start/stop sync is OFF on purpose: the mesh is clock-only.** A stop on one
  device must not stop the mesh. This tree still CONSUMES transport
  (`isPlaying()` -> Start/Stop/Continue + all-notes-off) and never calls
  `setIsPlaying`. With sync disabled `isPlaying()` reads true all session, so gear
  gets one Start and never a Stop. Re-enabling needs BOTH trees in one change plus
  a flash here.
- The three notification callbacks (`setNumPeersCallback`, `setTempoCallback`,
  `setStartStopCallback`) run on a Link-managed thread, **Realtime-safe: no** —
  bounded logging / atomics only.
- `setTempo` rewrites tempo for EVERY peer. This tree sets it only on an explicit
  LTMP command; aloopprime refuses to propose when peers own it. Neither is an
  unconditional writer.
- **Interface readiness is a real race.** 500 ms settle before constructing Link,
  then IGMP re-asserted every 30 s (15 x 2 s ticks) for the life of the
  association, never a one-shot join: one join at GOT_IP can race netif readiness
  and not stick, and a snooping AP ages out a join that is never repeated.

### The vendored Link is a checked-in FORK, not a submodule

`components/link-esp/link` is a patched copy of Ableton Link committed directly
(1506 tracked files, no nested `.git`). The stale `.gitmodules` entry was removed
because a `git submodule update` would clobber the patch with no compile error and
silently kill ESP peer discovery.

The divergence is the relay hook: `link/include/ableton/platforms/asio/Socket.hpp`
declares `extern "C" wifi_link_multicast_forward(...)` and calls it on every send,
so `wifi_config.cpp` can unicast-copy Link discovery datagrams across the SoftAP
boundary. **Any Link bump must re-apply that hook** — losing it compiles fine and
fails only as silent non-discovery. It already died once that way: the hook's
multicast test read `dstip & 0xff` while asio's `to_uint()` is HOST order (see
asio's own `is_loopback()` masking `0xFF000000`), so 224.76.78.75 read as 75 and
the relay forwarded nothing from the day it was written. The hook is now gated
`if (!g_ap_active) return;` — in STA role it used to unicast-duplicate to the
gateway, which that fix would otherwise have switched on.

No root `.gitmodules`, so `Dockerfile`, `setup.sh`, `.github/workflows/build.yml`
deliberately do NOT init submodules. Do not re-add. Two nested ones survive,
inert: upstream's own under `components/link-esp/link/`, and
`components/link-esp/.gitmodules` (this repo's stale entry, deleted).

### SoftAP multicast gap

ESP32 SoftAP does not carry Link multicast between host and stations, hence
`link_multicast_relay_task`: re-emits each datagram to the group, to the AP's own
IP, and unicast to every associated station, preserving the source IP (Link needs
it for direct peer connect). Whether aloopprime's `brcmfmac` AP has the same gap
is UNVERIFIED (`ap_isolate=0` may suffice) — do not port the relay by assumption.

## Clock is hardware-scheduled, never tick-emitted

Click and 24 ppqn clock come from an `esp_timer` one-shot armed to
`SessionState::timeAtBeat()` of the NEXT pulse, never the 4 kHz gptimer tick
(`LINK_TICK_PERIOD 250` us). A tick only discovers a beat after it passed, so tick
emission costs wake-up latency plus WiFi/lwIP/log contention. Raising the tick
rate cannot tighten the click.

- Link's ESP clock IS `esp_timer_get_time()` — a Link microsecond timestamp is
  already an esp_timer deadline. No offset, no drift correction.
- Lateness instrumented: `MetroStats` (`fired`/`last`/`worst`/`mean`/`rms`) rides
  in the UDP status reply as `metro`.
- Next click's PWM pitch is primed while the buzzer is silent, so the ON edge is
  two duty writes, not an `ledc_timer_config()` on the beat.
- **Skip-don't-catch-up:** the scheduler steps past pulses already due (capped
  `MAX_CATCHUP_PULSES_PER_SCHEDULE = 1024`). A burst reads as a tempo spike to any
  downstream PLL. `MIDI_CLOCK_RESYNC_THRESHOLD` is log-only.
- Same rule on the note path: `BassEngine::process` fires notes in `(last, cur]`;
  past `kMaxAdvanceStepsPerProcess` is a discontinuity, so it releases note-offs,
  re-anchors `m_lastPhrasePosSteps`, emits nothing. **A tempo change does not trip
  it** (`setTempo` rebuilds the timeline from `toBeats(atTime)`, so `beatAtTime`
  stays continuous); only a phase jump (peer join, adoption) does.
- A watchdog re-arms the alarm from the tick when a pulse is really overdue.

**`CONFIG_LWIP_MAX_SOCKETS=16` in `sdkconfig.defaults` is load-bearing**, not
tuning: the default 10 is exhausted by the app's own sockets (Link relay,
LCLK/TTMP broadcast, tempo listener, discovery forward, 4 for HTTP), leaving too
few for Link's per-interface discovery gateway -> EMFILE -> no broadcast -> peers
never discover. 16 is the ESP32 maximum.

## Firmware reaches the ticker over USB only

No OTA, no serial console on this machine.

- CI builds on push to any branch in `espressif/idf:latest` (`Dockerfile:1`) —
  **not a pinned 6.2**. On green `main` it commits the three images under `build/`
  ("ci: update firmware binaries [skip ci]"). Flashable images come from git.
- Offsets: 0x1000 bootloader, 0x8000 partition table, app at the first app
  partition of the **BUILT** table (0x20000 with `partitions_large.csv`). **The
  app offset is DERIVED, never hardcoded** — `flash-ticker.js`, `flash.sh`,
  `tools/flash-usbip.py` read `build/partition_table/partition-table.bin`:
  32-byte entries, LE magic `0x50AA`, then type(1) subtype(1) offset(4) size(4)
  label(16); app type == 0. **Never parse `partitions_large.csv`** — every offset
  but nvs's is blank (the build computes them), so parsing yields `int('')`.
  `--list` refuses to guess when >1 serial port exists; this machine has two
  Bluetooth COM ports that are not the ESP32.
- esptool 5.x: `write-flash` is hyphenated; **`--verify` was dropped** (it always
  verifies — "Hash of data verified"). Passing `--verify` aborts and flashes
  nothing.
- SPIFFS `storage` is flashed separately: `flash_midi_data.sh` builds a `0x19000`
  image from `./data` at **`0x317000`** (derivable: nvs 0x011000, phy_init
  0x017000, app0 0x020000, app1 0x1A0000, storage 0x317000; app partitions round
  up to 0x10000). Re-derive if the layout changes.

### Hands-free flash: `python tools/flash-usbip.py`

`flash-ticker.js` needs BOOT/IO0 held — a WCH/CH340 Windows driver limitation, not
the board. `flash-usbip.py` talks CH341 over USB/IP (usbipd-win) behind a
pyserial-shaped shim, so the WCH driver never loads. `--dry` = enter download
mode, prove sync, reboot; bare = flash and boot. Measured: exit 0, ~49 s, app in
~19.8 s at 460800 (~510 kbit/s), "Hash of data verified", boots `boot:0x13`. No
BOOT hold, no WSL.

Four counter-intuitive facts make it work:

1. **The D1 R32's auto-reset is DIFFERENTIAL.** EN low only at (DTR# high, RTS#
   low) -> `0x40`; IO0 only at (DTR# low, RTS# high) -> `0x20`; `0x60` or `0x00`
   conducts NEITHER — the opposite of esptool's hard-coded convention, hence every
   esptool reset gave `boot:0x13`. Entry is the flip **0x40 -> 0x20** (`boot:0x3`);
   leave with `0x40` then `0x00`.
2. **The CH341 withholds the first replies.** One sync frame gets no answer for
   seconds, then past replies surface behind a later FULL-SIZE OUT; ~20 ms after
   that. esptool sends one frame on a 0.1 s deadline, so prime with repeated
   framed syncs (`SYNC_TIMEOUT` 2 s); typically 2-8.
3. **The first attach back often fails** — usbipd keeps the device claimed after a
   disconnect, so the first `OP_REP_IMPORT` returns `status=4`; `open_port()`
   retries (10 x 6 s) and the second lands.
4. **`USBIPD_BUSID` in `tools/ch341.py` is THIS MACHINE's busid (`2-2`)**, not a
   board property. Re-read per host and after any replug; `usbipd bind --busid
   <busid>` once per boot or nothing attaches.

It passes **`--after no-reset-stub`, never `--after no-reset`**, and guards
`esptool.main()` with `except BaseException` (commit `09da1052`): `no-reset`
calls `soft_reset(True)` at the flash baud while the ROM speaks only 115200 ->
intermittent `FatalError: No more data to read` AFTER all three images verified,
and `FatalError` is not `SystemExit`, so `except SystemExit` lets EN go unpulsed.

**Hands-free serial read:** `UsbipPort()` from `tools/usbip-port.py` MUST have
`.init()` called after construction (else 4096 zero bytes per bulk IN floods the
output). Then `hs(HOLD_EN)` -> short drain -> `hs(RELEASE_BOTH)` resets the board;
`drain(seconds)` returns the boot log. Hyphen in the filename: import with
`importlib.util.spec_from_file_location`, not `import`.

Baud: the ROM only speaks 115200, so `enter_and_sync` pins it (`ROM_BAUD`);
`--baud` applies only after esptool's RAM stub re-times the port. **USB/IP path
only:** 115200 -> 72.2 s (139.9 kbit/s) vs 460800 -> 19.4 s (519.9 kbit/s);
460800 flaked once the same way and a plain re-run succeeded, so that `FatalError`
is a transient attach flake, not baud-specific. 921600 dies right after "Changed." with `FatalError` from `flash_begin`: nothing
written, chip left in the stub; `--dry` recovers it. **921600 is UNMEASURED on
`flash-ticker.js`'s real COM port** (PRD `flash-ticker-js-921600-unmeasured`).
esptool leaves the port at the flash baud, so `boot_app()` resets to `ROM_BAUD`
before pulsing EN or no `boot:` line appears.

Layers: `tools/ch341.py` (raw USB/IP; `SET_CONFIGURATION` first or bulk IN is
dead), `tools/usbip-port.py` (bulk OUT + pyserial surface), `tools/flash-usbip.py`
(entry, priming, esptool); `tools/usbip-boot-entry.py` and
`tools/usbip-sync-latency.py` are the instruments behind facts 1-2, uncalled.
Benign: `flash_id()` reads `0xFFFFFF` in download mode, so esptool prints "Failed
to communicate with the flash chip" and auto-detects no size — erase/program still
go through the ROM.

## Mesh UDP protocol and MIDI emission

All payloads little-endian `int64`.

| Port | Direction | Payload | Purpose |
|---|---|---|---|
| 20810 | esp -> group ~50 Hz | `LCLK` + link-clock micros | phase for peers behind a unicast-RX wall |
| 20811 | looper -> esp | `LTMP` + microsPerBeat (+ beat0 micros + quantum microbeats) | looper sets tempo and phase |
| 20812 | esp -> group ~10 Hz | `TTMP` + microsPerBeat + beatOrigin microbeats + timeOrigin micros | esp -> looper timeline |
| 20812 | request/response | any datagram -> one-line JSON | status: peers/bpm/playing/beat/phase/quantum/ap/metro |

- **Every multicast send must iterate the netifs and set `IP_MULTICAST_IF` per
  interface** (`esp_netif_next_unsafe`). A raw socket with no `IP_MULTICAST_IF`
  exits the wrong interface on a dual-netif device — Link's own socket reached the
  peer (it sets egress) while ours did not.
- **The looper sits behind a unicast-RX wall**: its bcm4343 delivers multicast but
  not unicast-to-self, so Link's ping/pong never completes there (peers=0). LCLK
  gives it phase; TTMP carries tempo back. Standard Link apps ignore all 3 ports.
- Clock broadcast rate-limited to one packet per 20 ms (timeline 100 ms): a 4 kHz
  flood saturates the Pi's single radio-RX drain and starves its control plane.
- Tempo bounded to Link's real range **20..999 bpm** (Ableton Test Plan TEMPO-4
  names both ends). A 400 ceiling silently dropped legitimate tempos.
- `beatAtTime` is quantum-insensitive, `phaseAtTime` is not — the timeline
  broadcast uses `LINK_QUANTUM`.
- LTMP/phase requests are applied on the Link task, not the listener task:
  `captureAppSessionState`/`commitAppSessionState` need a single owner.
- Status is request/response, not broadcast: free when idle, cannot pollute the
  Link multicast group.

MIDI emission — one path, no per-device clock code. Byte values are named
constants in `main.h` (`0xF8` clock, `0xFA` start, `0xFC` stop, `0xFB` continue,
`0xF2` SPP, CC123 all-notes-off). Rates `MIDI_PULSES_PER_QUARTER_NOTE = 24` and
`MIDI_SPP_UNITS_PER_BEAT = 4` live in `link_sync.cpp:17`, NOT `main.h`.

- Continuous 24 ppqn clock; SPP + Start/Continue ONLY at the 16-bar phrase
  boundary; Stop + CC123 on all 16 channels on transport stop and on peer loss.
  SPP counts MIDI beats (sixteenths): 1 Link beat = 4 units, 14-bit, LSB first.
- Realtime bytes are single-byte and may legally interleave a running-status
  message, so the buzzer path and the clock path never corrupt each other.
- Note-offs as velocity 0; CC123 on stop is what keeps RC-505 MK2 loops from
  sticking.
- Target behaviour constraining the design: KO2 locks to ext clock, needs Start,
  is SPP-sensitive (hence one SPP per phrase); Volca Drum follows the pulse and
  ignores SPP; MicroKorg/MiniNova arp, delay and LFO need it non-bursting; Micron
  needs Start and stays in phrase when Start is phrase-aligned; RC-505 MK2 locks
  to clock+Start and repositions on SPP.
- Start is deferred to the next phrase boundary. `s_transport_running` records
  what gear was last TOLD (Start vs Continue) — it is not `isPlaying()`.
- Force-start waits 8 s, not 5 s, so two co-booting devices can discover each
  other before either free-runs at an independent phase.

## Bass generator: 8 approaches, per-bar shaping

`main/state_machine.cpp` maps each of the 4 touch pads to two approaches —
single tap / double tap (`DOUBLE_TAP_WINDOW_US = 300000`):

| Pad | Single | Double |
|---|---|---|
| 0 | `APP_ROLL` | `APP_ACID` |
| 1 | `APP_FUNK` | `APP_GLITCH` |
| 2 | `APP_STAB` | `APP_DRIFT` |
| 3 | `APP_PEDAL` | `APP_BREAK` |

`setApproach()` is the ONLY thing that activates the engine (`m_active`) — nothing
plays until a pad is tapped. It also sets `m_activeBank = approach / 2`.
While already active it sets `m_regenPending`, consumed in `process()` at the next
bar boundary (`bass_engine.cpp:601`), so a switch lands within one bar; a fresh
activation regenerates immediately.

**Every bar is shaped, not repeated.** `regeneratePhrase()` used to replay four
16-step motifs verbatim across all 64 bars — that is what made everything sound
like a ringtone. Each bar now goes through `shapeBar()`, driven by
`ApproachShape` per approach: density, gate, octaveJumpProb, syncTarget,
mutatePerBar, velDrift, filtSweepDepth, timingSteps, rolling, questionAnswer,
pickups. Phrase log: `Phrase[%d] approach=%d bank=%d scale=%s prog=%d notes=%d
bars=64`.

Input timing (verified, not assumed): `update_input_state()` runs off the 250 us
Link tick, so `DEBOUNCE_COUNT = 2` settles a press or release in ~500 us — the
300 ms double-tap window is ~600x wider than needed. No widening required.
`DOUBLE_TAP_TIME_MS = 300` / `HOLD_TIME_MS = 200` in `main.h` are DEAD — nothing
reads them; they are not the gesture timings.

## HTTP clip server (8080) and SPIFFS

`main/network_midi.cpp` IS in the build. `network_midi_init()` starts
`esp_http_server` on **8080** from `app_main`: `POST /upload/*` ->
`/spiffs/loops/<name>`, `GET /info` -> `{"device_ip":...}`, `POST /clear` deletes
every `*.mid`. Invisible at the call sites: the server reserves **4 sockets** of
the 16, and the SoftAP allows **8 stations** (`cfg.ap.max_connection = 8` =
`MAX_AP_STA_IPS`) — 8 is also all the forwarder can unicast to, so a 9th
associates but never gets Link traffic. Power-save is off (`WIFI_PS_NONE`): a
dozing station misses multicast. Do not re-enable.

**SPIFFS IS mounted and both storage endpoints are LIVE** (commit `9568578`;
`mount_clip_storage()` called first from `network_midi_init`; `spiffs` in
`main/CMakeLists.txt` REQUIRES). Proven on-device:

    I (15802) NETWORK_MIDI: Clip storage mounted at /spiffs: 46937 of 86846 bytes used
    I (15802) NETWORK_MIDI: Network MIDI server initialized on 192.168.4.1:8080

`format_if_mount_failed = false` on purpose — mounting must never format. Success
is quiet, failure logs `ESP_LOGE`. Uploads: **503** unmounted, **413** body over
`total - used`, **400** bad/empty name or body. The URI wildcard is one segment and
re-checked at runtime (`is_single_clip_name`), so `/` and `..` cannot escape
`/spiffs/loops`. Empty bodies rejected before `fopen`; incomplete upload unlinks
the partial file and answers 500; `new` is `std::nothrow` under
`CONFIG_COMPILER_CXX_EXCEPTIONS=y`; `/info` re-reads the IP per request (the netif
is recreated on every AP<->STA role change); `network_midi_init()` is idempotent.

Open, not code: no auth on an open SSID; `network_midi_start()`/`_stop()` both
dead. Open in code: **`filepath` truncation is unchecked in `clear_handler`**
(checked only in upload) — 512 bytes vs `CONFIG_HTTPD_MAX_URI_LEN=8192`. Not
defects: embedded NUL (`%s` stops at it — shortened path, never overflow);
esp_http_server runs all sessions in one task, so handlers cannot interleave.
`/clear` filters `d_type == DT_REG`, which SPIFFS VFS may leave `DT_UNKNOWN`, and
`strstr(d_name, ".mid")` matches anywhere, so `notes.midi` dies too.

## MIDI file player (`main/midi_file.cpp`) — NOT in the build

- Parser relies on **MIDI running status**; note-on with **velocity 0 IS a
  note-off**; a note held at end-of-track is released implicitly.
- `process()` walks notes in `startBeat` order and early-breaks — **the sort order
  is load-bearing**, not cosmetic.
- Trigger window 0.03 beat; `playedNotes`/`sentCCs` dedup fires each once.
- Loop length rounds to the nearest quantum within 0.1, else up (ceil).
- **SPIFFS has no real directories**, so `setFolder()` cannot `opendir` a
  subfolder — it scans `/spiffs` and matches a name prefix.
- `updateTempo`'s 120 bpm reference and 0.5 bpm threshold are dormant
  (`syncToBpm` is false — enabling it double-scaled `playbackRate` and corrupted
  `endBeatAbsolute` on any tempo change, leaving notes stuck).
- Warts, left: `parseFile()` closes the file twice; the built-in default's `MTrk`
  declares 19 bytes but writes 12, header says format 1.

## Wiring

GPIO numbers from `main/main.h` — a pin number cannot be expressed in code that
reads the macro.

| Function | Pad / channel | GPIO |
|---|---|---|
| Touch 1 | `TOUCH_PAD_NUM0` | 4 |
| Touch ARP | `TOUCH_PAD_NUM5` | 12 |
| Touch REV | `TOUCH_PAD_NUM6` | 14 |
| Touch FILT | `TOUCH_PAD_NUM7` | 27 |
| Pot 1 | `ADC_CHANNEL_6` | 34 |
| Pot 2 | `ADC_CHANNEL_0` | 39 |
| Buzzer (LEDC) | `BUZZER` | 13 |
| MIDI UART2 TX / RX | `MIDI_TX_PIN` / `MIDI_RX_PIN` | 17 / 16 |

Touch pads use the **legacy `driver/touch_pad.h` API** (IDF also ships
`touch_sensor`). A pad reads LOW when touched; ARP is read first so its press
latency stays lowest. Pots are raw ADC 0-4095 -> MIDI 0-127, with a slow EMA
tracking the stable center beside the fast one used for control. In the MIDI file
player **pot1 = note length (0..2), pot2 = velocity (0..1)**; the accessors are
still `getPot1Value`/`getPot2Value` despite the names.

## Constants whose constraint is invisible in the code

- `MAX_ARP_INDEX_WRAP = 128` (`arp_constants.h`) is referenced nowhere. If wired
  up it must stay **larger than the longest reasonably expected progression** —
  that bound, not 128, is the requirement.
- **E-Slew must stay derivable from sheer, anchored at 104.** `reset` sets sheer to
  0, so a derivation returning anything but 104 at sheer 0 makes the reset
  overwrite what `setSidechainPattern()` just sent. `gateESlewForSheer(sheer)` =
  `kGateESlewDefault + sheer*kHeadroomAboveDefault/kSheerMax` (104 at sheer 0 ->
  127). The old `64 + sheer/2` gave 64. 104 is not magic; sheer 0 is.
- **Gate wet/dry must equal depth, not `127 - depth`.** `gateWetDryForDepth()`
  returns clamped depth and `SIDECHAIN_DEFAULT_DEPTH == SIDECHAIN_DEPTH_MAX ==
  127`, because MiniNova **CC 91 is wetness with 127 = full Gator** (User Manual
  v1.01: the Slot's FX Amount "needs to be at maximum - 127"). The old
  `127 - depth` paired the deepest ducking with a bypassed gate.
  `setFxSlot1Level()` sends the same CC 91. All outside SRCS — none of it runs
  on-device today.
- `kScales[3]` is `{0,3,5,7,10,3,5}` while `kScaleLens[3] == 5`: the trailing
  `{3,5}` is deliberate padding so the row matches its neighbours; only the first
  five are read. Do not "fix" it.
- `kRegisterSpan = 15` (semitones, `bassline_interpreter.h`) must track the
  `anchorMotif` clamp in `bass_engine.cpp` — recorded nowhere else, and neither
  file includes the other.
- `kProgs` rows are **scale-degree indices**, not semitones and not MIDI notes
  (corrected in `c0fc974`). Intents nowhere else: `{0,5,3,6}` i-VI-iv-VII,
  `{0,6,5,6}` i-VII-VI-VII, `{0,3,6,2}` i-iv-VII-III, `{0,5,6,4}` i-VI-VII-V
  ("dark cadence"), `{0,2,6,3}` i-III-VII-iv, `{0,0,5,6}` the pedal row.

## Synth CC/NRPN facts not derivable from the code

- **MicroKorg**: LFO2 shape is CC 75 (`kCcLfo2Shape`); delay sync is CC 13
  (`kCcDelaySync`), 0 = off, 1..32 are sync values.
- **MiniNova**: `LFO2_SYNC_ENABLE` NRPN (MSB 1, LSB 40) is an **assumption never
  confirmed on hardware** — the one NRPN with no verified source. Filter bypass
  (NRPN 0/60 value 0) is deliberately NOT sent by `deactivateFilter()`, which
  unpatches only the LFO.
- **MiniNova "Table 3"** is the source of `setDelaySyncRate` / `setLfoRateSync`;
  `main/lfo_constants.h` carries only a **subset** of the LFO rate-sync row (NRPN
  0/86) — not exhaustive.
- `SynthInterface` contracts only in the implementations: `activateDelay()` /
  `activateReverb()` select Delay 1 / Reverb 1 in **FX Slot 1**;
  `setFxSlot1Level()` is **CC 91**; `activateFilter()` implicitly selects **LP24**;
  the LFO methods assume **LFO2 -> Filter1 Freq via Mod Matrix Slot 1**.
- **LFO depth is bipolar**: `-64..+63` maps to `0..127` with **64 = zero**, so
  `unpatchLfoFromFilter()` writes 64, not 0.
- Note-off velocity 0 is the **default**, not a call-site convention:
  `sendNoteOff(uint8_t note, uint8_t velocity = 0)` is declared identically in
  `synth_interface.h`, `synth_microkorg.h`, `synth_mininova.h`. The two surviving
  `= 64` defaults (`activateFilter` cutoff, `patchLfoToFilter` depth) are
  legitimately 64 and must stay.

## Build shape

`main/CMakeLists.txt` SRCS names **11** TUs: `main.cpp`, `link_sync.cpp`,
`io_helpers.cpp`, `input_handler.cpp`, `synth_mininova.cpp`,
`synth_microkorg.cpp`, `state_machine.cpp`, `bass_engine.cpp`,
`bassline_interpreter.cpp`, `wifi_config.cpp`, `network_midi.cpp`.
**NOT** in the build: `effect_arp.cpp`, `effect_filter.cpp`, `effect_handler.cpp`,
`effect_sidechain.cpp`, `midi_file.cpp` — nothing `#include`s a `.cpp`, so that
cluster is reachable only from itself (`effect_handler.h` only by the four effect
`.cpp`s; `midi_file.h` only by `effect_arp.h` and `midi_file.cpp`). Edits there
cannot break the build; adding them to SRCS is a behaviour change.

`bassline_interpreter` must stay **host-compilable**: it includes only its own
header plus `<algorithm>` `<cmath>` `<cstring>` — no ESP-IDF headers — so
`g++ -std=c++17` exercises its DP and scale logic off-device, the only way to. On
MinGW also pass `-D_USE_MATH_DEFINES` or `M_PI` is undefined.

## CI autobump: how it commits, and its hazard

`.github/workflows/build.yml` commits the three `.bin`s and pushes them: it
**records the sha it built**, then `git reset --hard FETCH_HEAD`, restores the
three `.bin`s from that sha and commits with a 3-path pathspec, up to 5 attempts.
**`reset --soft` is NOT usable**: it moves HEAD to the fetched tip but leaves the
index on the tree this checkout built, so the commit diffs `built_tree` minus
`remote_tip` and silently reverts every source commit in between. A rebase would
MERGE the previous `.bin`s and die on "Cannot merge binary files".

That was the live failure mode, fixed at `bc633fc` (merge `3b9bcd3`): of 76
autobump commits stat-checked, **16 touched a non-`.bin` path**, all before
`bc633fc`; every one since (`ffa49c7`, `4caca07`, `b104d79`, `e667d4e`) is
bins-only. `49cd311` removed 264 lines of `AGENTS.md` and 60 of
`network_midi.cpp` while shipping two `.bin`s; `9eb06bd` reverted 13 paths;
`1c3fcb1`/`4f06700` and `8aec1f0`/`28268b5` are exact inverse mirror pairs — that
is how a whole commit silently vanished.

**Residual rule: any commit touching a non-`.bin` path must be re-verified against
HEAD after a pull** — a green build never proves a change survived. Because CI
rewrites the `.bin`s on `main` on nearly every push, the remote moves between
commit and push routinely; `remote_moved` is the normal outcome, not an error.
Push with `git_push {recover_remote_moved: true}`.

`../aloopprime` is the opposite: a **fetch-only checkout** (`origin.pushurl =
no-push`, no local git identity), so `git_finalize` there commits and the push is
refused. Local-only commits are the intended end state — do not "fix" it by
editing its `.git/config` or adding a remote.

## ESP-IDF 6.x breaks already absorbed

Historical — none visible in current source, each silently re-breaks on a bump.

- `esp_netif_next()` -> `esp_netif_next_unsafe()` in 6.1
  (`components/link-esp/link/.../esp32/ScanIpIfAddrs.hpp`).
- Legacy `driver/adc.h` removed in 6.1; `main/io_helpers.cpp` carries a
  `hall_sensor_read()` stub where that include used to be.

## Comment sweep (INVARIANT 3)

`main/` is **comment-free**: 42 files, 7857 lines, 0 matches for `^\s*(//|/\*)`.
The last block (`main/wifi_config.cpp:121-124`) was removed in `0c23634`. Any new
comment must become self-explanatory code; what cannot goes here, after checking
it is still relevant and factual. Re-count before quoting these numbers.

## gm usage gotchas (folded in from memory)

- `prd-add` **rejects a row with no `id`** — pass `body.id` explicitly.
- `git_finalize` params go inside `body` as JSON
  (`{"message":..., "paths":[...], "recover_remote_moved": true}`) plus a
  top-level `session_id`; top-level `paths`/`message` yields
  `blanket_stage_refused`.
- gm `bash` takes `raw_body` when the JSON `body` key arrives mangled.
- `codesearch` takes `query`/`mode`/`path`/`exhaustive`/`max_results` in `body`;
  regex mode honours the pattern as written.
