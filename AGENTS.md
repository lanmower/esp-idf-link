# esp-idf-link — agent notes

`CLAUDE.md` `@`-includes this file. Durable cross-session facts live here, not in
code. Do not extend this by editing `CLAUDE.md`. Compacted 2026-10-10; the memories it
supersedes are folded in.

## Mesh with `../aloopprime`: paired invariants (change BOTH or the mesh splits)

One ad-hoc single-AP mesh so Link multicast reaches every device. No credentials:
exactly one device hosts open SSID `ticker`, the rest join as stations. Every value
below exists in BOTH trees; changing one alone splits the mesh silently.

| Invariant | esp-idf-link | aloopprime |
|---|---|---|
| SSID | `main.cpp` `wifi_scan_best_bssid/sta/link_ap("ticker")` | `hostapd.conf`, `wpa_supplicant.conf` `ssid=ticker` |
| Auth | `wifi_connect_sta("ticker","")` open, empty password | open (`key_mgmt=NONE`; `wpa=` commented out) |
| AP addr / DHCP | `wifi_config.cpp` `esp_netif_set_ip_info` `192.168.4.1/255.255.255.0` | `192.168.4.1/24`, dnsmasq `.2-.20` |
| Channel | SoftAP ch6 (`kTickerChannel = 6`, `wifi_config.h`) | `hostapd.conf` `channel=6` |
| Link multicast | `224.76.78.75:20808` | same (hardcoded in Link) |
| Quantum | `main.h` `LINK_QUANTUM 16.0` | `link_bridge.cpp` `quantum = 16.0` |
| Start/stop sync | `main.cpp` `enableStartStopSync(false)` | same, `false` |
| Host election | lowest MAC/BSSID wins | lowest MAC/BSSID wins |

`PHRASE_BEATS 64.0` (`main.h`) is NOT the quantum — it is the SPP /
transport-correction boundary (16 bars). Do not "align" it.

### Host election

Two cold-booting devices can each scan before the other's AP exists, so "nothing
found -> host" makes BOTH host: two isolated L2 domains Link cannot cross. So: hold
`HOLD_MAX_MS = 6000`, strictly monotonic in own STA MAC, rescan 1/s, scan staggered
`mac[5] * 15` ms. `../aloopprime/src/net/autoap.sh` mirrors it (0-6 s, lowest-wins).
Invert the convention here and there in the same change.

Supervisor (`wifi_config.cpp`, 2 s cadence, forever):
- STA: a dropped link reconnects up to 30 times (~60 s) before re-hosting.
- AP: re-scans; yields to a strictly-lower `ticker` BSSID and drops its AP.
- **Yields only while it has ZERO stations** (`ap_has_associated_stations()`) — a host
  that ever accepts a station is sticky, so a peer must host and let this box join; it
  must not join this box's SoftAP expecting it to then yield.
- **The comparison MAC is the STA MAC** (`wifi_get_sta_mac()`, here
  `e4:65:b8:77:0d:14`), not the SoftAP MAC (`...:15`). Pi is `e4:5f:01:53:f0:f1`,
  lower, so this box yields to the Pi.
- **Self-BSSID guard reads the interface** (`esp_wifi_get_mac(WIFI_IF_AP)`), never
  an inferred MAC — `esp_read_mac()` reads eFuse and may differ. Without it the box
  yields to itself and flaps AP->STA->AP every tick (`5faf01a`).
- IGMP is re-joined on yield, not only at boot (`d1b7f9a`). Proof of a lease on the
  ESP side is the `WIFI: IP: 192.168.4.x` line.

**The hosting scan is pinned to the mesh channel.** The election scan needs
`ensure_sta_started()` -> `WIFI_MODE_APSTA`, and an all-channel scan dwells
off-channel, interrupting SoftAP beaconing: a station associating inside a scan window
can sit with no RX and never finish DHCP. So the supervisor passes `kTickerChannel`;
boot/hold scans stay `kScanAllChannels` (before the AP exists). Cost: a host that left
channel 6 would never be seen — the same ch6 pairing the table rests on.

### By-the-book Link checklist (both trees)

- `captureAppSessionState()`/`commitAppSessionState()` off the audio thread; the
  `AudioSessionState` variants only on it. This tree uses the App variants.
- **Start/stop sync is OFF on purpose: the mesh is clock-only** (`b94ac2d`). With sync
  off `isPlaying()` reads true all session, so gear gets one Start and never a Stop.
  Re-enabling needs BOTH trees in one change plus a flash here.
- The three notification callbacks run on a Link-managed thread, **Realtime-safe: no**
  — bounded logging / atomics only.
- `setTempo` rewrites tempo for EVERY peer. This tree sets it only on an explicit LTMP
  command; aloopprime refuses to propose when peers own it.
- **Interface readiness is a real race.** 500 ms settle before constructing Link,
  then IGMP re-asserted every 30 s (`IGMP_REASSERT_PERIOD_TICKS = 15` x 2 s) for the
  life of the association, never a one-shot join: one join at GOT_IP can race netif
  readiness and not stick, and a snooping AP ages out a join never repeated.

### The vendored Link is a checked-in FORK, not a submodule

`components/link-esp/link` is a patched copy of Ableton Link committed directly
(1506 tracked files, no nested `.git`). The stale `.gitmodules` entry was removed
because `git submodule update` would clobber the patch with no compile error and
silently kill ESP peer discovery. No root `.gitmodules`, so `Dockerfile`, `setup.sh`
and `build.yml` do NOT init submodules. Do not re-add. Two nested ones survive,
inert: upstream's own, and `components/link-esp/.gitmodules`.

The divergence is the relay hook: `link/include/ableton/platforms/asio/Socket.hpp`
declares `extern "C" wifi_link_multicast_forward(...)` and calls it on every send,
so `wifi_config.cpp` can unicast-copy Link discovery datagrams across the SoftAP
boundary. **Any Link bump must re-apply that hook** — losing it compiles fine and
fails only as silent non-discovery; it died once to a byte-order bug in its multicast
test (HOST-order `to_uint()` read as `dstip & 0xff`, so 224.76.78.75 read as 75). The
hook is now gated `if (!g_ap_active) return;` — in STA role it used to
unicast-duplicate to the gateway, which that fix would otherwise have switched on.

### SoftAP multicast gap

ESP32 SoftAP does not carry Link multicast between host and stations, hence
`link_multicast_relay_task`: re-emits each datagram to the group, to the AP's own IP,
and unicast to every associated station, preserving the source IP (Link needs it for
direct peer connect). Whether aloopprime's `brcmfmac` AP has the same gap is
UNVERIFIED (`ap_isolate=0` may suffice) — do not port the relay by assumption.

**The relay is a singleton and `g_ap_netif` is mutex-guarded** (`0bffe4c`):
`wifi_start_link_relay()` used to spawn a second task on every STA->AP re-host while
the old blocked in `recvfrom` forever, leaking sockets toward the 16-socket ceiling.
It now signals the running task (20 ms `SO_RCVTIMEO` so the stop flag is seen) and
waits for exit; `link_relay_release()` frees socket and raw pcb on every path.
`ap_netif_lock()` covers `g_ap_netif` writes and the relay's read-and-send section, `LOCK_TCPIP_CORE` inside it on both sides so lock order cannot
invert.

## Clock is hardware-scheduled, never tick-emitted

Click and 24 ppqn clock come from an `esp_timer` one-shot armed to
`SessionState::timeAtBeat()` of the NEXT pulse, never the 4 kHz gptimer tick
(`LINK_TICK_PERIOD 250` us), which discovers a beat only after it passed — so tick
emission costs wake-up latency plus WiFi/lwIP/log contention. Raising the tick rate
cannot tighten the click.

- Link's ESP clock IS `esp_timer_get_time()` — a Link microsecond timestamp is already
  an esp_timer deadline. No offset, no drift correction.
- **Skip-don't-catch-up:** the scheduler steps past pulses already due (capped
  `MAX_CATCHUP_PULSES_PER_SCHEDULE = 1024`); a burst reads as a tempo spike to any
  downstream PLL. `MIDI_CLOCK_RESYNC_THRESHOLD` is log-only.
- Same rule on the note path: `BassEngine::process` fires notes in `(last, cur]`;
  past `kMaxAdvanceStepsPerProcess` is a discontinuity, so it releases note-offs,
  re-anchors `m_lastPhrasePosSteps`, emits nothing (`5598ed6`). **A tempo change does
  not trip it** (`setTempo` rebuilds from `toBeats(atTime)`); only a phase jump does.

**`CONFIG_LWIP_MAX_SOCKETS=16` in `sdkconfig.defaults` is load-bearing**, not tuning:
the default 10 is exhausted by the app's own sockets (Link relay, LCLK/TTMP broadcast,
tempo listener, discovery forward, 4 for HTTP), leaving too few for Link's
per-interface discovery gateway -> EMFILE -> peers never discover. 16 is the maximum.

## Firmware reaches the ticker over USB only

No OTA, no serial console on this machine.

- CI builds on push to any branch. **The IDF image is digest-pinned** in
  `Dockerfile:1` (`ARG IDF_IMAGE=espressif/idf@sha256:1355cce31b...fea95`);
  `build.yml`'s `resolve-idf-image` job parses that line and exits 1 unless the digest
  is exactly 64 lowercase hex digits. A toolchain bump is that one line. On green
  `main` it commits the three images under `build/`; flashable images come from git.
- **Every build also uploads a flash bundle** `ticker-firmware-<sha>` (90-day
  retention, keeps the `build/` prefix so it drops over a local `build/` tree;
  `9c0014a`, `88ba35b`).
- Offsets: 0x1000 bootloader, 0x8000 partition table, app at the first app partition
  of the **BUILT** table (0x20000 with `partitions_large.csv`). **The app offset is
  DERIVED, never hardcoded** — `flash-ticker.js`, `flash.sh`, `tools/flash-usbip.py`
  read `build/partition_table/partition-table.bin`: 32-byte entries, LE magic `0x50AA`,
  then type(1) subtype(1) offset(4) size(4) label(16); app type == 0. **Never parse
  `partitions_large.csv`** — every offset but nvs's is blank (the build computes them),
  so parsing yields `int('')`. `--list` refuses to guess when >1 serial port exists;
  this machine has two Bluetooth COM ports that are not the ESP32.
- esptool 5.x: `write-flash` is hyphenated; **`--verify` was dropped** (it always
  verifies — "Hash of data verified"). Passing `--verify` aborts and flashes nothing.
- SPIFFS `storage` is flashed separately: `flash_midi_data.sh` builds a `0x19000`
  image from `./data` at **`0x317000`** (derivable: nvs 0x011000, phy_init 0x017000,
  app0 0x020000, app1 0x1A0000, storage 0x317000; app partitions round up to
  0x10000). Re-derive if the layout changes.

### Hands-free flash: `python tools/flash-usbip.py`

`flash-ticker.js` needs BOOT/IO0 held — a WCH/CH340 Windows driver limitation, not
the board. `flash-usbip.py` talks CH341 over USB/IP (usbipd-win) behind a
pyserial-shaped shim, so the WCH driver never loads. `--dry` = enter download mode,
prove sync, reboot; bare = flash and boot. Measured: exit 0, ~49 s, app in ~19.8 s at
460800 (~510 kbit/s), "Hash of data verified", boots `boot:0x13`. No BOOT hold, no
WSL. Four facts make it work:

1. **The D1 R32's auto-reset is DIFFERENTIAL.** EN low only at (DTR# high, RTS# low)
   `0x40`; IO0 only at (DTR# low, RTS# high) `0x20`; `0x60`/`0x00` conducts NEITHER —
   the opposite of esptool's convention, hence every esptool reset gave `boot:0x13`.
   Entry is the flip **0x40 -> 0x20** (`boot:0x3`); leave `0x40` then `0x00`.
2. **The CH341 withholds the first replies** — a sync frame goes unanswered for
   seconds, then surfaces behind a later FULL-SIZE OUT. esptool sends one frame on a
   0.1 s deadline, so prime with repeated framed syncs (`SYNC_TIMEOUT` 2 s); 2-8.
3. **The first attach back often fails** — usbipd keeps the device claimed after a
   disconnect, so the first `OP_REP_IMPORT` returns `status=4`; `open_port()` retries
   (10 x 6 s).
4. **`USBIPD_BUSID` in `tools/ch341.py` is THIS MACHINE's busid (`2-2`)**, not a board
   property. Re-read per host and after any replug; `usbipd bind --busid <busid>` once
   per boot or nothing attaches.

It passes **`--after no-reset-stub`, never `--after no-reset`**, and guards
`esptool.main()` with `except BaseException` (`09da1052`): `no-reset` calls
`soft_reset(True)` at the flash baud while the ROM speaks only 115200 -> intermittent
`FatalError: No more data to read` AFTER all three images verified, and `FatalError`
is not `SystemExit`, so `except SystemExit` lets EN go unpulsed.

**Hands-free serial read:** `UsbipPort()` from `tools/usbip-port.py` MUST have
`.init()` called after construction (else 4096 zero bytes per bulk IN floods the
output). Then `hs(HOLD_EN)` -> short drain -> `hs(RELEASE_BOTH)` resets the board;
`drain(seconds)` returns the boot log. Hyphen in the filename: import with
`importlib.util.spec_from_file_location`.

Baud: the ROM only speaks 115200, so `enter_and_sync` pins it (`ROM_BAUD`); `--baud`
applies only after the RAM stub re-times the port. USB/IP: 115200 -> 72.2 s vs 460800
-> 19.4 s; the one 460800 `FatalError` was a transient attach flake, not baud-specific.
921600 dies after "Changed." (`FatalError` from `flash_begin`, nothing written, chip
left in the stub); `--dry` recovers it, and it is UNMEASURED on `flash-ticker.js`'s
COM port (PRD `flash-ticker-js-921600-unmeasured`). esptool leaves the port at the
flash baud, so `boot_app()` resets to `ROM_BAUD` before pulsing EN. Benign: `flash_id()`
reads `0xFFFFFF` in download mode -> "Failed to communicate with the flash chip";
erase/program still go through the ROM.

## Mesh UDP protocol and MIDI emission

All payloads little-endian `int64`.

| Port | Direction | Payload | Purpose |
|---|---|---|---|
| 20810 | esp -> group ~50 Hz | `LCLK` + link-clock micros | phase for peers behind a unicast-RX wall |
| 20811 | looper -> esp | `LTMP` + microsPerBeat (+ beat0 micros + quantum microbeats) | looper sets tempo and phase |
| 20812 | esp -> group ~10 Hz | `TTMP` + microsPerBeat + beatOrigin microbeats + timeOrigin micros | esp -> looper timeline |
| 20812 | request/response | any datagram -> one-line JSON | status: peers/bpm/playing/beat/phase/quantum/ap/metro |

- **Every multicast send must iterate the netifs and set `IP_MULTICAST_IF` per
  interface** (`esp_netif_next_unsafe`): a raw socket with no `IP_MULTICAST_IF` exits
  the wrong interface on a dual-netif device — Link's own socket reached the peer while
  ours did not.
- **The looper sits behind a unicast-RX wall**: its bcm4343 delivers multicast but
  not unicast-to-self, so Link's ping/pong never completes there (peers=0). LCLK gives
  it phase; TTMP carries tempo back.
- Clock broadcast rate-limited to one packet per 20 ms (timeline 100 ms): a 4 kHz
  flood saturates the Pi's radio-RX drain and starves its control plane.
- Tempo bounded to Link's real range **20..999 bpm** (Ableton Test Plan TEMPO-4 names
  both ends). A 400 ceiling silently dropped legitimate tempos.
- `beatAtTime` is quantum-insensitive, `phaseAtTime` is not — the timeline broadcast
  uses `LINK_QUANTUM`.
- LTMP/phase requests are applied on the Link task, not the listener task:
  `captureAppSessionState`/`commitAppSessionState` need a single owner.
- Status is request/response, not broadcast: free when idle, cannot pollute the Link
  group. The dev PC is not on 192.168.4.0/24, so probe it from a mesh station. `MetroStats` (`fired`/`last`/`worst`/`mean`/`rms`) rides in it as
  `metro` — the only evidence of click lateness.

MIDI emission — one path, no per-device clock code. Byte values are named constants
in `main.h` (`0xF8` clock, `0xFA` start, `0xFC` stop, `0xFB` continue, `0xF2` SPP,
CC123 all-notes-off). `MIDI_PULSES_PER_QUARTER_NOTE = 24` and
`MIDI_SPP_UNITS_PER_BEAT = 4` live in `link_sync.cpp`, NOT `main.h`.

- Continuous 24 ppqn clock; SPP + Start/Continue ONLY at the 16-bar phrase boundary;
  Stop + CC123 on all 16 channels on transport stop and on peer loss. SPP counts MIDI
  beats (sixteenths): 1 Link beat = 4 units, 14-bit LSB first.
- Realtime bytes are single-byte and may interleave a running-status message, so the
  buzzer and clock paths never corrupt each other.
- Note-offs as velocity 0; CC123 on stop is what keeps RC-505 MK2 loops from sticking.
- Target behaviour: KO2 needs Start + is SPP-sensitive (hence one SPP per phrase);
  Volca Drum ignores SPP; MicroKorg/MiniNova need it non-bursting; Micron stays in
  phrase when Start is phrase-aligned; RC-505 MK2 repositions on SPP.
- Start is deferred to the next phrase boundary. `s_transport_running` records what
  gear was last TOLD (Start vs Continue) — not `isPlaying()`.
- Force-start waits 8 s, not 5 s, so two co-booting devices discover each other before
  either free-runs at an independent phase.
- NRPN sequence is CC99, CC98, CC6, CC38=0, CC101=127, CC100=127 — the null reset is
  mandatory or later CC6 is misread as NRPN data.
- UART TX ring buffer is 256 bytes, not 0: a 12-NRPN burst (108 bytes) would otherwise
  block the FreeRTOS main loop ~35 ms.

## Bass generator: 8 approaches, per-bar shaping

`main/state_machine.cpp` maps each of the 4 touch pads to two approaches — single tap
/ double tap (`DOUBLE_TAP_WINDOW_US = 300000`):

| Pad | Single | Double |
|---|---|---|
| 0 | `APP_ROLL` | `APP_ACID` |
| 1 | `APP_FUNK` | `APP_GLITCH` |
| 2 | `APP_STAB` | `APP_DRIFT` |
| 3 | `APP_PEDAL` | `APP_BREAK` |

`setApproach()` is the ONLY thing that activates the engine — nothing plays until a
pad is tapped. **Every bar is shaped, not repeated**: `regeneratePhrase()` used to
replay four 16-step motifs across all 64 bars — that is what made it sound like a ringtone — so each bar now goes through `shapeBar()` driven by the 24-field
per-approach `ApproachShape` (`bass_engine.h:80`). Phrase log carries `uniq=` distinct
bars /64, `sync=` mean syncopation permille, `reg=` register span. `DEBOUNCE_COUNT = 2`
settles a press in ~500 us; `DOUBLE_TAP_TIME_MS`/`HOLD_TIME_MS` in `main.h` are DEAD.

Anti-ringtone axes, per approach: Reich phase (`rotateMotif`, `bar/phaseShiftBars`);
asymmetric chord cells (`cellBars` 2-8 vs `kBarsPerChordCell` 4); **systematic**
microtiming (`bli::microOffsetForStep` × `timingSteps/kMicroTimingFullDepth` +
`voiceOffsetSteps` — never random per note (random reads sloppy); ghosts
(`insertGhosts`, post-thinning); drop bars (`bar % 16 == 15` or `restBarProb`);
`accentMask` (bit i = accent); `swingSteps` (odd 16ths delayed). **Swing and
syncopation are separate knobs**: `grooveSwing` feeds only `buildOnsets`
(`bassline_interpreter.cpp:121`), swing comes from `swingSteps`. Syncopation is
**boundary-weighted** (`kSyncBoundaryBars 4`, `Gain 1.6`, `BodyScale 0.70`): moderate
sync at boundaries beats uniform high sync. Microtiming caps are asserted:
max-abs ≤ 0.17 step (20 ms), SD ≤ 0.15 (18 ms), table SD 0.0955. Do NOT raise
`kMicroTimingFullDepth`.

Measured — g++ host sim, 2x64 bars/approach, bpm 128, seed 0x9E3779B9 (the only way
to compile `bass_engine.cpp` off-device):

| appr | n/bar | uniq/64 | adj/126 | sync |
|---|---|---|---|---|
| ROLL | 13.14 | 50,49 | 4 | .481 |
| ACID | 7.69 | 56,56 | 2 | .359 |
| FUNK | 7.52 | 60,59 | 0 | .439 |
| GLITCH | 3.41 | 49,55 | 5 | .482 |
| STAB | 3.16 | 39,40 | 12 | .453 |
| DRIFT | 1.66 | 13,14 | 32 | .014 |
| PEDAL | 1.78 | 13,13 | 67 | .026 |
| BREAK | 3.73 | 44,49 | 10 | .434 |

`adj/126` = adjacent identical bars; cross-approach bar-fingerprint sharing 0.8-1.3%
for the six rhythmic voices. DRIFT/PEDAL share heavily because a 1.7-note bar has only
a handful of (step,pitch) fingerprints — the metric saturates, not evidence of sameness.

**An approach is a VOICE, not just a shaping preset** (`0f1271f`). `dialsForApproach()`
biases the base dials; a switch clears `m_hasPrevPhraseMotif` (was blending 40% of the
OLD approach's motif in) and sets `m_regenPending`. Bias -> band is `Dials::bandIndex`,
so a small bias can jump a groove; bands index `kGrooves[9]`/`ContourShape[8]`/
`kScaleNames[9]`:

| approach | groove | contour | scale | minGate |
|---|---|---|---|---|
| ROLL | techno_roll (8) | wave (5) | minorPentatonic (3) | 0.22 |
| ACID | acid_303 (4) | terraced (7) | phrygian (2) | 0.20 |
| FUNK | funk_synco (7) | valley (4) | mixolydian (5) | 0.30 |
| GLITCH | garage_2step (5) | zigzag (6) | phrygianDom (6) | 0.20 |
| STAB | four_floor (3) | level (0) | dorian (0) | 0.20 |
| DRIFT | deep (1) | climb (1) | aeolian (1) | 0.70 |
| PEDAL | minimal (0) | level (0) | dorian (0) | 1.10 |
| BREAK | dembow (6) | arch (3) | melodicMinor (4) | 0.24 |

PEDAL/DRIFT `minGate` > 1 step is deliberate: sustained voices, not sequencer runs.
Not ported: tempo-responsive swing (`clamp(1.9 − (bpm−130)*0.005, 1.0, 2.0)`, scale
`swingSteps` by `target/1.7`) — needs `bpm` in `regeneratePhrase`; per-approach
`kTurnaroundTailSteps` {4,8,12} (16 today); register drift (no code path).

## HTTP clip server (8080) and SPIFFS

`main/network_midi.cpp` IS in the build. `network_midi_init()` starts
`esp_http_server` on **8080** from `app_main`: `POST /upload/*` -> `/spiffs/loops/<name>`,
`GET /info` -> `{"device_ip":...}`, `POST /clear` deletes every `*.mid`. Invisible at the
call sites: the server reserves **4 sockets** of the 16, and the SoftAP allows **8
stations** (`cfg.ap.max_connection = 8` = `MAX_AP_STA_IPS`) — also all the forwarder
can unicast to, so a 9th associates but never gets Link traffic. Power-save is off
(`WIFI_PS_NONE`): a dozing station misses multicast. Do not re-enable.

**SPIFFS IS mounted and both storage endpoints are LIVE** (`9568578`):
`mount_clip_storage()` runs first from `network_midi_init`; `format_if_mount_failed =
false` — never formats. Contract: **503** unmounted, **413** body over `total - used`,
**400** bad/empty name/body, **500** incomplete upload (partial unlinked). The URI
wildcard is one segment, re-checked at runtime (`is_single_clip_name`), so `/` and `..`
cannot escape `/spiffs/loops`. `/info` re-reads the IP per request — the netif is
recreated on every AP<->STA role change.

Open, not code: no auth on an open SSID; `network_midi_start()`/`_stop()` dead. Open in
code: **`filepath` truncation is unchecked in `clear_handler`** (checked only in
upload) — 512 bytes vs `CONFIG_HTTPD_MAX_URI_LEN=8192`; `/clear` filters
`d_type == DT_REG`, which SPIFFS VFS may leave `DT_UNKNOWN`, and `strstr(d_name,
".mid")` matches anywhere, so `notes.midi` dies too.

## MIDI file player (`main/midi_file.cpp`) — NOT in the build

Parser relies on **MIDI running status**; note-on with **velocity 0 IS a note-off**.
`process()` walks notes in `startBeat` order and early-breaks — the sort order is
load-bearing. **SPIFFS has no real directories**, so `setFolder()` matches a name
prefix. `syncToBpm` is `false`: enabling it double-scaled `playbackRate` and left notes
stuck.

## Wiring

GPIO numbers from `main/main.h` — a pin number cannot be expressed in code reading the
macro.

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

Touch pads use the **legacy `driver/touch_pad.h` API** (IDF also ships `touch_sensor`).
A pad reads LOW when touched; ARP is read first so its press latency stays lowest.
Pots are raw ADC 0-4095 -> MIDI 0-127, with a slow EMA tracking the stable center
beside the fast one. In the MIDI file player **pot1 = note length (0..2), pot2 =
velocity (0..1)**; the accessors are still `getPot1Value`/`getPot2Value`.

## Constants whose constraint is invisible in the code

- **E-Slew must stay derivable from sheer, anchored at 104.** `reset` sets sheer 0, so
  a derivation returning anything but 104 at sheer 0 makes the reset overwrite what
  `setSidechainPattern()` just sent. `gateESlewForSheer(sheer)` =
  `kGateESlewDefault + sheer*kHeadroomAboveDefault/kSheerMax` (104 -> 127 at sheer 0).
  104 is not magic; sheer 0 is.
- **Gate wet/dry must equal depth, not `127 - depth`.** `gateWetDryForDepth()` returns
  clamped depth and `SIDECHAIN_DEFAULT_DEPTH == SIDECHAIN_DEPTH_MAX == 127`, because
  MiniNova **CC 91 is wetness with 127 = full Gator** (User Manual v1.01: FX Amount
  "needs to be at maximum - 127"). The old `127 - depth` paired the deepest ducking
  with a bypassed gate. All outside SRCS — none runs on-device today.
- `kScales[3]` is `{0,3,5,7,10,3,5}` while `kScaleLens[3] == 5`: the trailing `{3,5}`
  is padding so the row matches its neighbours; only the first five are read. Do not
  fix it.
- `kRegisterSpan = 15` (semitones, `bassline_interpreter.h`) must track the
  `anchorMotif` clamp in `bass_engine.cpp` — recorded nowhere else, and neither file
  includes the other.
- `kProgs` rows are **scale-degree indices**, not semitones and not MIDI notes
  (corrected in `c0fc974`). Intents nowhere else: `{0,5,3,6}` i-VI-iv-VII,
  `{0,6,5,6}` i-VII-VI-VII, `{0,3,6,2}` i-iv-VII-III, `{0,5,6,4}` i-VI-VII-V ("dark
  cadence"), `{0,2,6,3}` i-III-VII-iv, `{0,0,5,6}` pedal.
- `main.h` asserts `LINK_QUANTUM == METRONOME_ACCENT_CYCLE_BEATS`: the click's accent
  cycle IS the quantum, so changing one alone breaks the build.

## Synth CC/NRPN facts not derivable from the code

- **MicroKorg**: LFO2 shape is CC 75 (`kCcLfo2Shape`); delay sync is CC 13
  (`kCcDelaySync`), 0 = off, 1..32 are sync values.
- **MiniNova**: `LFO2_SYNC_ENABLE` NRPN (MSB 1, LSB 40) is an **assumption never
  confirmed on hardware** — the one NRPN with no verified source. Filter bypass (NRPN
  0/60 value 0) is deliberately NOT sent by `deactivateFilter()`, which unpatches only
  the LFO.
- **MiniNova "Table 3"** is the source of `setDelaySyncRate` / `setLfoRateSync`;
  `main/lfo_constants.h` carries only a subset of that row (NRPN 0/86), not all.
- `SynthInterface` contracts live only in the implementations: `activateDelay()` /
  `activateReverb()` select Delay 1 / Reverb 1 in **FX Slot 1**; `setFxSlot1Level()` is
  **CC 91**; `activateFilter()` implicitly selects **LP24**; the LFO methods assume
  **LFO2 -> Filter1 Freq via Mod Matrix Slot 1**.
- **LFO depth is bipolar**: `-64..+63` maps to `0..127` with **64 = zero**, so
  `unpatchLfoFromFilter()` writes 64, not 0.
- Note-off velocity 0 is the **default**, not a call-site convention: `sendNoteOff(
  uint8_t note, uint8_t velocity = 0)` is declared identically across
  `synth_interface.h`, `synth_microkorg.h`, `synth_mininova.h`. The two surviving
  `= 64` defaults (cutoff, patch depth) are legitimately 64 and must stay.

## Build shape

**NOT** in the build: `effect_arp.cpp`, `effect_filter.cpp`, `effect_handler.cpp`,
`effect_sidechain.cpp`, `midi_file.cpp` — nothing `#include`s a `.cpp`, so that cluster
is reachable only from itself. Edits there cannot break the build. `bassline_interpreter` must stay **host-compilable**: only its
own header plus `<algorithm>` `<cmath>` `<cstring>` — no ESP-IDF headers — so
`g++ -std=c++17` exercises its DP and scale logic off-device, the only way to. On
MinGW also pass `-D_USE_MATH_DEFINES` or `M_PI` is undefined.

**`.gitattributes` pins EOL** (`7e8dacd`): `* text=auto eol=lf`, explicit `eol=lf` for
`*.sh *.bash *.py *.cmake CMakeLists.txt Makefile *.yml *.yaml *.csv sdkconfig
sdkconfig.defaults Dockerfile`, `eol=crlf` for `*.bat *.cmd`, `binary` for
`*.bin *.elf *.mid` and archives — a CRLF-checked-out shell script dies in the Linux
CI container.

## CI autobump: how it commits, and its hazard

`.github/workflows/build.yml` commits the three `.bin`s and pushes them: it **records
the sha it built**, then `git reset --hard FETCH_HEAD`, restores the three `.bin`s from
that sha and commits with a 3-path pathspec, up to 5 attempts. **`reset --soft` is NOT
usable**: it moves HEAD to the fetched tip but leaves the index on the tree this
checkout built, so the commit diffs `built_tree` minus `remote_tip` and silently
reverts every source commit in between. A rebase would MERGE the previous `.bin`s and
die on "Cannot merge binary files". Since `8454302` it also defines
`refuse_unless_bins_only()`, walking `git diff-tree --name-only -r` and returning 1 on
any path outside the three.

**Residual rule: any commit touching a non-`.bin` path must be re-verified against
HEAD after a pull** — a green build never proves a change survived. Whole commits
vanished: one autobump removed 264 lines of `AGENTS.md` and 60 of `network_midi.cpp`
while shipping two `.bin`s, and two later pairs are exact inverse mirrors. Because CI
rewrites the `.bin`s on `main` on nearly every push, `remote_moved` is normal, not an
error. Push with `git_push {recover_remote_moved: true}` (alias `pull_first`; opt-in,
clean-worktree gate, ff-only then merge, abort on conflict); `git_finalize`
auto-recovers and reports `auto_recovered:true`.

`../aloopprime` is the opposite: a **fetch-only checkout** (`origin.pushurl = no-push`,
no local git identity), so `git_finalize` there commits and gm refuses the push with
`push_disabled_by_config`. No identity either, so a merging `git_pull` dies with
`git_identity_required`, and local `main` sits months behind `origin/main` — never
merge to publish. Local-only commits are the intended end state; do not "fix" it by
editing its `.git/config` or adding a remote.
- aloop's shutdown SIGSEGV is fixed (`worker()` published stack locals into globals ->
  process-lifetime `unique_ptr`s; 17 stops, 0 `signal=11`).

## ESP-IDF 6.x breaks already absorbed

Historical — none visible in current source, each silently re-breaks on a bump:
`esp_netif_next()` -> `esp_netif_next_unsafe()` (6.1, link's `ScanIpIfAddrs.hpp`); legacy
`driver/adc.h` removed (6.1) -> `hall_sensor_read()` stub in `main/io_helpers.cpp`.

## Comment sweep (INVARIANT 3)

`main/` is **comment-free**: 42 files, 7911 lines, 0 matches for `^\s*(//|/\*)`. Any
new comment must become self-explanatory code; what cannot goes here, after checking
it is still relevant and factual. Re-count before quoting.

## gm usage gotchas

- `codesearch` is the canonical search verb; `{"help":true}` lists every field.
  Exhaustive modes (`literal`/`regex`/`filename`) REFUSE unknown fields; `dual`
  ignores them. `mode:"comments"` (or `{"comments":true}`) needs no query.
- `git_finalize` takes `message`/`paths`/`files`/`allow_whole_index`/
  `recover_remote_moved`/`source_ref`/`rev` in `body` or at the top level; top-level
  is folded in, `body` wins.
- `prd-add` derives an `id` from any text field when none is given; `bash`/`exec_js`
  take their script in a JSON `body` (`script`/`command`/`code`/`text`) or `raw_body`.
