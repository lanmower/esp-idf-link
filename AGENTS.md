# esp-idf-link — agent notes

`CLAUDE.md` includes this file. Durable cross-session facts belong here, not
inline in the code.

## esp-idf-link <-> aloopprime mesh: paired invariants (change BOTH or the mesh splits)

This project (the ESP32 "ticker" box) and `../aloopprime` (the Pi 4 hardware
looper) form ONE ad-hoc single-AP mesh so Ableton Link's multicast peer
discovery reaches every device. No credential provisioning: exactly one device
hosts the open SSID `ticker` and the rest join as stations. Every value below
exists in BOTH trees; changing it in one alone silently stops the two from
meshing, with no error on either side.

| Invariant | esp-idf-link | aloopprime |
|---|---|---|
| Mesh SSID | `main.cpp` `wifi_scan_best_bssid("ticker")` / `wifi_start_link_ap("ticker")` / `wifi_start_supervisor("ticker")` | `src/net/config/hostapd.conf` `ssid=ticker`, `wpa_supplicant.conf` `ssid="ticker"` |
| Auth | `wifi_connect_sta("ticker", "")` (open, empty password) | open (`key_mgmt=NONE`; `wpa=` lines commented out) |
| AP address / DHCP | `wifi_config.cpp` `esp_netif_set_ip_info` `192.168.4.1/255.255.255.0` | `192.168.4.1/24`, dnsmasq `.2-.20` |
| Channel | SoftAP ch6 | `hostapd.conf` `channel=6` |
| Link multicast | `224.76.78.75:20808` | same (hardcoded in Link itself — cannot drift) |
| Link quantum | `main.h` `#define LINK_QUANTUM 16.0` | `link_bridge.cpp` `quantum = 16.0` |
| Start/stop sync | `main.cpp` `g_link->enableStartStopSync(false)` | `link_bridge.cpp` `enableStartStopSync(false)` |
| Host election | lowest MAC/BSSID wins | lowest MAC/BSSID wins |

`PHRASE_BEATS 64.0` is NOT the Link quantum — it is this project's own
transport-correction/SPP boundary (16 bars). It intentionally differs from
`LINK_QUANTUM 16.0` and does not affect phase agreement with aloopprime.

### Host election must stay MAC-ordered on both sides

Two devices cold-booting together can each scan before the other's AP exists,
so a naive "nothing found -> host" makes BOTH host -- two isolated L2 domains
Link can never cross. This project holds for a duration strictly monotonic in
its own STA MAC (`HOLD_MAX_MS = 6000`, lowest MAC ~0 ms), rescanning every
second and joining the instant a peer's AP appears;
`../aloopprime/src/net/autoap.sh` mirrors it (0-6 s, lowest-wins). Both
supervisors also yield to a strictly-lower `ticker` BSSID -- never while clients
are attached, which would drop peers mid-session. Invert the convention here and
it MUST be inverted in aloopprime in the same change.

### By-the-book Ableton Link integration checklist (both projects)

Derived from a full audit of both trees against Link's own header docs.

- **Thread-correct session-state API.** `captureAppSessionState()` /
  `commitAppSessionState()` off the audio thread; the `AudioSessionState`
  variants only on it. This project uses the App variants throughout.
- **Start/stop sync is OFF on both sides on purpose: the mesh is clock-only.**
  A stop on one device must not stop the whole mesh — every box keeps its own
  timeline so timing survives a local stop elsewhere. This project calls
  `g_link->enableStartStopSync(false)` (`main.cpp:43`), still CONSUMES transport
  (`state.isPlaying()` in `link_sync.cpp` -> MIDI Start/Stop/Continue +
  all-notes-off) and never calls `setIsPlaying` — it has no local play/stop
  control. With sync disabled `isPlaying()` reads true for the whole session, so
  downstream gear gets one Start and never a Stop.
  `../aloopprime/src/link/link_bridge.cpp` calls `enableStartStopSync(false)`
  too (was `true` until 2026-10-10, aligned on this decision); going the other
  way needs BOTH flipped in the same change, plus a firmware flash here.
- **The three notification callbacks** — `setNumPeersCallback(std::size_t)`,
  `setTempoCallback(double)`, `setStartStopCallback(bool)`. Link's header
  documents each as invoked on a Link-managed thread and **Realtime-safe:
  no** — bounded logging / atomics only, never allocation or locks.
- **Tempo authority.** `setTempo` rewrites tempo for EVERY peer. This project
  only sets tempo on an explicit LTMP command (`s_tempoReq`), which is the
  correct restraint; `../aloopprime` additionally refuses to propose when peers
  already own the tempo. Do not make either side an unconditional writer.
- **Quantum is a shared constant.** `LINK_QUANTUM 16.0` here,
  `kLinkQuantum` in `../aloopprime/src/link/link_bridge.h`. They move together.
  `PHRASE_BEATS 64.0` is this project's own SPP/transport boundary and is
  deliberately different — do not "align" it to the quantum.
- **Interface readiness is a real race, and this project already models the
  fix.** The 500 ms settle before constructing Link (`main.cpp`) and the ~10 s
  IGMP re-assert (`wifi_config.cpp`) are what `../aloopprime` was corrected
  against -- it had no service ordering between its `aloopprime` and `autoap`
  units. The re-assert exists because a single IGMP join at GOT_IP can race
  netif readiness, so membership silently does not stick.

### The vendored Link is a FORK, checked in — not a submodule

`components/link-esp/link` is a **patched copy of Ableton Link committed
directly into this repo** (1506 tracked files, git mode `100755`, no nested
`.git`), NOT an active submodule -- the stale `.gitmodules` entry pointing at
`mathiasbredholt/link-esp` was removed because it invited a `git submodule
update` that would clobber the patch below with no compile error and silently
kill ESP peer discovery.

The divergence is the multicast relay hook:
`link/include/ableton/platforms/asio/Socket.hpp` declares `extern "C"
wifi_link_multicast_forward(...)` (:30) and calls it on every send (:67), so
`wifi_config.cpp`'s relay can unicast-copy each Link discovery datagram across
the SoftAP host/station boundary that does not carry multicast. Committed in
`88d6866` / `622f7ca`. **Any future Link version bump must re-apply that
hook** -- losing it compiles perfectly and fails only at runtime, as silent
non-discovery.

Git reads none but the root `.gitmodules`, and there is none there, so
`Dockerfile`, `setup.sh` and `.github/workflows/build.yml` deliberately do NOT
initialise submodules: no-ops today, they are exactly the mechanism that would
overwrite the fork the moment a root `.gitmodules` came back. Do not re-add
them. Two nested ones survive, inert: `components/link-esp/link/.gitmodules` is
upstream's own (kept, so the fork's divergence stays limited to the hook);
`components/link-esp/.gitmodules` was this repo's stale `[submodule "link"]`
entry and is deleted -- leaving it invites the clobbering `git submodule update`.

## Metronome and MIDI clock are hardware-scheduled, not tick-emitted

The click and the 24 ppqn `0xF8` come from a **hardware-scheduled `esp_timer`
one-shot armed to `SessionState::timeAtBeat()` of the NEXT pulse**
(`link_event_cb`), never from the 4 kHz gptimer tick task. A tick only
discovers a beat AFTER it passed, so tick emission costs every click the tick's
own wake-up latency plus WiFi/lwIP/logging/input-polling contention --
milliseconds of audible jitter. Only the alarm's dispatch latency (tens of
microseconds) is left, and it is the same for the click and the clock.

- The tick the scheduler replaced is `LINK_TICK_PERIOD 250` (250 us = 4 kHz).
  Raising its rate cannot tighten the click -- a tick only ever notices a beat
  AFTER it passed; only the esp_timer one-shot sets the edge.
- **Link's ESP clock IS `esp_timer_get_time()`**
  (`components/link-esp/link/include/ableton/platforms/esp32/Clock.hpp`), so a
  Link-clock microsecond timestamp is already an esp_timer deadline: no offset,
  no drift correction between the two.
- **Lateness is instrumented.** `MetroStats` (`fired`/`last`/`worst`/`mean`/
  `rms`) rides in the UDP status reply on 20812 as a `metro` object, so click
  stability is measurable from a peer
  (`../mc-420/test/hardware/link-mesh-status.js --esp <ip>`) with no scope on
  the buzzer.
- **The next click's PWM pitch is primed while the buzzer is silent**
  (`prime_buzzer_freq` from the buzzer-off callback), so the ON edge is two duty
  writes instead of a `ledc_timer_config()` reconfigure landing on the beat.
- **No-burst invariant, new mechanism.** There is no per-tick catch-up cap any
  more (`MIDI_MAX_CLOCKS_PER_TICK` is gone); the scheduler steps the pulse
  counter past every pulse whose due time is already behind, so a stall or a
  peer-imposed phase jump DROPS pulses instead of bursting them -- a burst of
  `0xF8` reads as a tempo spike to any downstream device PLL.
- `MIDI_CLOCK_RESYNC_THRESHOLD` is **log-only**: "we fell behind, resyncing",
  one warning, no burst. Its name predates that.
- A watchdog re-arms the alarm from the tick when a pulse is really overdue
  (threshold far longer than the fire/re-arm window) -- an alarm that never got
  re-armed would stop the clock silently.

Build trap: **`-Werror=volatile` is on**, so `++` on a `volatile` is a hard
error -- the fired-alarm counter is deliberately non-volatile.

Build trap: **`CONFIG_LWIP_MAX_SOCKETS=16` in `sdkconfig.defaults` is
load-bearing, not a tuning knob.** The default 10 is exhausted by the app's own
UDP sockets (Link relay, LCLK/TTMP broadcast, tempo listener, discovery
forward), leaving too few for Ableton Link's per-interface discovery gateway,
which then fails to open with EMFILE ("Too many open files") and never
broadcasts -> peers never discover each other. 16 is the ESP32 maximum.

## Firmware reaches the ticker over USB only

No OTA path and no serial console on this machine: a firmware change gets to
the device only over USB.

- CI (espressif/idf docker, ESP-IDF 6.2) builds on push to any branch, and on a
  green `main` it commits the three images -- `build/bootloader/bootloader.bin`,
  `build/partition_table/partition-table.bin`, `build/link-idf-example.bin`
  ("ci: update firmware binaries [skip ci]"). Flashable images come from git,
  not a local build.
- `node flash-ticker.js [COMx]` (repo root) does the whole thing over a COM port
  with esptool 5.x (`python -m esptool`; v5 hyphenates `write-flash`, though the
  underscored `write_flash` alias still works on 5.5.0, which
  `flash_midi_data.sh` relies on). Offsets: 0x1000 bootloader, 0x8000 partition
  table, app at the first app partition of the BUILT table (0x20000 with
  `partitions_large.csv`) -- the script reads
  `build/partition_table/partition-table.bin`, never hardcode 0x10000.
  **Read the offset from the built binary, not the CSV**: `partitions_large.csv`
  leaves every offset but nvs's empty because the build computes them, so
  parsing it yields `int('')`. Entries are 32 bytes, little-endian magic
  `0x50AA`, then type(1) subtype(1) offset(4) size(4) label(16); app entries
  have type == 0. `--list` lists ports, and it refuses to guess when more than
  one serial port exists -- this machine has two Bluetooth COM ports that are
  not the ESP32. esptool 5.x dropped `--verify` (it always verifies, "Hash of
  data verified" per image), so `--verify` is not ignored -- it aborts
  `write-flash` with "No such option" and flashes nothing.

- The SPIFFS `storage` partition (the MIDI files) is flashed separately:
  `flash_midi_data.sh` builds a `0x19000` (100K) image from `./data` and writes
  it at **`0x317000`**. That offset is NOT re-derivable from
  `partitions_large.csv` (the `storage` row leaves its offset blank; only `nvs`
  has one) and appears nowhere else in the repo but that script's own argument,
  so if the layout changes, re-read it from the built partition-table binary.

### No BOOT hold: `python tools/flash-usbip.py` flashes it hands-free

`flash-ticker.js` needs BOOT/IO0 held -- a WCH/CH340 Windows driver limitation,
not the board. `tools/flash-usbip.py` talks CH341 over USB/IP (usbipd-win,
`usbipd bind --busid 2-2` once) behind a pyserial-shaped shim, so the WCH
driver never loads and Windows cannot re-assert a line. `--dry` = enter
download mode, prove sync, reboot; bare = flash and boot. Three measured facts
make it work, all counter-intuitive:

1. **The D1 R32's auto-reset is DIFFERENTIAL.** Each transistor is driven by
   the DTR#-RTS# difference: EN low only at (DTR# high, RTS# low) -> `0x40`;
   IO0 only at (DTR# low, RTS# high) -> `0x20`; both asserted (`0x60`) or both
   clear (`0x00`) conducts NEITHER -- the opposite of esptool's hard-coded
   convention, hence every esptool reset gave `boot:0x13`. Entry is the flip
   **0x40 -> 0x20**; verified `boot:0x3`. Leave with `0x40` then `0x00`.
2. **The CH341 withholds the first replies.** One sync frame gets no answer for
   seconds, then past replies surface behind a later FULL-SIZE OUT; a 1-byte
   `0xC0` kick does not release them. After the first, latency is ~20 ms and
   stays. esptool sends one frame on a 0.1 s deadline, so prime with repeated
   framed sync frames (`SYNC_TIMEOUT` 2 s); typically 2-8.
3. **The first attach back often fails.** usbipd keeps the device claimed after
   a client disconnects, so the first `OP_REP_IMPORT` returns `status=4`;
   `open_port()` retries (10 x 6 s) and the second lands -- `attach 1/10:
   OP_REP_IMPORT status=4` is normal.

### Flash baud: 460800 is the default, 921600 does not work

The ROM only speaks 115200, so `enter_and_sync` pins it (`ROM_BAUD`); `--baud`
applies only after esptool uploads its RAM stub and re-times the port -- passing
`--baud` into the sync is what sank the first 921600 attempt. On the
1262560-byte app image: **115200 -> 72.2 s** (139.9 kbit/s) vs **460800 -> 19.4
s** (519.9 kbit/s, `Hash of data verified`, boots `boot:0x13`), repeatable
across two runs, USB/IP path only (`flash-ticker.js` drives a real COM port at
`--baud 921600`, unmeasured). 921600 dies just after `Changing baud rate to
921600... Changed.` with `FatalError: No more data to read from the serial
port` from `reset_chip -> soft_reset -> flash_begin`: no `Writing` line,
nothing written, chip left in the stub; `--dry` recovers it, firmware intact.
esptool leaves the port at the flash baud, so `boot_app()` resets to `ROM_BAUD`
before pulsing EN -- else the 115200 log is read at 460800 and no `boot:` line
appears at all.

Layers: `tools/ch341.py` (raw USB/IP; `SET_CONFIGURATION` first or bulk IN is
dead), `tools/usbip-port.py` (bulk OUT + pyserial surface),
`tools/flash-usbip.py` (entry, priming, esptool); `tools/usbip-boot-entry.py`
and `tools/usbip-sync-latency.py` are the instruments behind facts 1-2,
uncalled. Benign oddity: `flash_id()` reads `0xFFFFFF` (SPI RDID all ones) in
download mode, so esptool prints "Failed to communicate with the flash chip"
and auto-detects no size -- `detect_flash_size` returns None and
`_set_flash_parameters` skips `flash_set_parameters`, while erase/program go
through the ROM's flash_begin/flash_data.

## Mesh UDP protocol and MIDI emission

Ports, all little-endian `int64` payloads on both ends:

| Port | Direction | Payload | Purpose |
|---|---|---|---|
| 20810 | esp -> group, ~50 Hz | `LCLK` + link-clock micros | phase for peers behind a unicast-RX wall |
| 20811 | looper -> esp | `LTMP` + microsPerBeat (+ beat0 micros + quantum microbeats) | looper sets group tempo and phase |
| 20812 | esp -> group, ~10 Hz | `TTMP` + microsPerBeat + beatOrigin microbeats + timeOrigin micros | esp -> looper timeline |
| 20812 | request/response | any datagram -> one-line JSON | status: peers/bpm/playing/beat/phase/quantum/ap/metro |

- **Every multicast send must iterate the netifs and set `IP_MULTICAST_IF` per
  interface.** We may be AP or STA (boot race) and a raw socket with no
  `IP_MULTICAST_IF` exits the wrong interface on a dual-netif device -- Link's
  own socket reached the peer (it sets egress) while ours did not (Pi `clkRx`
  stayed 0).
- **The looper sits behind a unicast-RX wall**: its bcm4343 delivers
  multicast/broadcast but NOT unicast-to-self, so Link's ping/pong measurement
  never completes there (Pi `:4445` WLAN `uniRx`/`RALV` stay empty, peers=0).
  LCLK gives it phase; TTMP carries tempo back, since Link's native discovery
  never reaches it. Standard Link apps ignore all three ports.
- **The clock broadcast is rate-limited to one packet per 20 ms**: a 4 kHz flood
  saturates the Pi's single radio-RX drain and starves its control plane. 20 ms
  is far under one beat, so phase accuracy is unaffected.
- **Tempo is bounded to Link's real range, 20..999 bpm** (Ableton's Test Plan
  TEMPO-4 exercises the full range and names both ends). A 400 ceiling silently
  dropped legitimate session tempos instead of following them.
- `beatAtTime` is quantum-insensitive while `phaseAtTime` is not, so the
  timeline broadcast uses `LINK_QUANTUM` for the beat value.
- LTMP/phase requests are applied on the Link task, not the listener task:
  `captureAppSessionState`/`commitAppSessionState` need a single owner or the
  two tasks race.
- Status is deliberately request/response, not a broadcast: it costs nothing
  when idle and cannot pollute the Link multicast group.

MIDI emission -- one path, no per-device clock code:

- Continuous 24 ppqn `0xF8`; at the 16-bar phrase boundary only, SPP (`0xF2`) +
  Start/Continue (`0xFA`/`0xFB`); Stop (`0xFC`) + CC123 on all 16 channels on
  transport stop and on peer loss. SPP counts MIDI beats (sixteenths): one Link
  beat = 4 units, 14-bit, LSB first.
- Realtime bytes are single-byte and may legally interleave a running-status
  message, so the buzzer path and the clock path never corrupt each other.
- Note-offs go out as velocity 0; CC123 on stop is what keeps RC-505 MK2 loops
  from sticking.
- Target behaviour that constrains the design: KO2 locks to ext clock, needs
  Start, is SPP-sensitive (hence one SPP per phrase, not per 4 beats); Volca
  Drum follows the pulse and ignores SPP; MicroKorg/MiniNova arp, delay and LFO
  follow the pulse and need it non-bursting; Micron needs Start and stays in
  phrase when Start is phrase-aligned; RC-505 MK2 locks to clock+Start and
  repositions on SPP.
- Start is deferred to the next phrase boundary so gear begins in phrase
  rather than jerking mid-phrase. `s_transport_running` records what
  downstream gear was last TOLD (Start vs Continue) -- it is not
  `isPlaying()`.
- Force-start waits 8 s, not 5 s, so two co-booting devices can discover each
  other before either free-runs at an independent phase.

## HTTP clip server is live, and the mesh tops out at 8 stations

`main/network_midi.cpp` is **in the build** (see the SRCS note below) and
`network_midi_init()` starts an `esp_http_server` on **port 8080** from
`app_main`: `POST /upload/*` writes the body to `/spiffs/loops/<name>`,
`GET /info` returns `{"device_ip":...}`, `POST /clear` deletes every `*.mid`
there. Two bounds are invisible at the call sites: the server reserves **4
sockets** (`max_open_sockets = 4`) of the `CONFIG_LWIP_MAX_SOCKETS=16` budget,
and the SoftAP allows **8 stations** (`cfg.ap.max_connection = 8` =
`MAX_AP_STA_IPS`) -- 8 is also all the multicast forwarder can unicast to, so a
9th associates but never gets relayed Link traffic. WiFi power-save is off
(`WIFI_PS_NONE`): a dozing station misses multicast; do not re-enable it.

### 8080 audit (2026-10-10): fixed vs deliberately not

Fixed, each provable from the code: the URI wildcard entered
`/spiffs/loops/%s` unsanitised, so a name with `/` or `..` escaped it -- and
SPIFFS has no real directories, so `readdir` never lists that file and `/clear`
cannot remove it; the wildcard is now one segment. `filepath` is 512 bytes
against `CONFIG_HTTPD_MAX_URI_LEN=8192`, so `snprintf` could silently write a
different file -- now checked. `fopen(...,"wb")` ran before any body check, so
`Content-Length: 0` destroyed an existing clip; empty bodies are rejected
first. `fwrite`/`httpd_req_recv` were unchecked and the handler answered 200
regardless, leaving a truncated MIDI file -- an incomplete upload now unlinks
the partial file and answers 500. Unchecked `new` under
`CONFIG_COMPILER_CXX_EXCEPTIONS=y` threw `std::bad_alloc` out of the handler.
`/info` served `g_device_ip` from boot, but the netif is recreated on every
AP<->STA role change -- now re-read per request. `network_midi_init()` is
idempotent; a second call used to overwrite `g_server` and leak its sockets.

Checked, NOT defects: an embedded NUL (`%s` stops at NUL -- a shortened path,
never an overflow); the existing error returns (each already sends and returns
non-`ESP_OK`, closing the session). esp_http_server runs all sessions in one
task, so handlers cannot interleave.

PRD rows, not code: **nothing mounts SPIFFS** (0 hits for
`esp_vfs_spiffs_register`/`esp_vfs`/`mount_point` in `main/`), so
`fopen("/spiffs/...")` always fails and both storage endpoints are inert --
mounting needs a base path, a partition label and `format_if_mount_failed`,
which is destructive. Also: no auth on an open SSID; no size cap against the
100K partition; `network_midi_start()` a no-op vs `_stop()` (both dead).
`/clear` filters `d_type == DT_REG`, which SPIFFS VFS may leave `DT_UNKNOWN`;
`strstr(d_name, ".mid")` matches anywhere, so `notes.midi` dies too -- both
unverifiable without a device.

## MIDI file player (`main/midi_file.cpp`, files on SPIFFS)

- Parser relies on **MIDI running status**: a data byte with no status byte in
  front repeats the previous status. Note-on with **velocity 0 IS a note-off**;
  a note held at end-of-track is released implicitly.
- `process()` walks notes in `startBeat` order and early-breaks, so **the sort
  order is load-bearing**, not cosmetic.
- Trigger window is 0.03 beat; `playedNotes`/`sentCCs` dedup fires each once.
- Loop length rounds to the nearest quantum within 0.1, else up (ceil).
- **SPIFFS has no real directories**, so `setFolder()` cannot `opendir` a
  subfolder: it scans `/spiffs` and matches a name prefix.
- `updateTempo`'s 120 bpm reference and 0.5 bpm threshold are dormant
  (`syncToBpm` is false). See CLAUDE.md for why.
- Warts, left alone: `parseFile()` closes the file twice (`fclose` plus a
  `FileGuard`); the built-in default MIDI file's `MTrk` declares 19 bytes but
  writes 12, and its header encodes format 1. (Unbuilt, SPIFFS unmounted.)

## Input and buzzer wiring facts

Touch pad / pot / MIDI wiring (GPIO numbers, from `main/main.h` -- a pin number
cannot be expressed in code that reads the macro):

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

- Touch pads use the **legacy `driver/touch_pad.h` API** (6.2 also ships
  `touch_sensor`; this tree is on the legacy one). A pad reads LOW when touched;
  ARP is read first so its press latency stays lowest.
- Pots are raw ADC 0-4095 -> MIDI 0-127; a slow EMA tracks the stable center
  beside the fast one used for control. In the MIDI file player **pot1 = note
  length (0..2), pot2 = velocity (0..1)** (`effect_arp.cpp`
  `handle_arp_adjust_pots`); the accessors are still `getPot1Value`/
  `getPot2Value` despite the names.
- `DOUBLE_TAP_TIME_MS = 300` / `HOLD_TIME_MS = 200` (`main/main.cpp`) are
  **dead**: nothing reads either, so they are not the pad gesture timings.

## Constants whose constraint is not visible in the code

Constants with the right value but no visible rule; all were comments, none
expressible in code.

- `MAX_ARP_INDEX_WRAP = 128` (`main/arp_constants.h`) is referenced nowhere in
  the tree. If it is ever wired up it must stay **larger than the longest
  reasonably expected progression sequence** -- that bound, not 128, is the
  requirement.
- **E-Slew must stay derivable from sheer, anchored at 104.** `reset` sets sheer
  to 0 (`effect_handler.cpp:31`, `effect_sidechain.cpp:29`), so a derivation
  returning anything but 104 at sheer 0 makes the reset overwrite what
  `setSidechainPattern()` (`synth_mininova.cpp:153`) just sent.
  `gateESlewForSheer(sheer)` (`synth_mininova.h:6-11`) is
  `kGateESlewDefault + sheer*kHeadroomAboveDefault/kSheerMax` -- 104 at sheer 0
  rising to 127. The old `64 + sheer/2` gave 64. 104 is not magic; sheer 0 is.
- **Gate wet/dry must equal depth, not `127 - depth`.**
  `gateWetDryForDepth(depth)` (`main/sidechain_constants.h:50`) returns clamped
  depth, and `SIDECHAIN_DEFAULT_DEPTH == SIDECHAIN_DEPTH_MAX == 127` (`:12`),
  because MiniNova **CC 91 is wetness with 127 = full Gator** (User Manual
  v1.01: the Slot's FX Amount "needs to be at maximum - 127" for the Gator to
  have full effect). `setGateWetDry` therefore writes wet directly; the old
  `127 - depth` paired the deepest ducking with a bypassed gate.
  `setFxSlot1Level()` sends the same CC 91. Both rows live outside
  `main/CMakeLists.txt` SRCS, so none of it runs on-device today.
- `kScales[3]` (`main/bassline_interpreter.cpp`) is `{0,3,5,7,10,3,5}` while
  `kScaleLens[3] == 5`: the trailing `{3,5}` is deliberate padding so the row
  matches its neighbours; only the first five are read. Do not "fix" it.
- `kRegisterSpan = 15` (semitones, same file) must track the anchorMotif clamp
  in `bass_engine.cpp` -- recorded nowhere else, and neither file includes the
  other.
- `kProgs` (`main/bass_engine.cpp`) rows are **scale-degree indices**, not
  semitones and not MIDI notes. Intents exist nowhere else: `{0,5,3,6}` i-VI-iv-VII, `{0,6,5,6}` i-VII-VI-VII, `{0,3,6,2}` i-iv-VII-III,
  `{0,5,6,4}` i-VI-VII-V ("dark cadence"), `{0,2,6,3}` i-III-VII-iv,
  `{0,0,5,6}` the pedal row (i-i-VI-VII, a tonic drone).

## Synth CC/NRPN facts that are not derivable from the code

- **MicroKorg**: LFO2 shape is CC 75 (`kCcLfo2Shape`); delay sync is CC 13
  (`kCcDelaySync`), where 0 = off and 1..32 are sync values.
- **MiniNova**: `LFO2_SYNC_ENABLE` NRPN (MSB 1, LSB 40) is an **assumption
  never confirmed on hardware** -- the one NRPN in the tree with no verified
  source. Filter bypass (NRPN 0/60 value 0) is deliberately NOT sent by
  `deactivateFilter()`, which unpatches only the LFO.
- **MiniNova "Table 3"** is the source of `setDelaySyncRate` and
  `setLfoRateSync`; `main/lfo_constants.h` carries only a **subset** of the LFO
  rate-sync row (NRPN 0/86). Do not treat that list as exhaustive.
- `SynthInterface` contracts that exist only in the implementations:
  `activateDelay()`/`activateReverb()` select Delay 1 / Reverb 1 in **FX Slot
  1**; `setFxSlot1Level()` is **CC 91**; `activateFilter()` implicitly selects
  **LP24** and applies its defaults; the LFO methods assume **LFO2 -> Filter1
  Freq via Mod Matrix Slot 1**.
- **LFO depth is bipolar**: `-64..+63` maps to MIDI `0..127` with **64 = zero**,
  so `unpatchLfoFromFilter()` writes 64, not 0.
- Note-off velocity 0 is now the **default**, not a call-site convention:
  `sendNoteOff(uint8_t note, uint8_t velocity = 0)` is declared identically in
  `synth_interface.h`, `synth_microkorg.h`, `synth_mininova.h`. The two
  surviving `= 64` defaults in `synth_interface.h` (`activateFilter`'s cutoff,
  `patchLfoToFilter`'s depth) are legitimately 64 and must stay.

## `bassline_interpreter` must stay host-compilable

`main/bassline_interpreter.cpp` is kept free of ESP-IDF headers so it compiles
on the host with `g++ -std=c++17` -- the only way to exercise its DP and scale
logic off-device. Do not add ESP-IDF includes. On MinGW a host build also needs
`-D_USE_MATH_DEFINES`, or `M_PI` is undefined.

## Five `main/*.cpp` files are not in the build

`main/CMakeLists.txt` SRCS names 11 translation units, and `network_midi.cpp`
IS one of them -- compiled and linked, despite sitting below a blank line away
from the other ten (which is why earlier audits missed it). `effect_arp.cpp`,
`effect_filter.cpp`, `effect_handler.cpp`, `effect_sidechain.cpp` and
`midi_file.cpp` are NOT among them, and nothing `#include`s a `.cpp`, so that
cluster is reachable only from itself (`effect_handler.h` only by the four
effect `.cpp`s; `midi_file.h` only by `effect_arp.h` and `midi_file.cpp`). So
the MIDI-file-player behaviour in `CLAUDE.md` is not running on the device, and
edits there cannot break the build. Adding them to SRCS is a behaviour change.

## CI commits binaries with `reset --soft`, never a rebase

`.github/workflows/build.yml` commits the three `.bin` files and pushes them.
`idf.py build` leaves other unstaged files under `build/`, so a plain rebase
refuses ("unstaged changes"); a concurrent workflow advancing the branch is the
expected cause of a rejected push. The retry loop fetches, `git reset --soft
FETCH_HEAD`, re-stages only the three `.bin`s and re-commits once, up to 5
attempts. A rebase would MERGE the previous build's `.bin`s and die on "Cannot
merge binary files".

## ESP-IDF 6.x API breaks this tree has already absorbed

Historical -- none of it is visible in the current source, and each item
silently re-breaks on an IDF bump:

- `esp_netif_next()` -> `esp_netif_next_unsafe()` in 6.1
  (`components/link-esp/link/include/ableton/platforms/esp32/ScanIpIfAddrs.hpp`).
- Legacy `driver/adc.h` was removed in 6.1; `main/io_helpers.cpp` carries a
  `hall_sensor_read()` stub where that include used to be.
- `main/main.cpp` wraps the provisioning_mode block in braces because a `goto`
  crossed a variable initialization boundary. (The 512-byte filepath buffer in
  `network_midi.cpp` is not an IDF break -- see the HTTP clip server.)

## The SoftAP multicast gap is this project's, and may not be aloopprime's

The ESP32 SoftAP does not carry Link's multicast between host and stations,
which is why `link_multicast_relay_task` exists: it re-emits each Link datagram
to the group, to the AP's own IP, and unicast to every associated station,
preserving the original source IP (Link needs the true source for direct peer
connect). Whether aloopprime's Broadcom `brcmfmac` AP mode has the same gap is
UNVERIFIED -- `ap_isolate=0` may suffice there. Do not assume it needs this
relay ported; see `../aloopprime/docs/LINK-MESH-TESTING.md`.
