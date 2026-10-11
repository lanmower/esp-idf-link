# esp-idf-link — agent notes

`CLAUDE.md` `@`-includes this file; durable facts live here, not in code. Do not extend it by
editing `CLAUDE.md`. Compacted 2026-10-11 from 33866 bytes; git log through `ac377d6` folded in.

## Mesh with `../aloopprime`: paired invariants (change BOTH or the mesh splits)

| Invariant | esp-idf-link | aloopprime |
|---|---|---|
| SSID | `main.cpp` `wifi_*("ticker")` | `hostapd.conf`, `wpa_supplicant.conf` |
| Auth | `wifi_connect_sta("ticker","")` | open (`key_mgmt=NONE`; `wpa=` commented) |
| AP addr / DHCP | `esp_netif_set_ip_info` `192.168.4.1/255.255.255.0` | `192.168.4.1/24`, dnsmasq `.2-.20` |
| Channel | SoftAP ch6 (`kTickerChannel = 6`, `wifi_config.h`) | `hostapd.conf` `channel=6` |
| Link multicast | `224.76.78.75:20808` | same (hardcoded in Link) |
| Quantum | `main.h` `LINK_QUANTUM 16.0` | `link_bridge.cpp` `quantum = 16.0` |
| Start/stop sync | `main.cpp` `enableStartStopSync(false)` | same, `false` |
| Host election | lowest MAC/BSSID wins | lowest MAC/BSSID wins |

`PHRASE_BEATS 64.0` (`main.h`) is NOT the quantum — SPP/transport-correction boundary (16 bars).
Pi spells `ticker` in `config/aloop.conf:62`, `autoap.sh:7`. Audited 2026-10-11: all eight rows
MATCH; deliberate asymmetries: IGMP one-sided (pi none, `ap_isolate=0`), scan stagger one-sided.

### Host election

Two cold-booting devices each scan before the other's AP exists, so "nothing found -> host" makes
BOTH host — Link cannot cross two isolated L2 domains. Hence hold `HOLD_MAX_MS = 6000`, monotonic
in own STA MAC (`mac_rank * HOLD_MAX_MS >> 24`), rescan 1/s, stagger `mac[5] * 15` ms;
`../aloopprime/src/net/autoap.sh` mirrors (0-6 s, lowest-wins). Invert both in one change.

Supervisor (`wifi_config.cpp`, 2 s cadence, forever):
- STA: dropped link reconnects up to 30 times (~60 s) before re-hosting.
- AP: re-scans; yields to a strictly-lower `ticker` BSSID and drops its AP — **only while it has
  ZERO stations** (`ap_has_associated_stations()`); a host that ever accepts a station is sticky.
- **Comparison MAC is the STA MAC** (`wifi_get_sta_mac()`, `e4:65:b8:77:0d:14`), not the SoftAP
  MAC (`...:15`); Pi `e4:5f:01:53:f0:f1` is lower, so this box yields.
- **Self-BSSID guard reads the interface** (`esp_wifi_get_mac(WIFI_IF_AP)`), never an inferred
  MAC — `esp_read_mac()` reads eFuse and may differ; without it the box flaps AP->STA->AP every
  tick (`5faf01a`).
- IGMP re-joined on yield, not only at boot (`d1b7f9a`). Lease proof: `WIFI: IP: 192.168.4.x`.
- **Hosting scan pinned to the mesh channel** (`2c2d39c`): the election scan needs
  `ensure_sta_started()` -> `WIFI_MODE_APSTA`, and an all-channel scan dwells off-channel,
  interrupting SoftAP beaconing, so a station associating inside a scan window never finishes
  DHCP. Supervisor passes `kTickerChannel`; boot/hold scans stay `kScanAllChannels` (no AP yet);
  cost: a host that left ch6 is never seen.

**Reading `peers=0`:** `gw(ok=1)` = gateway up; `dst=` on a high port (`192.168.4.1:54146`) means
the announcement arrived and Link is unicast-pinging — the failure is the ping/pong, NOT
multicast. `dst` is recorded before the role gate, so it shows even in STA role.

### By-the-book Link checklist (both trees)

- `captureAppSessionState()`/`commitAppSessionState()` off the audio thread; `AudioSessionState`
  variants only on it. This tree uses the App variants.
- **Start/stop sync OFF on purpose: mesh is clock-only** (`b94ac2d`); `isPlaying()` then reads
  **false** all session (transport is ESP-local). Re-enabling needs BOTH trees + a flash here.
- The three notification callbacks run on a Link thread, **Realtime-safe: no** — bounded logging
  / atomics only.
- `setTempo` rewrites tempo for EVERY peer. This tree sets it only on an explicit LTMP command;
  aloopprime refuses to propose when peers own it.
- **Interface readiness is a real race:** 500 ms settle before constructing Link, then IGMP
  re-asserted every 30 s (`IGMP_REASSERT_PERIOD_TICKS = 15` x 2 s) for the life of the
  association — never a one-shot join.
- **The Pi advertises one NodeState across every IFF_RUNNING interface**, so its `mep4` flips
  `192.168.4.1` <-> ICS `192.168.137.100` (the `54920` in the msr line);
  `Peers::sawPeerOnGateway` cached whichever arrived last. Off-link `mep4` is **counted and
  logged, never dropped** (`cfddf6d` reverted `4481a5a`).
- Measurement window 5x50 ms -> 12x250 ms (`4481a5a`): ~51 pongs for `kNumberDataPoints=100`.

### The vendored Link is a checked-in FORK, not a submodule

`components/link-esp/link` is a patched copy committed directly (1506 files, no nested `.git`);
`git submodule update` would clobber the patch with no compile error and silently kill ESP peer
discovery — hence the stale `.gitmodules` entry was removed. No root `.gitmodules`: `Dockerfile`,
`setup.sh`, `build.yml` do NOT init submodules; do not re-add.

Two API facts, each of which cost a CI failure (both compile on host, break on-device): vendored
standalone asio takes **flags AFTER the endpoint** (`send_to(buffer, to, 0, ec)`), opposite of
Boost's `(buffer, flags, to, ec)`; `PeerState::endpoint` is a **data member**, while
`MeasurementService`/`PingResponder`/`Socket::endpoint()` are methods — `peerState.endpoint()`
does not exist.

**Compiled asio is `components/espressif__asio` 1.32.0, not vendored
`link/modules/asio-standalone` 1.17.0**: `link-esp` REQUIRES `asio`, so the component's
`INCLUDE_DIRS` win as `-I` over the vendored tree's `-isystem` — the vendored copy is shadowed.
Corollary: `asio/error.hpp` maps only `no_buffer_space = ENOBUFS`, yet the send failure prints
`send_to: Not enough space` — the toolchain's `strerror_r` for **errno 12 / ENOMEM**, what our
instrumentation records. Do not call it ENOBUFS.

Tempo rides the wire as **int64 microseconds-per-beat, truncated not rounded**
(`link/include/ableton/link/Tempo.hpp:52`): a peer publishing 119.049 round-trips with ~1e-5 bpm
error; status JSON prints `st.tempo()` with `%.3f`, quantizing nothing.

**The relay hook is the divergence**: `link/include/ableton/platforms/asio/Socket.hpp` declares
`extern "C" wifi_link_multicast_forward(...)` and calls it on every send, so `wifi_config.cpp`
can unicast-copy Link discovery datagrams across the SoftAP boundary. **Any Link bump must
re-apply it** — losing it fails only as silent non-discovery. Gated `if (!g_ap_active) return;`.
`Socket::send` retries a failed `send_to` 3x, one-tick backoff, recording
ok/retry/attempt/giveup + errno instead of throwing (`cfddf6d`); `0a8760c` made it exponential
1..8 ticks reset on success — the flat `vTaskDelay(1)` let the storm feed itself.

### SoftAP multicast gap

ESP32 SoftAP does not carry Link multicast between host and stations, hence
`link_multicast_relay_task`: re-emits each datagram to the group, to the AP's own IP, and unicast
to every associated station, preserving the source IP (Link needs it for direct peer connect).
aloopprime's `brcmfmac` sharing the gap is UNVERIFIED (`ap_isolate=0` may suffice) — do not port
the relay by assumption. **The relay is a singleton, `g_ap_netif` mutex-guarded** (`0bffe4c`) —
it now signals the running task (20 ms `SO_RCVTIMEO`) and waits for exit, and
`link_relay_release()` frees socket + raw pcb on every path. `ap_netif_lock()` covers `g_ap_netif` writes and the relay's read-and-send, `LOCK_TCPIP_CORE`
inside on both sides so lock order cannot invert.

## Clock is hardware-scheduled, never tick-emitted

Click and 24 ppqn clock come from an `esp_timer` one-shot armed to `SessionState::timeAtBeat()`
of the NEXT pulse, never the 4 kHz gptimer tick (`LINK_TICK_PERIOD 250` us), which discovers a
beat only after it passed; raising the tick rate cannot tighten the click.

- Link's ESP clock IS `esp_timer_get_time()`: a Link timestamp is already an esp_timer deadline.
- **Skip-don't-catch-up:** the scheduler steps past pulses already due (capped
  `MAX_CATCHUP_PULSES_PER_SCHEDULE = 1024`); a burst reads as a tempo spike downstream.
  `MIDI_CLOCK_RESYNC_THRESHOLD` is log-only.
- Same rule on the note path: `BassEngine::process` fires notes in `(last, cur]`; past
  `kMaxAdvanceStepsPerProcess` it releases note-offs, re-anchors `m_lastPhrasePosSteps`, emits
  nothing (`5598ed6`). **A tempo change does not trip it**; only a phase jump does.
- `++` on a `volatile` counter is a **hard compile error** here (`23e5170`); `s_fire_count` is
  plain `uint32_t`.

**`sdkconfig.defaults` is load-bearing, not tuning.** `CONFIG_LWIP_MAX_SOCKETS=16` (16 is the
max): the default 10 is exhausted by the app's own sockets (Link relay, LCLK/TTMP broadcast,
tempo listener, discovery forward, 4 for HTTP), leaving too few for Link's per-interface
discovery gateway -> EMFILE -> peers never discover. Also `LWIP_TCPIP_RECVMBOX_SIZE=64`
(32->64) — ENOMEM from `sendto` means EITHER pool drained OR mailbox full;
`LWIP_UDP_RECVMBOX_SIZE=6`, `ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM=64`, `LWIP_STATS=y` (read
`mem.err`/`link.drop`/`link.memerr`) — all `cfddf6d`. **`CONFIG_LWIP_PBUF_POOL_SIZE` is NOT a
Kconfig symbol in IDF v6.1** and has been removed from `sdkconfig.defaults`; do not re-add it.
The rest is tuning.

## Firmware reaches the ticker over USB only

- CI builds on push to any branch. **The IDF image is digest-pinned** in `Dockerfile:1`
  (`ARG IDF_IMAGE=espressif/idf@sha256:1355cce31b...fea95`); `build.yml`'s `resolve-idf-image`
  job exits 1 unless the digest is exactly 64 lowercase hex digits — a toolchain bump is that one
  line. On green `main` it commits the three images under `build/`; flashable images come from
  git. Runners pinned `ubuntu-24.04`, not `ubuntu-latest` (`3a4413b`): GitHub moves that label to
  Ubuntu 26 on 2026-10-19. Every build also uploads a flash bundle `ticker-firmware-<sha>` (90-day
  retention; `9c0014a`, `88ba35b`).
- Offsets: 0x1000 bootloader, 0x8000 partition table, app at the first app partition of the
  **BUILT** table (0x20000 with `partitions_large.csv`). **The app offset is DERIVED, never
  hardcoded** (`31a6b99`): `flash-ticker.js`, `flash.sh`, `tools/flash-usbip.py` read
  `build/partition_table/partition-table.bin` — 32-byte entries, LE magic `0x50AA`, then
  type(1) subtype(1) offset(4) size(4) label(16), app type == 0. **Never parse
  `partitions_large.csv`** — every offset but nvs's is blank, so parsing yields `int('')`.
- esptool 5.x: `write-flash` is hyphenated; **`--verify` was dropped** (it always verifies).
  SPIFFS `storage` is flashed separately: `flash_midi_data.sh` builds a `0x19000` image from
  `./data` at **`0x317000`** (derivable: nvs 0x011000, phy_init 0x017000, app0 0x020000, app1
  0x1A0000, storage 0x317000; app partitions round up to 0x10000).

### Hands-free flash: `python tools/flash-usbip.py`

`flash-ticker.js` needs BOOT/IO0 held — a WCH/CH340 driver limit, not the board;
`flash-usbip.py` talks CH341 over USB/IP (usbipd-win) behind a pyserial shim, so the WCH driver
never loads (WSL2/CH340 setup is in `CLAUDE.md`). Measured: exit 0, ~49 s, app in ~19.8 s at
460800 (~510 kbit/s), "Hash of data verified", boots `boot:0x13`, no BOOT hold, no WSL.

1. **The D1 R32's auto-reset is DIFFERENTIAL.** EN low only at (DTR# high, RTS# low) `0x40`; IO0
   only at (DTR# low, RTS# high) `0x20`; `0x60`/`0x00` conduct NEITHER — opposite of esptool's
   convention, hence `boot:0x13` on every esptool reset. Entry is the flip **0x40 -> 0x20**
   (`boot:0x3`); leave `0x40` then `0x00`. CH341 handshake register is active-low: **bit6 =
   RTS#/EN, bit5 = DTR#/IO0**; `tools/usbip-boot.py`, `usbip341.py`, `usbip-tap.py` once had them
   inverted (BIT_RTS naming the IO0 line), fixed in `9ddd92f`.
2. **The CH341 withholds the first replies**: a sync frame goes unanswered for seconds, then
   surfaces behind a later FULL-SIZE OUT; esptool allows one frame on a 0.1 s deadline, so prime
   with repeated framed syncs (`SYNC_TIMEOUT` 2 s); 2-8.
3. **The first attach back often fails**: usbipd keeps the device claimed, so the first
   `OP_REP_IMPORT` returns `status=4`; `open_port()` retries (10 x 6 s).
4. **`USBIPD_BUSID` in `tools/ch341.py` is THIS MACHINE's busid (`2-2`)**, not a board property;
   re-read per host and after any replug, and `usbipd bind --busid <busid>` once per boot or
   nothing attaches.

Also: **`--after no-reset-stub`, never `--after no-reset`**, and guard `esptool.main()` with
`except BaseException` (`09da1052`) — `no-reset` soft-resets at the flash baud while the ROM
speaks 115200 -> `FatalError: No more data to read` AFTER all three images verified, and
`FatalError` is not `SystemExit`. **That guard is now a bounded retry loop**
(`FLASH_ATTEMPTS = 6`), not a single attempt: on failure it re-enters download mode at
`ROM_BAUD`, re-syncs and retries, because each retry visibly got further (one died only at
"Unable to verify flash chip connection" after "Staying in flasher stub"). Two more hazards it
absorbs: **`esp.flash_id()` reads `0xFFFFFF` in download mode** -> "Failed to communicate with
the flash chip"; that call is wrapped and non-fatal (erase/program still go through the ROM).
**`UsbipPort.timeout` has a 2.0 s floor** (`_min_timeout`, enforced in the setter): esptool
lowers `timeout` below that, and a CH341 reply that arrives late was then read as a dead stream
-> "Serial data stream stopped" / "Unable to verify flash chip connection". Do not let the port
read deadline drop under 2 s. The ROM only speaks 115200 (`ROM_BAUD`); `--baud` applies only
after the RAM stub re-times it: USB/IP 115200 -> 72.2 s vs 460800 -> 19.4 s; 921600 dies after
"Changed." — `--dry` recovers it, and it is UNMEASURED on
`flash-ticker.js` (PRD `flash-ticker-js-921600-unmeasured`). esptool leaves the port at the flash
baud, so `boot_app()` resets to `ROM_BAUD` before pulsing EN. Serial read: `UsbipPort()` from
`tools/usbip-port.py` MUST have `.init()` called after construction (else 4096 zero bytes per
bulk IN floods the output); `hs(HOLD_EN)` -> short drain -> `hs(RELEASE_BOTH)` resets the board,
`drain(seconds)` returns the boot log; import the hyphenated filename via
`importlib.util.spec_from_file_location`.

## Mesh UDP protocol and MIDI emission

Payloads little-endian `int64`; ports are `#define`s in `link_sync.cpp`: `LINK_PHASE_PORT 20810`,
`LINK_TEMPO_PORT 20811`, `LINK_TTMP_PORT 20812`, `LINK_STATUS_PORT 20812`.

| Port | Direction | Payload | Purpose |
|---|---|---|---|
| 20810 | esp -> group ~50 Hz | `LCLK` + link-clock micros | phase for peers behind a unicast-RX wall |
| 20811 | looper -> esp | `LTMP` + microsPerBeat (+ beat0 micros + quantum microbeats) | looper sets tempo and phase |
| 20812 | esp -> group ~10 Hz | `TTMP` + microsPerBeat + beatOrigin microbeats + timeOrigin micros | esp -> looper timeline |
| 20812 | request/response | any datagram -> one-line JSON | status: peers/bpm/playing/beat/phase/quantum/ap/metro |
| 34999 | esp -> gw / group | `TX_PROBE_PORT`, diagnostic | `log_tx_probe` every 10 s + `log_send_failure_probe`; nothing replies |

- **aloop binds only `224.76.78.75:20808`** (Pi AF_PACKET capture, 2026-10-11), so 20810 and 20812
  are dropped at its IP layer — `Udp: IgnoredMulti` ~57/s while `NoPorts` is frozen. **20808
  stays pure Link** (Link parses every datagram on it as `_asdp_v1`), so a peer wanting our phase
  must itself bind 20810/20812.
- **Live defect 2026-10-11 — UDP sends intermittently failed with ENOMEM (12)** (re-test in
  `## Live state`; ENOMEM not ENOBUFS despite the `Not enough space` string). `txprobe` (10 s
  cadence, 75 s): `e_unknown=12 e_group=12` at ~30 s and ~40 s, `0` at 9/19/49/59/69 s;
  `e_gw=0 e_sock=0 pb=1` at all 7 — socket/netconn exhaustion, PBUF_POOL and heap (~117 KB)
  **ruled out**; RX never stalls; `t_gw` 2.1-3.0 ms, not growing. **Both 12s were the probe's own
  bugs, not the app's**: `ip4_addr_t::addr` is network-order, so the first octet is the LOW byte
  and `(gw & 0xffffff00) | 99` built **99.168.4.1** (wire `0x63A80401`, matching a Pi capture)
  -> ARP on a public 99.x; and the probe socket had no `IP_MULTICAST_IF`. Both fixed; the same
  byte-order slip had `txprobe: gw=` printing the gateway reversed. **Still unreproduced.** **Why `peers=0` then: the measurement never finished** — `Link msr: start=15
  to=192.168.4.1:54920 rx=148 match=148 mism=0 parse=0 fin=0 fail=14`, `pongerr=send_to: Not
  enough space`, `pingtx=341 pingfail=51`, RTT 5 ms -> `last=40297us max=249414us`; RX healthy,
  the handshake died on the **send** side.
- **Every multicast send must iterate the netifs and set `IP_MULTICAST_IF` per interface**
  (`esp_netif_next_unsafe`): a raw socket without it exits the wrong interface on a dual-netif
  device — Link's own socket reached the peer while ours did not.
- **LCLK/TTMP are subscriber-driven** (`0a8760c`): without a hit on 20812 (status) or 20811
  (LTMP) within `SUBSCRIBER_HOLD_US` (15000000 = 15 s), clock drops 50 Hz -> 1 Hz, timeline 10 Hz
  -> 0.2 Hz. Clock is rate-limited to one packet per 20 ms (timeline 100 ms): a 4 kHz flood
  saturates the Pi's radio-RX drain. `uc` sends run ~15/s even with `peers=0` against ~14.6/s RX
  — volume tracks what we receive, not any registered peer.
- **The looper sits behind a unicast-RX wall**: its bcm4343 delivers multicast but not
  unicast-to-self, so Link's ping/pong never completes there; LCLK gives it phase, TTMP carries
  tempo back.
- Tempo bounded to **20..999 bpm** (`LINK_TEMPO_MAX_BPM 999.0`; Ableton Test Plan TEMPO-4,
  `396092d`). `beatAtTime` is quantum-insensitive, `phaseAtTime` is not — the timeline broadcast
  uses `LINK_QUANTUM`. LTMP/phase requests are applied on the Link task, not the listener task.
- Status is request/response, not broadcast: cannot pollute the Link group. The dev PC is not on
  192.168.4.0/24, so probe it from a mesh station. `MetroStats`
  (`fired`/`last`/`worst`/`mean`/`rms`) rides in it as `metro` — only evidence of click lateness.

MIDI emission — one path, no per-device clock code; bytes are named constants in `main.h` (`0xF8`
clock, `0xFA` start, `0xFC` stop, `0xFB` continue, `0xF2` SPP, CC123 all-notes-off).
`MIDI_PULSES_PER_QUARTER_NOTE = 24` and `MIDI_SPP_UNITS_PER_BEAT = 4` live in `link_sync.cpp`,
NOT `main.h`. NRPN sequence, note-off velocity 0, UART TX ring buffer, `syncToBpm` and the
per-target behaviour table are in `CLAUDE.md` / the top of `link_sync.cpp`. SPP + Start/Continue ONLY at the 16-bar boundary; Stop + CC123 on all 16 channels on
transport stop and peer loss (CC123 keeps RC-505 MK2 loops from sticking). SPP counts MIDI beats
(sixteenths): 1 Link beat = 4 units, 14-bit LSB first. Start is deferred to the next phrase
boundary; `s_transport_running` records what gear was last TOLD, not `isPlaying()`; force-start
waits 8 s, not 5 s (co-booting devices discover each other before either free-runs).

## Live state 2026-10-11 (firmware ac377d6)

- **The ENOMEM send failure did NOT reproduce**: a 240 s listen-only soak at ~14.5 min uptime saw
  8526 successful sends, `fail=0 giveup=0 errno=0`, heap flat (133956 -> 133096, min 116456); the
  `0a8760c` capture read `fail=1501 giveup=396 errno=12` over a comparable window. **Unreproduced,
  NOT fixed; cause unknown.** Needs a longer soak before any claim; the difference may be the
  Pi/AP side (mc-420 reported its own peers=1 recovery).
- peers=1 reached 2.1 s after Link init and held; bpm settled to 118.53 (the Pi's tempo); session
  ids converged on the peer's.
- **Session bounce**: `enable(false)`/`enable(true)` from `tickTask` drops peers to 0 and re-joins
  in **2540 ms**, adopting the peer's session id; beat stays monotonic (76.20 -> 81.242), no
  tempo break. Start/stop sync stayed off, so no Link Start/Stop is emitted; the Stop+CC123 seen
  is our own peer-loss handler. **The per-boot auto bounce was removed; only the on-demand `LRST`
  to `192.168.4.19:20811` remains** — `LINK_AUTO_BOUNCE_AT_US` was a diagnostic witness only.
- **`bad` (badmagic) is benign**: it is Link's own `_link_v1` measurement traffic on Link's
  socket, 1:1 with `Link msr: rx` (368 vs 368), not a third-party source.
- Open: ping/pong RTT ~120 ms with `pingfail=0`; heap drifted 164 KB -> 122 KB over 90 s.
- **Wi-Fi buffer stats**: `esp_wifi_statis_dump(uint32_t modules)` is public in `esp_wifi.h`
  (v6.1) and is the only Wi-Fi buffer-statistics observation API; there is no public enum for its
  module bitmask, so dump all modules with `0xffffffff`. AMPDU has **no runtime control** — only
  `wifi_init_config_t.ampdu_tx_enable` at init (`3d703ce`).

## Bass generator: 8 approaches, per-bar shaping

`main/state_machine.cpp` maps pad -> approach, single/double tap (`DOUBLE_TAP_WINDOW_US =
300000`): 0 `APP_ROLL`/`APP_ACID`, 1 `APP_FUNK`/`APP_GLITCH`, 2 `APP_STAB`/`APP_DRIFT`, 3
`APP_PEDAL`/`APP_BREAK`. `setApproach()` is the ONLY thing that activates the engine. **Every bar
is shaped, not repeated** — `shapeBar()` per bar from the 24-field `ApproachShape`
(`bass_engine.h:80`); replaying four 16-step motifs across 64 bars is the ringtone effect.
**An approach is a VOICE, not just a preset** (`0f1271f`): `dialsForApproach()` biases base dials
-> `Dials::bandIndex` -> `kGrooves[9]`/`ContourShape[8]`/`kScaleNames[9]`; a switch clears
`m_hasPrevPhraseMotif` and sets `m_regenPending`. PEDAL/DRIFT `minGate` 1.10/0.70 > 1 step =
sustained voices, not sequencer runs. Microtiming caps are asserted (max-abs <= 0.17 step, SD <=
0.15): do NOT raise `kMicroTimingFullDepth`; `DEBOUNCE_COUNT = 2` (~500 us) settles a press.

## HTTP clip server (8080) and SPIFFS

`main/network_midi.cpp` IS in the build: `network_midi_init()` starts `esp_http_server` on 8080 —
`POST /upload/*` -> `/spiffs/loops/<name>`, `GET /info` -> `{"device_ip":...}`, `POST /clear`
deletes `*.mid`. Invisible at the call sites: **4 sockets** of 16 reserved; SoftAP allows **8
stations** (`cfg.ap.max_connection = 8` = `MAX_AP_STA_IPS`) — also all the forwarder can unicast
to, so a 9th associates but gets no Link traffic; `WIFI_PS_NONE` (a dozing station misses
multicast) — do not re-enable. **SPIFFS IS mounted** (`9568578`): `mount_clip_storage()` runs
first; `format_if_mount_failed = false` — never formats. Contract: 503 unmounted, 413 body over
`total - used`, 400 bad/empty name/body, 500 incomplete (partial unlinked). URI wildcard is one
segment re-checked (`is_single_clip_name`), so `/` and `..` cannot escape. `/info` re-reads the IP
per request (netif recreated on every role change). Open: no auth on an open SSID;
`network_midi_start()`/`_stop()` dead; `filepath` truncation unchecked in `clear_handler` (512 vs
`CONFIG_HTTPD_MAX_URI_LEN=8192`); `d_type == DT_REG` (SPIFFS may leave `DT_UNKNOWN`), `strstr`.

## MIDI file player (`main/midi_file.cpp`) — NOT in the build

Running status; note-on vel 0 IS note-off; `process()` walks notes in `startBeat` order and
early-breaks (order load-bearing); `setFolder()` matches a prefix (SPIFFS has no directories).

## Wiring

GPIO numbers from `main/main.h` — a pin cannot be expressed by reading the macro.

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

Touch pads use the **legacy `driver/touch_pad.h` API** (IDF also ships `touch_sensor`); a pad
reads LOW when touched; ARP read first (lowest press latency). Pots: raw ADC 0-4095 -> MIDI 0-127;
in the player pot1 = note length (0..2), pot2 = velocity (0..1).

## Constants whose constraint is invisible in the code

- **E-Slew must stay derivable from sheer, anchored at 104**: `reset` sets sheer 0, so a
  derivation not returning 104 at sheer 0 makes the reset overwrite what `setSidechainPattern()`
  just sent. `gateESlewForSheer(sheer)` = `kGateESlewDefault +
  sheer*kHeadroomAboveDefault/kSheerMax` (104 -> 127 at sheer 0). 104 is not magic; sheer 0 is.
- **Gate wet/dry must equal depth, not `127 - depth`**: `gateWetDryForDepth()` returns clamped
  depth and `SIDECHAIN_DEFAULT_DEPTH == SIDECHAIN_DEPTH_MAX == 127`, because MiniNova **CC 91 is
  wetness, 127 = full Gator** (User Manual v1.01). All outside SRCS — none runs on-device today.
- `kScales[3]` is `{0,3,5,7,10,3,5}` while `kScaleLens[3] == 5`: trailing `{3,5}` is padding so
  the row matches its neighbours; only the first five are read. Do not fix it.
- `kRegisterSpan = 15` (semitones, `bassline_interpreter.h`) must track the `anchorMotif` clamp in
  `bass_engine.cpp` — recorded nowhere else; neither file includes the other.
- `kProgs` rows are **scale-degree indices**, not semitones, not MIDI notes (fixed `c0fc974`):
  `{0,5,3,6}` i-VI-iv-VII, `{0,6,5,6}` i-VII-VI-VII, `{0,3,6,2}` i-iv-VII-III, `{0,5,6,4}`
  i-VI-VII-V ("dark cadence"), `{0,2,6,3}` i-III-VII-iv, `{0,0,5,6}` pedal.
- `main.h` asserts `LINK_QUANTUM == METRONOME_ACCENT_CYCLE_BEATS`: the click's accent cycle IS the
  quantum — changing one alone breaks the build.

## Synth CC/NRPN facts not derivable from the code

- **MicroKorg**: LFO2 shape CC 75 (`kCcLfo2Shape`); delay sync CC 13 (`kCcDelaySync`), 0 = off,
  1..32 sync values.
- **MiniNova**: `LFO2_SYNC_ENABLE` NRPN (MSB 1, LSB 40) is an **assumption never confirmed on
  hardware** — the only NRPN with no verified source. Filter bypass (NRPN 0/60 value 0) is
  deliberately NOT sent by `deactivateFilter()`, which unpatches only the LFO.
- **MiniNova "Table 3"** is the source of `setDelaySyncRate`/`setLfoRateSync`;
  `main/lfo_constants.h` carries only NRPN 0/86 of that row. `main/nrpn_chart.txt` is the chart
  (was misspelled `nprn_chart.txt`, `3d42c32`).
- `SynthInterface` contracts live only in the implementations: `activateDelay()`/`activateReverb()`
  select Delay 1 / Reverb 1 in **FX Slot 1**; `setFxSlot1Level()` is **CC 91**;
  `activateFilter()` implicitly selects **LP24**; LFO methods assume **LFO2 -> Filter1 Freq via
  Mod Matrix Slot 1**.
- **LFO depth is bipolar**: `-64..+63` -> `0..127`, **64 = zero**, so `unpatchLfoFromFilter()`
  writes 64, not 0.
- Note-off velocity 0 is the **default**: `sendNoteOff(uint8_t note, uint8_t velocity = 0)`
  declared identically across `synth_interface.h`, `synth_microkorg.h`, `synth_mininova.h`. The
  two surviving `= 64` defaults (cutoff, patch depth) must stay.

## Build shape

**NOT** in the build: `effect_arp.cpp`, `effect_filter.cpp`, `effect_handler.cpp`,
`effect_sidechain.cpp`, `midi_file.cpp` — nothing `#include`s a `.cpp`, so that cluster is
reachable only from itself. `bassline_interpreter` must stay **host-compilable**: only its own
header plus `<algorithm>` `<cmath>` `<cstring>`, no ESP-IDF headers, so `g++ -std=c++17`
exercises its DP and scale logic off-device (MinGW: also `-D_USE_MATH_DEFINES`).
**`.gitattributes` pins EOL** (`7e8dacd`): `* text=auto eol=lf`, `eol=lf` for shell/py/cmake/
csv/sdkconfig/Dockerfile, `eol=crlf` for `*.bat *.cmd`, `binary` for `*.bin *.elf *.mid`.

## CI autobump: how it commits, and its hazard

`build.yml` commits the three `.bin`s: records the sha it built, `git reset --hard FETCH_HEAD`,
restores the `.bin`s, commits with a 3-path pathspec, up to 5 attempts (`111d902`: recommit on
the latest ref, never rebase). **`reset --soft` is NOT usable**: it moves HEAD to the fetched tip
but leaves the index on the tree this checkout built, so the commit silently reverts every source
commit in between. Since `8454302` `refuse_unless_bins_only()` walks `git diff-tree --name-only
-r` and returns 1 on any path outside the three; the binary commit is gated to `main`
(`d0afa95`).

**Residual rule: re-verify any commit touching a non-`.bin` path against HEAD after a pull** — a
green build never proves a change survived. CI rewrites the `.bin`s on `main` on nearly every
push, so `remote_moved` is normal. Push with `git_push {recover_remote_moved: true}` (alias
`pull_first`; opt-in, clean-worktree gate, ff-only then merge, abort on conflict);
`git_finalize` auto-recovers, reports `auto_recovered:true`.

`../aloopprime` is the opposite — **fetch-only checkout** (`origin.pushurl = no-push`, no local
git identity): `git_finalize` there commits and gm refuses the push with
`push_disabled_by_config`; a merging `git_pull` dies with `git_identity_required`. Local-only
commits are the intended end state; do not "fix" it.

## ESP-IDF 6.x breaks already absorbed

Historical — none visible in current source, each silently re-breaks on a bump:
`esp_netif_next()` -> `esp_netif_next_unsafe()` (6.1, link's `ScanIpIfAddrs.hpp`); legacy
`driver/adc.h` removed (6.1) -> `hall_sensor_read()` stub in `main/io_helpers.cpp`;
`CONFIG_LWIP_PBUF_POOL_SIZE` dropped (not a v6.1 symbol).

## Comment sweep (INVARIANT 3)

`main/` is **comment-free** (42 files, 8659 lines, 0 `^\s*(//|/\*)`; recount before quoting).

## gm usage gotchas

- `codesearch` is the canonical search verb; `{"help":true}` lists every field. Exhaustive modes
  (`literal`/`regex`/`filename`) REFUSE unknown fields; `dual` ignores them. `mode:"comments"`
  (or `{"comments":true}`) needs no query.
- `git_finalize` takes `message`/`paths`/`files`/`allow_whole_index`/`recover_remote_moved`/
  `source_ref`/`rev` — `body` or top level, `body` wins. `git_show` takes `rev`/`stat`/`path`
  (no `stat_only`); `git_log` takes `limit`/`range`/`path` (no `skip`) — page with `range`.
- `prd-add` derives an `id` from any text field when none is given; `bash`/`exec_js` take their
  script in `body` (`script`/`command`/`code`/`text`) or `raw_body`, and need `session_id`.
