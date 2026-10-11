# esp-idf-link — agent notes

`CLAUDE.md` `@`-includes this file; durable facts live here, not in code. Don't extend it by editing
`CLAUDE.md`. Compacted 2026-10-11 (33866 -> 33193 -> this); git log through `f0fc252` folded in.

## Mesh with `../aloopprime`: paired invariants (change BOTH or the mesh splits)

| Invariant | esp-idf-link | aloopprime |
|---|---|---|
| SSID | `wifi_*("ticker")` in `main.cpp` | `hostapd.conf`, `wpa_supplicant.conf` |
| Auth | `wifi_connect_sta("ticker","")` | open (`key_mgmt=NONE`; `wpa=` commented) |
| AP / DHCP | `esp_netif_set_ip_info` `192.168.4.1/255.255.255.0` | `192.168.4.1/24`, dnsmasq `.2-.20` |
| Channel | SoftAP ch6 (`kTickerChannel = 6`) | `hostapd.conf` `channel=6` |
| Link multicast | `224.76.78.75:20808` | same (hardcoded in Link) |
| Quantum | `main.h` `LINK_QUANTUM 16.0` | `link_bridge.cpp` `quantum = 16.0` |
| Start/stop sync | `enableStartStopSync(false)` | same |
| Host election | lowest MAC/BSSID wins | same |

`PHRASE_BEATS 64.0` is NOT the quantum — SPP/transport-correction boundary (16 bars). Pi spells
`ticker` in `config/aloop.conf:62`, `autoap.sh:7`. Audited 2026-10-11: all eight MATCH; asymmetries
deliberate: IGMP one-sided (pi none, `ap_isolate=0`), scan stagger one-sided, takeover esp ~60 s vs
pi ~90-160 s.

### Host election

Two cold-booting devices each scan before the other's AP exists, so "nothing found -> host" makes
BOTH host — Link can't cross two isolated L2 domains. Hence hold `HOLD_MAX_MS = 6000`, monotonic in
own STA MAC (`mac_rank * HOLD_MAX_MS >> 24`), rescan 1/s, stagger `mac[5] * 15` ms;
`../aloopprime/src/net/autoap.sh` mirrors (0-6 s, lowest-wins). Invert both in one change.

Supervisor (`wifi_config.cpp`, 2 s cadence, forever):
- STA: dropped link reconnects up to 30 times (~60 s) before re-hosting.
- **STA is "up" only when it has an IP** (`579d4ef`): `g_wifi_connected` was cleared only by
  `WIFI_EVENT_STA_DISCONNECTED`, set only by `IP_EVENT_STA_GOT_IP`, with **no `IP_EVENT_STA_LOST_IP`
  handler** — an association outliving its IP left the flag true forever, so the supervisor sat on
  its connected branch doing nothing (no reconnect, IGMP re-assert, re-host, log line). Link drops a
  netif whose address is 0, `PeerGateways` erases the gateway, and with zero gateways Link neither
  sends nor receives — **peers 1->0 while still associated, no Wi-Fi event**. Now LOST_IP clears the
  flag and requests a reconnect; gate is `g_wifi_connected && sta_has_ip()`; both `volatile`.
- AP: re-scans; yields to a strictly-lower `ticker` BSSID — **only while ZERO stations**
  (`ap_has_associated_stations()`); a host that ever accepts a station is sticky.
- **Comparison MAC is the STA MAC** (`wifi_get_sta_mac()`, `e4:65:b8:77:0d:14`), not SoftAP `...:15`;
  Pi `e4:5f:01:53:f0:f1` is lower, so this box yields to it.
- **Self-BSSID guard reads the interface** (`esp_wifi_get_mac(WIFI_IF_AP)`), never an inferred MAC —
  `esp_read_mac()` reads eFuse and may differ; without it the box flaps AP->STA->AP every tick.
- IGMP re-joined on yield, not only at boot; since `579d4ef` also in **AP role** (was STA-only,
  one-shot at AP start). Lease proof: `WIFI: IP: 192.168.4.x`.
- **Hosting scan pinned to the mesh channel**: the election scan needs `ensure_sta_started()` ->
  `WIFI_MODE_APSTA`, and an all-channel scan dwells off-channel, interrupting SoftAP beaconing, so a
  station associating inside a scan window never finishes DHCP. Supervisor passes `kTickerChannel`;
  boot/hold scans stay `kScanAllChannels`. Cost: a host that left ch6 is never seen.
- **The AP->STA yield paths run only after a device has hosted once**, so a soak that never leaves
  STA role exercises none of them.

**Reading `peers=0`:** `gw(ok=1)` = gateway up; `dst=` on a high port (`192.168.4.1:54146`) means
the announcement arrived and Link is unicast-pinging — failure is ping/pong, NOT multicast (`dst` is
recorded before the role gate, so it shows in STA role too). **`dst` is HOST order** —
`wifi_link_send_dst` gets asio's `to_v4().to_uint()` (`Socket.hpp:63`) while `ip4_addr_t::addr` is
network order, so `htonl` before `IP2STR`; pre-`579d4ef` images printed it reversed (`75.78.76.224`
was 224.76.78.75) — same trap as `99.168.4.1`.

**How Link drops a peer** (vendored tree, read-only — do NOT patch): routes are **TTL ~6 s**
(`mTtl = 5` + 1 s, `PeerGateway.hpp:120-158`); ByeBye; **`gatewayClosed`** — `~GatewayObserver`
(`Peers.hpp:136-144`) -> `Peers.hpp:302-313` erases ALL peers on that gateway address, the
1->0-with-no-Wi-Fi-event route, also fired by `enable(false)`'s `clear()` (`PeerGateways.hpp:74`);
`resetPeers()` `Peers.hpp:112`; `forgetSession()` `Peers.hpp:104`. `UdpMessenger.hpp:74-77` swallows
every send error (upstream's `throw UdpSendException` at `:73` is commented out, so the doc at `:52`
lies) — hence **`repairGateway` (`PeerGateways.hpp:98-105`) is dead on ESP**: sole caller
`Controller.hpp:753`, and `Config.hpp:42` picks `platforms/esp32/Context.hpp`, whose ctor
(`:143-146`) takes the exception handler and discards it, so `asio/Context.hpp:68-71`'s catch loop
isn't compiled. The equivalent recovery runs anyway: gateway erase + 5 s rescan via `Callback::
operator()` (`Service.hpp:35`, `InterfaceScanner.hpp:74`).

### By-the-book Link checklist (both trees)

- `captureAppSessionState()`/`commitAppSessionState()` off the audio thread; `AudioSessionState`
  variants only on it. This tree uses the App variants.
- **Start/stop sync OFF on purpose: mesh is clock-only** (`b94ac2d`); `isPlaying()` then reads
  **false** all session (transport is ESP-local). Re-enabling needs BOTH trees + a flash here.
- The three notification callbacks run on a Link thread, **Realtime-safe: no** — bounded logging /
  atomics only.
- `setTempo` rewrites tempo for EVERY peer; this tree sets it only on an explicit LTMP command.
- **Interface readiness is a real race:** 500 ms settle before constructing Link, then IGMP
  re-asserted every 30 s for the life of the association — never a one-shot join.
- The Pi advertises one NodeState across every IFF_RUNNING interface, so its `mep4` flips
  `192.168.4.1` <-> ICS `192.168.137.100` (the `54920` in the msr line); `Peers::sawPeerOnGateway`
  cached whichever arrived last. Off-link `mep4` is **counted and logged, never dropped** (`cfddf6d`
  reverted `4481a5a`). Measurement window 5x50 ms -> 12x250 ms (`4481a5a`).

### The vendored Link is a checked-in FORK, not a submodule

`components/link-esp/link` is a patched copy committed directly (1506 files, no nested `.git`);
`git submodule update` would clobber the patch with no compile error and silently kill ESP peer
discovery — hence the stale `.gitmodules` entry was removed. No root `.gitmodules`: `Dockerfile`,
`setup.sh`, `build.yml` do NOT init submodules; don't re-add. Our patches live in `Socket.hpp`,
`PeerGateways.hpp`, `UdpMessenger.hpp`, `Controller.hpp`, `Peers.hpp` — re-apply on any bump.

Two API facts, each costing a CI failure (both compile on host, break on-device): vendored standalone
asio takes **flags AFTER the endpoint** (`send_to(buffer, to, 0, ec)`), opposite of Boost's
`(buffer, flags, to, ec)`; `PeerState::endpoint` is a **data member**, while
`MeasurementService`/`PingResponder`/`Socket::endpoint()` are methods — `peerState.endpoint()`
doesn't exist. **Compiled asio is `components/espressif__asio` 1.32.0, not vendored
`link/modules/asio-standalone` 1.17.0** (`link-esp` REQUIRES `asio`, so the component's
`INCLUDE_DIRS` win as `-I` over the vendored tree's `-isystem`): `asio/error.hpp` maps only
`no_buffer_space = ENOBUFS`, yet the send failure prints `send_to: Not enough space` — the
toolchain's `strerror_r` for **errno 12 / ENOMEM**. Tempo rides the wire as **int64
microseconds-per-beat, truncated not rounded** (`Tempo.hpp:52`): a peer publishing 119.049
round-trips with ~1e-5 bpm error; status JSON prints `st.tempo()` `%.3f`, quantizing nothing.

**The relay hook is the divergence**: `platforms/asio/Socket.hpp` declares `extern "C"
wifi_link_multicast_forward(...)` and calls it on every send, so `wifi_config.cpp` can unicast-copy
Link discovery datagrams across the SoftAP boundary. **Any Link bump must re-apply it** — losing it
fails only as silent non-discovery. Gated `if (!g_ap_active) return;`. `Socket::send` retries a
failed `send_to` 3x, one-tick backoff, recording ok/retry/attempt/giveup + errno instead of throwing
(`cfddf6d`); `0a8760c` made it exponential 1..8 ticks reset on success.

### SoftAP multicast gap

ESP32 SoftAP doesn't carry Link multicast between host and stations, hence
`link_multicast_relay_task`: re-emits each datagram to the group, to the AP's own IP, and unicast to
every associated station, preserving the source IP (Link needs it for direct peer connect).
aloopprime's `brcmfmac` sharing the gap is UNVERIFIED (`ap_isolate=0` may suffice) — don't port the
relay by assumption. **The relay is a singleton, `g_ap_netif` mutex-guarded**: it signals the
running task (20 ms `SO_RCVTIMEO`) and waits for exit; `link_relay_release()` frees socket + raw pcb
on every path. `ap_netif_lock()` covers `g_ap_netif` writes and the relay's read-and-send,
`LOCK_TCPIP_CORE` inside on both sides so lock order can't invert.

**The AP->STA yield is where role-switch leaks lived** (`dc9add9`): the relay was never stopped
there; `g_relay_task` was non-volatile under a spin; the forward socket was a function-local
`static`, so it stayed held for the process lifetime — one of only 16 — now closed on yield under a
mutex the send hook also takes, so `close()` can't land mid-`sendto`. `wifi_stop_link_relay()` bounds
its wait (100 x 20 ms); `wifi_start_link_relay()` refuses a second task. `wifi_start_link_ap()` takes
one locked local of `g_ap_netif` and clears `g_ap_sta_ips[]`. `wifi_join_link_multicast()` returns
unless the STA already has an IP — the supervisor re-asserts within 2 s of `sta_up`.

## Clock is hardware-scheduled, never tick-emitted

Click and 24 ppqn clock come from an `esp_timer` one-shot armed to `SessionState::timeAtBeat()` of
the NEXT pulse, never the 4 kHz gptimer tick (`LINK_TICK_PERIOD 250` us), which discovers a beat
only after it passed; raising the tick rate can't tighten the click.

- Link's ESP clock IS `esp_timer_get_time()`: a Link timestamp is already an esp_timer deadline.
- **Skip-don't-catch-up:** the scheduler steps past pulses already due (capped
  `MAX_CATCHUP_PULSES_PER_SCHEDULE = 1024`); a burst reads as a tempo spike downstream.
- Same rule on the note path: `BassEngine::process` fires notes in `(last, cur]`; past
  `kMaxAdvanceStepsPerProcess` it releases note-offs, re-anchors `m_lastPhrasePosSteps`, emits
  nothing (`5598ed6`). **A tempo change doesn't trip it**; only a phase jump does.
- **`volatile` bites twice**: `++` on a volatile counter is a hard compile error (`23e5170`, so
  `s_fire_count` is plain `uint32_t`), and `xTaskCreate`'s last parameter is non-volatile
  `TaskHandle_t*` — passing `&g_relay_task` after it was made volatile is `invalid conversion from
  'tskTaskControlBlock* volatile*'` under `-Werror` (`f0fc252`): create into a local, then assign.

**`sdkconfig.defaults` is load-bearing, not tuning.** `CONFIG_LWIP_MAX_SOCKETS=16` (16 is the max):
the default 10 is exhausted by the app's own sockets (Link relay, LCLK/TTMP broadcast, tempo
listener, discovery forward, 4 for HTTP), leaving too few for Link's per-interface discovery gateway
-> EMFILE -> peers never discover. Also `LWIP_TCPIP_RECVMBOX_SIZE=64` (32->64) — ENOMEM from
`sendto` means EITHER pool drained OR mailbox full; `LWIP_UDP_RECVMBOX_SIZE=6`,
`ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM=64`, `LWIP_STATS=y` (read `mem.err`/`link.drop`/`link.memerr`)
— all `cfddf6d`. **`CONFIG_LWIP_PBUF_POOL_SIZE` is NOT a v6.1 symbol**; don't re-add.

## Firmware reaches the ticker over USB only

- CI builds on push to any branch. **The IDF image is digest-pinned** in `Dockerfile:1`
  (`ARG IDF_IMAGE=espressif/idf@sha256:1355cce31b...fea95`); `build.yml`'s `resolve-idf-image` job
  exits 1 unless the digest is exactly 64 lowercase hex digits — a toolchain bump is that one line.
  On green `main` it commits the three images under `build/`; flashable images come from git.
  Runners pinned `ubuntu-24.04`, not `ubuntu-latest` (GitHub moves that label 2026-10-19).
- Offsets: 0x1000 bootloader, 0x8000 partition table, app at the first app partition of the
  **BUILT** table (0x20000 with `partitions_large.csv`). **The app offset is DERIVED, never
  hardcoded**: `flash-ticker.js`, `flash.sh`, `tools/flash-usbip.py` read
  `build/partition_table/partition-table.bin` — 32-byte entries, LE magic `0x50AA`, then type(1)
  subtype(1) offset(4) size(4) label(16), app type == 0. **Never parse `partitions_large.csv`** —
  every offset but nvs's is blank, so parsing yields `int('')`. `--list` refuses to guess when >1
  serial port exists (this machine has two Bluetooth COM ports that aren't the ESP32).
- esptool 5.x: `write-flash` is hyphenated; **`--verify` was dropped** (it always verifies). SPIFFS
  `storage` flashes separately: `flash_midi_data.sh` builds a `0x19000` image from `./data` at
  **`0x317000`** (nvs 0x011000, phy_init 0x017000, app0 0x020000, app1 0x1A0000, storage 0x317000).

### Hands-free flash: `python tools/flash-usbip.py`

`flash-ticker.js` needs BOOT/IO0 held — a WCH/CH340 driver limit, not the board; `flash-usbip.py`
talks CH341 over USB/IP (usbipd-win) behind a pyserial shim, so the WCH driver never loads (WSL2
/CH340 setup is in `CLAUDE.md`). Measured: exit 0, ~49 s, app in ~19.8 s at 460800 (~510 kbit/s),
"Hash of data verified", boots `boot:0x13`, no BOOT hold, no WSL.

1. **The D1 R32's auto-reset is DIFFERENTIAL.** EN low only at (DTR# high, RTS# low) `0x40`; IO0
   only at (DTR# low, RTS# high) `0x20`; `0x60`/`0x00` conduct NEITHER — opposite of esptool's
   convention, hence `boot:0x13` on every esptool reset. Entry is the flip **0x40 -> 0x20**
   (`boot:0x3`); leave `0x40` then `0x00`. The CH341 handshake register is active-low: **bit6 =
   RTS#/EN, bit5 = DTR#/IO0**; `tools/usbip-boot.py`, `usbip341.py`, `usbip-tap.py` once had them
   inverted (BIT_RTS naming the IO0 line), fixed in `9ddd92f`.
2. **The CH341 withholds the first replies**: a sync frame goes unanswered for seconds, then
   surfaces behind a later FULL-SIZE OUT; esptool allows one frame on a 0.1 s deadline, so prime
   with repeated framed syncs (`SYNC_TIMEOUT` 2 s); 2-8.
3. **The first attach back often fails**: usbipd keeps the device claimed, so the first
   `OP_REP_IMPORT` returns `status=4`; `open_port()` retries (10 x 6 s).
4. **`USBIPD_BUSID` in `tools/ch341.py` is THIS MACHINE's busid (`2-2`)**, not a board property;
   re-read per host and after any replug, and `usbipd bind --busid <busid>` once per boot.

Also: **`--after no-reset-stub`, never `--after no-reset`**, and guard `esptool.main()` with
`except BaseException` (`FatalError` isn't `SystemExit`); that guard is a bounded retry loop
(`FLASH_ATTEMPTS = 6`) — attempts 1-2 died with `Serial data stream stopped` right after
"Connected", attempt 3 wrote and verified. **`esp.read_mac()`/`flash_id()` are wrapped non-fatal**:
a `FatalError` from either aborted **after download-mode entry**, stranding the board at `boot:0x3`.
**Nothing between sync and `boot_app()` may raise.** `UsbipPort.timeout` has a **2.0 s floor**.
The ROM only speaks 115200 (`ROM_BAUD`); `--baud` applies only after the RAM stub re-times it:
115200 -> 72.2 s vs 460800 -> 19.4 s; 921600 dies after "Changed." (`--dry` recovers; UNMEASURED on
`flash-ticker.js`).
`boot_app()` resets to `ROM_BAUD` before pulsing EN. Serial read: `UsbipPort()` from
`tools/usbip-port.py` MUST have `.init()` called (else 4096 zero bytes per bulk IN floods the
output); `hs(HOLD_EN)` -> drain -> `hs(RELEASE_BOTH)` resets; import the hyphenated name via
`importlib.util.spec_from_file_location`.

## Mesh UDP protocol and MIDI emission

Payloads little-endian `int64`; ports are `#define`s in `link_sync.cpp`: `LINK_PHASE_PORT 20810`,
`LINK_TEMPO_PORT 20811`, `LINK_TTMP_PORT`/`LINK_STATUS_PORT` 20812, `TX_PROBE_PORT 34999`.

| Port | Direction | Payload | Purpose |
|---|---|---|---|
| 20810 | esp -> group ~50 Hz | `LCLK` + link-clock micros | phase for peers behind a unicast-RX wall |
| 20811 | looper -> esp | `LTMP` + microsPerBeat (+ beat0 + quantum) | looper sets tempo and phase |
| 20812 | esp -> group ~10 Hz | `TTMP` + microsPerBeat + beatOrigin + timeOrigin | esp -> looper timeline |
| 20812 | req/response | any datagram -> one-line JSON | status: peers/bpm/beat/phase/quantum/ap/metro |
| 34999 | esp -> gw / group | diagnostic | `log_tx_probe` every 10 s; nothing replies |

- **aloop binds only `224.76.78.75:20808`** (Pi AF_PACKET capture), so 20810 and 20812 are dropped
  at its IP layer — `Udp: IgnoredMulti` ~57/s while `NoPorts` is frozen. **20808 stays pure Link**
  (Link parses every datagram on it as `_asdp_v1`), so a peer wanting our phase must itself bind
  20810/20812.
- **Every multicast send must iterate the netifs and set `IP_MULTICAST_IF` per interface**
  (`esp_netif_next_unsafe`): a raw socket without it exits the wrong interface on a dual-netif
  device — Link's own socket reached the peer while ours didn't.
- **LCLK/TTMP are subscriber-driven** (`0a8760c`): without a hit on 20812/20811 within
  `SUBSCRIBER_HOLD_US` (15 s), clock drops 50 Hz -> 1 Hz, timeline 10 Hz -> 0.2 Hz. Clock
  rate-limited to one packet per 20 ms (timeline 100 ms): a 4 kHz flood saturates the Pi's radio-RX
  drain. `uc` sends ~15/s even with `peers=0` against ~14.6/s RX — volume tracks what we receive.
- **The looper sits behind a unicast-RX wall**: its bcm4343 delivers multicast but not
  unicast-to-self, so Link's ping/pong never completes there; LCLK gives it phase, TTMP carries
  tempo back. aloop's bcm4343 DOES deliver unicast, so that wall isn't reproduced there.
- Tempo bounded to **20..999 bpm** (`LINK_TEMPO_MAX_BPM 999.0`; Ableton Test Plan TEMPO-4).
  `beatAtTime` is quantum-insensitive, `phaseAtTime` isn't — the timeline broadcast uses
  `LINK_QUANTUM`. LTMP/phase requests are applied on the Link task, not the listener task.
- Status is request/response, not broadcast: can't pollute the Link group. The dev PC isn't on
  192.168.4.0/24, so probe it from a mesh station. `MetroStats` (`fired`/`last`/`worst`/`mean`/`rms`)
  rides in it as `metro`.

MIDI emission — one path, no per-device clock code; bytes are constants in `main.h` (`0xF8` clock,
`0xFA` start, `0xFC` stop, `0xFB` continue, `0xF2` SPP, CC123 all-notes-off);
`MIDI_PULSES_PER_QUARTER_NOTE = 24` and `MIDI_SPP_UNITS_PER_BEAT = 4` live in `link_sync.cpp`. SPP
+ Start/Continue ONLY at the 16-bar boundary; Stop + CC123 on all 16 channels on transport stop and
peer loss (CC123 keeps RC-505 MK2 loops from sticking). SPP counts MIDI beats (sixteenths): 1 Link
beat = 4 units, 14-bit LSB first. Start deferred to the next phrase boundary; `s_transport_running`
records what gear was last TOLD, not `isPlaying()`; force-start waits 8 s. NRPN sequence, note-off
velocity 0, UART TX ring buffer and `syncToBpm` are in `CLAUDE.md`. **No per-target compatibility
table exists in `link_sync.cpp`** — `CLAUDE.md:28` points at one that is not there; the KO2 /
RC-505 / MicroKorg per-target facts survive only as those two CC caveats.

## Live state 2026-10-11

- **Flashed 08:01:00Z..08:01:50Z** from CI binaries `51aded2` (source `eec4578` = `f0fc252` + docs):
  50 s, app 20.9 s @500.7 kbit/s, hash verified, `boot:0x13`. Cold boot scanned 1 `ticker` AP
  (`e4:5f:01:53:f0:f1`), joined STA `192.168.4.19`, IGMP ok, `txprobe e_gw=0 e_unknown=0 e_group=0
  e_sock=0`. First image carrying `dc9add9` + `f0fc252`; mc-420 re-soaking.
- **The ENOMEM send failure did NOT reproduce; cause unknown.** 1500 s soak on `df18c36`
  (06:07:25Z..06:32:22Z): 298 `peers=1`, zero `peers=0`/drops/`sendfail`/`IP lost`; `fin=55 fail=0`,
  `pingtx=3063 pingfail=0`, `ok=16307 fail=0 giveup=0 errno=0`; heap 123284-136176, rssi -42..-56.
  A 600 s run had one ~6 s burst (`err=12`, `fail=28 giveup=9`) inside an IGMP re-join; `0a8760c`
  read `fail=1501 giveup=396`. Call it *unreproduced*, not *fixed*.
- **Session bounce**: `enable(false)`/`enable(true)` drops peers to 0 and re-joins in **2540 ms**,
  adopting the peer's session id; beat monotonic, no tempo break. **The per-boot auto bounce was
  removed; only the on-demand `LRST` to `192.168.4.19:20811` (or `224.76.78.75:20811`) remains.**
- **`bad` (badmagic) is benign**: Link's own `_link_v1` measurement traffic, 1:1 with `Link msr: rx`.
- `esp_wifi_statis_dump(uint32_t modules)` is the only Wi-Fi buffer-statistics observation API; no
  public enum for its bitmask, so dump all with `0xffffffff`. AMPDU has **no runtime control** —
  only `wifi_init_config_t.ampdu_tx_enable` at init.

## Bass generator: 8 approaches, per-bar shaping

`main/state_machine.cpp` maps pad -> approach, single/double tap (`DOUBLE_TAP_WINDOW_US = 300000`):
0 `APP_ROLL`/`APP_ACID`, 1 `APP_FUNK`/`APP_GLITCH`, 2 `APP_STAB`/`APP_DRIFT`, 3
`APP_PEDAL`/`APP_BREAK`. `setApproach()` is the ONLY thing that activates the engine. **Every bar is
shaped, not repeated** — `shapeBar()` per bar from the 24-field `ApproachShape` (`bass_engine.h:80`);
replaying four 16-step motifs across 64 bars is the ringtone effect. **An approach is a VOICE**:
`dialsForApproach()` biases base dials -> `Dials::bandIndex` -> `kGrooves[9]`/`ContourShape[8]`/
`kScaleNames[9]`; a switch clears `m_hasPrevPhraseMotif` and sets `m_regenPending` (`0f1271f`).
PEDAL/DRIFT `minGate` 1.10/0.70 > 1 step = sustained voices, not sequencer runs. Microtiming caps
are asserted (max-abs <= 0.17 step, SD <= 0.15): don't raise `kMicroTimingFullDepth`;
`DEBOUNCE_COUNT = 2` (~500 us). Host-sim n/bar 1.66 (DRIFT) .. 13.14 (ROLL); uniq/64 13-60.

## HTTP clip server (8080) and SPIFFS

`main/network_midi.cpp` IS in the build: `network_midi_init()` starts `esp_http_server` on 8080 —
`POST /upload/*` -> `/spiffs/loops/<name>`, `GET /info` -> `{"device_ip":...}`, `POST /clear`
deletes `*.mid`. Invisible at the call sites: **4 sockets** of 16 reserved; SoftAP allows **8
stations** (`cfg.ap.max_connection = 8` = `MAX_AP_STA_IPS`) — also all the forwarder can unicast to,
so a 9th associates but gets no Link traffic; `WIFI_PS_NONE` — don't re-enable. **SPIFFS IS
mounted**: `mount_clip_storage()` runs first; `format_if_mount_failed = false` — never formats.
Contract: 503 unmounted, 413 body over `total - used`, 400 bad/empty name/body, 500 incomplete
(partial unlinked). URI wildcard is one segment re-checked (`is_single_clip_name`), so `/` and `..`
can't escape. `/info` re-reads the IP per request (netif recreated on every role change). Open: no
auth on an open SSID; `network_midi_start()`/`_stop()` dead; `filepath` truncation unchecked in
`clear_handler` (512 vs `CONFIG_HTTPD_MAX_URI_LEN=8192`); `d_type == DT_REG` (SPIFFS may leave
`DT_UNKNOWN`), `strstr(d_name, ".mid")` matches anywhere.

## Wiring

GPIO from `main/main.h` — a pin can't be expressed by reading the macro.

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

Touch pads use the **legacy `driver/touch_pad.h` API** (IDF also ships `touch_sensor`); a pad reads
LOW when touched; ARP read first (lowest press latency). Pots: raw ADC 0-4095 -> MIDI 0-127; in the
player pot1 = note length (0..2), pot2 = velocity (0..1), accessors still
`getPot1Value`/`getPot2Value`.

## Constants whose constraint is invisible in the code

- **E-Slew must stay derivable from sheer, anchored at 104**: `reset` sets sheer 0, so a derivation
  not returning 104 at sheer 0 makes reset overwrite what `setSidechainPattern()` just sent.
  `gateESlewForSheer(sheer)` = `kGateESlewDefault + sheer*kHeadroomAboveDefault/kSheerMax` (104 ->
  127 at sheer 0). 104 isn't magic; sheer 0 is.
- **Gate wet/dry must equal depth, not `127 - depth`**: `gateWetDryForDepth()` returns clamped depth
  and `SIDECHAIN_DEFAULT_DEPTH == _DEPTH_MAX == 127`, because MiniNova **CC 91 is wetness, 127 =
  full Gator** (Manual v1.01). All outside SRCS — nothing here runs on-device today.
- `kScales[3]` is `{0,3,5,7,10,3,5}` while `kScaleLens[3] == 5`: trailing `{3,5}` is padding so the
  row matches its neighbours; only the first five are read. Don't fix it.
- `kRegisterSpan = 15` (semitones, `bassline_interpreter.h`) must track the `anchorMotif` clamp in
  `bass_engine.cpp` — nowhere else; neither file includes the other.
- `kProgs` rows are **scale-degree indices**, not semitones, not MIDI notes (fixed `c0fc974`):
  `{0,5,3,6}` i-VI-iv-VII, `{0,6,5,6}` i-VII-VI-VII, `{0,3,6,2}` i-iv-VII-III, `{0,5,6,4}`
  i-VI-VII-V, `{0,2,6,3}` i-III-VII-iv, `{0,0,5,6}` pedal.
- `main.h` asserts `LINK_QUANTUM == METRONOME_ACCENT_CYCLE_BEATS`: the click's accent cycle IS the
  quantum — changing one alone breaks the build.

## Synth CC/NRPN facts not derivable from the code

- **MicroKorg**: LFO2 shape CC 75 (`kCcLfo2Shape`); delay sync CC 13 (`kCcDelaySync`), 0 = off,
  1..32 sync values.
- **MiniNova**: `LFO2_SYNC_ENABLE` NRPN (MSB 1, LSB 40) is an **assumption never confirmed on
  hardware** — the only NRPN with no verified source. Filter bypass (NRPN 0/60 value 0) is
  deliberately NOT sent by `deactivateFilter()`. **"Table 3"** is the source of
  `setDelaySyncRate`/`setLfoRateSync`; `main/lfo_constants.h` carries only NRPN 0/86 of that row.
  `main/nrpn_chart.txt` is the chart (was misspelled `nprn_chart.txt`).
- `SynthInterface` contracts live only in the implementations: `activateDelay()`/`activateReverb()`
  select Delay 1 / Reverb 1 in **FX Slot 1**; `setFxSlot1Level()` is **CC 91**;
  `activateFilter()` implicitly selects **LP24**; LFO methods assume **LFO2 -> Filter1 Freq via Mod
  Matrix Slot 1**.
- **LFO depth is bipolar**: `-64..+63` -> `0..127`, **64 = zero**, so `unpatchLfoFromFilter()` writes
  64, not 0.
- Note-off velocity 0 is the **default**: `sendNoteOff(uint8_t note, uint8_t velocity = 0)`
  declared identically across `synth_interface.h`, `synth_microkorg.h`, `synth_mininova.h`. The two
  surviving `= 64` defaults (cutoff, patch depth) must stay.

## Build shape

**NOT** in the build: `effect_arp.cpp`, `effect_filter.cpp`, `effect_handler.cpp`,
`effect_sidechain.cpp`, `midi_file.cpp` — nothing `#include`s a `.cpp`, so that cluster is reachable
only from itself; edits there can't break the build. `bassline_interpreter` must stay
**host-compilable**: only its own header plus `<algorithm>` `<cmath>` `<cstring>`, no ESP-IDF
headers, so `g++ -std=c++17` exercises its DP and scale logic off-device (MinGW: also
`-D_USE_MATH_DEFINES`, or `M_PI` is undefined). **`.gitattributes` pins EOL**: `* text=auto eol=lf`,
`eol=lf` for shell/py/cmake/csv/sdkconfig/Dockerfile, `eol=crlf` for `*.bat *.cmd`, `binary` for
`*.bin *.elf *.mid`; a CRLF-checked-out shell script dies in the Linux CI container. `midi_file.cpp`
(not built): running status, note-on vel 0 IS note-off, `process()` walks notes in `startBeat` order
and early-breaks (order load-bearing), `setFolder()` matches a prefix (SPIFFS has no directories),
`syncToBpm` false.

## CI autobump: how it commits, and its hazard

`build.yml` commits the three `.bin`s: records the sha it built, `git reset --hard FETCH_HEAD`,
restores the `.bin`s, commits with a 3-path pathspec, up to 5 attempts (recommit on the latest ref,
never rebase — a rebase would MERGE the previous `.bin`s and die on "Cannot merge binary files").
**`reset --soft` is NOT usable**: it moves HEAD to the fetched tip but leaves the index on the tree
this checkout built, so the commit silently reverts every source commit in between. Since `8454302`
`refuse_unless_bins_only()` returns 1 on any path outside the three; the binary commit is gated to
`main`.

**Residual rule: re-verify any commit touching a non-`.bin` path against HEAD after a pull** — a
green build never proves a change survived; whole commits have vanished (one autobump dropped 264
lines of AGENTS.md while shipping two `.bin`s). CI rewrites the `.bin`s on `main` on nearly every
push, so `remote_moved` is normal. Push with `git_push {recover_remote_moved: true}` (alias
`pull_first`); `git_finalize` auto-recovers, reports `auto_recovered:true`.

`../aloopprime` is the opposite — **fetch-only checkout** (`origin.pushurl = no-push`, no local git
identity): `git_finalize` there commits and gm refuses the push with `push_disabled_by_config`; a
merging `git_pull` dies with `git_identity_required`. Local-only commits are the intended end state;
don't "fix" it by editing its `.git/config` or adding a remote. Its shutdown SIGSEGV is fixed.

**mc-420** (Pi-side peer agent): its inbound pipe works; outbound `SendMessage` is **permanently
refused** ("No agent named 'mc-420' is reachable"), the uds address goes stale (`ECONNREFUSED` on
`\\.\pipe\LOCAL\cc-msg-*`), and `ListAgents` doesn't list it — so every reply is a file drop in
`C:\dev\mc-420\.gm\`. Don't retry SendMessage. Its `reassoc-watch.js` logs only `Date.now()` deltas,
so its absolute times must be reconstructed — always log a wall clock ourside.

## ESP-IDF 6.x breaks absorbed (historical; each silently re-breaks on a bump)

`esp_netif_next()` -> `esp_netif_next_unsafe()` (6.1, link's `ScanIpIfAddrs.hpp`); legacy
`driver/adc.h` removed -> `hall_sensor_read()` stub in `main/io_helpers.cpp`;
`CONFIG_LWIP_PBUF_POOL_SIZE` dropped.

## Comment sweep (INVARIANT 3)

`main/` and `tools/` are **comment-free** (41 files, 8809 lines, 0 comments; recount before
quoting). Only false positive: the string `"/upload/*"` in `network_midi.cpp`.

## gm usage gotchas

- `codesearch` is the canonical search verb; `{"help":true}` lists every field. Exhaustive modes
  (`literal`/`regex`/`filename`) REFUSE unknown fields; `dual` ignores them; `mode:"comments"` (or
  `{"comments":true}`) needs no query.
- `git_finalize`: **`paths` must be inside `body`** (top level -> `blanket_stage_refused`); never
  `git add` a gitignored path (e.g. `build/*.bin`).
