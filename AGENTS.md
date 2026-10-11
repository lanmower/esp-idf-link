# esp-idf-link — agent notes

`CLAUDE.md` `@`-includes this file; durable facts live here, not in code. Don't extend it by editing
`CLAUDE.md`. Compacted 2026-10-11; git log through `2c8807a` folded in.

## Mesh with `../aloopprime`: paired invariants (change BOTH or the mesh splits)

| Invariant | esp-idf-link | aloopprime |
|---|---|---|
| SSID | `main.cpp` `wifi_*("ticker")` | `hostapd.conf`, `wpa_supplicant.conf` |
| Auth | `wifi_connect_sta("ticker","")` | open (`key_mgmt=NONE`; `wpa=` commented) |
| AP addr / DHCP | `esp_netif_set_ip_info` `192.168.4.1/255.255.255.0` | `192.168.4.1/24`, dnsmasq `.2-.20` |
| Channel | SoftAP ch6 (`kTickerChannel = 6`) | `hostapd.conf` `channel=6` |
| Link multicast | `224.76.78.75:20808` | same (hardcoded in Link) |
| Quantum | `main.h` `LINK_QUANTUM 16.0` | `link_bridge.cpp` `quantum = 16.0` |
| Start/stop sync | `enableStartStopSync(false)` | same, `false` |
| Host election | lowest MAC/BSSID wins | lowest MAC/BSSID wins |
| Tempo authority | sets only on explicit LTMP | refuses to propose when peers own tempo |

`PHRASE_BEATS 64.0` is NOT the quantum — the SPP/transport boundary (16 bars). Pi spells `ticker` in
`config/aloop.conf:62`, `autoap.sh:7`. Audited 2026-10-11: all MATCH; asymmetries deliberate: IGMP
one-sided (pi none, `ap_isolate=0`), scan stagger one-sided, takeover esp ~60 s vs pi ~90-160 s.

### Host election

Two cold-booting devices each scan before the other's AP exists, so "nothing found -> host" makes
BOTH host — Link can't cross two isolated L2 domains. Hence `HOLD_MAX_MS = 6000`, monotonic in own
STA MAC (`mac_rank * HOLD_MAX_MS >> 24`), rescan 1/s, stagger `mac[5] * 15` ms;
`../aloopprime/src/net/autoap.sh` mirrors (0-6 s, lowest-wins). Invert both in one change.

Supervisor (`wifi_config.cpp`, 2 s cadence; `RECONNECT_TRIES=30`, IGMP re-assert every 30 s, tx probe
every 10 s):
- STA: a dropped link reconnects up to 30 times (~60 s) before re-hosting.
- **STA is "up" only when it has an IP** (`579d4ef323`): `g_wifi_connected` was set only by
  `IP_EVENT_STA_GOT_IP`, cleared only by `WIFI_EVENT_STA_DISCONNECTED`, with no `IP_EVENT_STA_LOST_IP`
  handler, so an association outliving its IP left the flag true and the supervisor did nothing. Link
  drops a netif whose address is 0 and erases the gateway; with zero gateways it neither sends nor
  receives — **peers 1->0 while still associated, no Wi-Fi event**. Gate is now
  `g_wifi_connected && sta_has_ip()`. Best match for mc-420's silent deaths; **unproven**.
- AP: re-scans; yields to a strictly-lower `ticker` BSSID **only while it has ZERO stations**
  (`ap_has_associated_stations()`). **Comparison MAC is the STA MAC** (`e4:65:b8:77:0d:14`), not the
  SoftAP MAC (`...:15`); Pi `e4:5f:01:53:f0:f1` is lower, so this box yields. **Self-BSSID guard reads
  the interface** (`esp_wifi_get_mac(WIFI_IF_AP)`, never `esp_read_mac()`, which reads eFuse) or the
  box flaps AP->STA->AP every tick (`5faf01a`).
- IGMP re-joined on yield, not only at boot (`d1b7f9a`), and since `579d4ef` also in **AP role**, so a
  long-lived SoftAP can't age its join out. Lease proof: `WIFI: IP: 192.168.4.x`.
- **Hosting scan pinned to the mesh channel**: it needs `ensure_sta_started()` -> `WIFI_MODE_APSTA`,
  and an all-channel scan dwells off-channel, interrupting SoftAP beaconing, so a station associating
  inside a scan window never finishes DHCP. Supervisor passes `kTickerChannel`; boot/hold scans stay
  `kScanAllChannels`. Cost: a host that left ch6 is never seen. **AP->STA yield paths run only after a
  device has hosted once**, so a soak that never leaves STA role exercises none of them.

**Reading `peers=0`:** `gw(ok=1)` = gateway up; `dst=` on a high port (`192.168.4.1:54146`) means the
announcement arrived and Link is unicast-pinging — failure is ping/pong, NOT multicast. **`dst` is
HOST order** (asio `to_v4().to_uint()` vs network-order `ip4_addr_t::addr`), so `htonl` before
`IP2STR`; pre-`579d4ef` images printed it reversed — same trap as `99.168.4.1`.

**How Link drops a peer** (read-only tree, do NOT patch): `UdpMessenger.hpp:74-77` swallows send
errors and `platforms/esp32/Context.hpp` discards the exception handler, so `asio/Context.hpp`'s catch
loop isn't compiled and **`repairGateway` is dead**; the equivalent (gateway erase + 5 s rescan) runs
via `PeerGateways` `Callback::operator()`. Routes: TTL ~6 s; ByeBye; **`gatewayClosed`**
(`~GatewayObserver`, `Peers.hpp:136-144`) erases ALL peers on that address — the
1->0-with-no-Wi-Fi-event route, also fired by `enable(false)`'s `clear()`; `resetPeers()`.

### By-the-book Link checklist (both trees)

- `captureAppSessionState()`/`commitAppSessionState()` off the audio thread; `AudioSessionState`
  variants only on it. This tree uses the App variants.
- **Start/stop sync OFF on purpose: mesh is clock-only** (`b94ac2d`); `isPlaying()` then reads
  **false** all session (transport is ESP-local). Re-enabling needs BOTH trees + a flash here.
- The three notification callbacks run on a Link thread, **Realtime-safe: no** — bounded logging /
  atomics only. `setTempo` rewrites tempo for EVERY peer; this tree sets it only on explicit LTMP.
- **Interface readiness is a real race:** 500 ms settle before constructing Link, then IGMP re-asserted
  every 30 s — never a one-shot join.
- **The Pi advertises one NodeState across every IFF_RUNNING interface**, so its `mep4` flips
  `192.168.4.1` <-> ICS `192.168.137.100`; `Peers::sawPeerOnGateway` cached whichever arrived last.
  Off-link `mep4` is **counted and logged, never dropped** (`cfddf6d` reverted `4481a5a`).

### The vendored Link is a checked-in FORK, not a submodule

`components/link-esp/link` is a patched copy committed directly (1506 files, no nested `.git`);
`git submodule update` would clobber the patch with no compile error and silently kill ESP peer
discovery — hence the stale `.gitmodules` entry was removed; `Dockerfile`, `setup.sh`, `build.yml` do
NOT init submodules. Our patches live in `Socket.hpp`, `PeerGateways.hpp`, `UdpMessenger.hpp`,
`Controller.hpp`, `Peers.hpp` — re-apply on any bump.

Two API facts, each costing a CI failure (both compile on host, break on-device): vendored standalone
asio takes **flags AFTER the endpoint** (`send_to(buffer, to, 0, ec)`), opposite of Boost's
`(buffer, flags, to, ec)`; `PeerState::endpoint` is a **data member**, while
`MeasurementService`/`PingResponder`/`Socket::endpoint()` are methods. **Compiled asio is
`components/espressif__asio` 1.32.0, not vendored `link/modules/asio-standalone` 1.17.0** — so
`asio/error.hpp` maps only `no_buffer_space = ENOBUFS`, yet the send failure prints `send_to: Not
enough space` (`strerror_r` for **errno 12 / ENOMEM**). Tempo rides the wire as **int64
microseconds-per-beat, truncated** (`Tempo.hpp:52`): 119.049 round-trips with ~1e-5 bpm error.

**The relay hook is the divergence**: `platforms/asio/Socket.hpp` declares `extern "C"
wifi_link_multicast_forward(...)` and calls it on every send, so `wifi_config.cpp` can unicast-copy
Link discovery datagrams across the SoftAP boundary. **Any Link bump must re-apply it** — losing it
fails only as silent non-discovery. Gated `if (!g_ap_active) return;`. `Socket::send` retries a
failed `send_to` 3x, exponential 1..8-tick backoff reset on success (`cfddf6d`, `0a8760c`), recording
ok/retry/attempt/giveup + errno instead of throwing.

## SoftAP multicast gap, and lwIP core context

ESP32 SoftAP doesn't carry Link multicast between host and stations, hence
`link_multicast_relay_task`: re-emits each datagram to the group, to the AP's own IP, and unicast to
every associated station, preserving the source IP (Link needs it for direct peer connect).
aloopprime's `brcmfmac` sharing the gap is UNVERIFIED (`ap_isolate=0` may suffice) — don't port the
relay by assumption. **The relay is a singleton, `g_ap_netif` mutex-guarded** (`0bffe4c`):
`link_relay_release()` frees socket + raw pcb on every path.

**`LOCK_TCPIP_CORE()` / `UNLOCK_TCPIP_CORE()` are EMPTY MACROS in this build** (`2c8807a`):
`CONFIG_LWIP_TCPIP_CORE_LOCKING` is unset and Kconfig defaults to `n`, so
`LWIP_TCPIP_CORE_LOCKING == 0` — every site that used them ran lwIP core work from a foreign task with
**no exclusion at all**. Real serialization is
`esp_netif_tcpip_exec(esp_netif_callback_fn fn, void* ctx)` (callback runs on the tcpip thread; exec
blocks until it returns). All IGMP joins and all relay raw-pcb work (`raw_new`, `raw_remove`,
`raw_sendto_if_src`) now go through it. **Never hold `ap_netif_lock` across an
`esp_netif_tcpip_exec`** — the relay takes ap lock then blocks on the tcpip thread, so the supervisor
snapshots `g_ap_netif` under the lock and joins outside it.

**The AP->STA yield is where role-switch leaks lived** (`dc9add9`): the relay was never stopped there
(`g_relay_stop` was only ever set inside `wifi_start_link_relay`), `g_relay_task` was non-volatile
under a spin, and the forward socket was a function-local `static` — one of only 16 — now closed on
yield under a mutex the send hook also takes, so `close()` can't land mid-`sendto`.
`wifi_stop_link_relay()` bounds its wait (100 x 20 ms). `wifi_join_link_multicast()` returns unless
the STA already has an IP — the supervisor re-asserts within 2 s of `sta_up`, so no report goes out
from 0.0.0.0.

## Clock is hardware-scheduled, never tick-emitted

Click and 24 ppqn clock come from an `esp_timer` one-shot armed to `SessionState::timeAtBeat()` of the
NEXT pulse, never the 4 kHz gptimer tick (`LINK_TICK_PERIOD 250` us), which discovers a beat only
after it passed; raising the tick rate can't tighten the click. Link's ESP clock IS
`esp_timer_get_time()`, so a Link timestamp is already an esp_timer deadline.

- **Skip-don't-catch-up:** the scheduler steps past pulses already due (capped
  `MAX_CATCHUP_PULSES_PER_SCHEDULE = 1024`); a burst reads as a tempo spike downstream.
  `MIDI_CLOCK_RESYNC_THRESHOLD` is log-only — it changes nothing.
- Same rule on the note path: `BassEngine::process` fires notes in `(last, cur]`; past
  `kMaxAdvanceStepsPerProcess` it releases note-offs, re-anchors `m_lastPhrasePosSteps`, emits nothing
  (`5598ed6`). **A tempo change doesn't trip it**; only a phase jump does.
- **`volatile` bites twice**: `++` on a volatile counter is a hard compile error (`23e5170`, so
  `s_fire_count` is plain `uint32_t`); `xTaskCreate`'s last parameter is non-volatile `TaskHandle_t*`
  (`f0fc252`) — create into a local, then assign.

**`sdkconfig.defaults` is load-bearing, not tuning.** `CONFIG_LWIP_MAX_SOCKETS=16` (16 is the max):
the default 10 is exhausted by the app's own sockets (relay, LCLK/TTMP, tempo listener, discovery
forward, 4 for HTTP), leaving too few for Link's per-interface discovery gateway -> EMFILE -> peers
never discover. Also `LWIP_TCPIP_RECVMBOX_SIZE=64` (32->64) — ENOMEM from `sendto` means EITHER pool
drained OR mailbox full; `LWIP_UDP_RECVMBOX_SIZE=6`, `ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM=64`,
`LWIP_STATS=y` (read `mem.err`/`link.drop`/`link.memerr`) — all `cfddf6d`.
**`CONFIG_LWIP_PBUF_POOL_SIZE` is NOT a v6.1 symbol** and is removed; don't re-add.

## Firmware reaches the ticker over USB only

- CI builds on push to any branch. **The IDF image is digest-pinned** in `Dockerfile:1`
  (`ARG IDF_IMAGE=espressif/idf@sha256:1355cce31b...fea95`); `resolve-idf-image` exits 1 unless the
  digest is exactly 64 lowercase hex digits — a toolchain bump is that one line. On green `main` it
  commits the three images under `build/`; flashable images come from git. Runners pinned
  `ubuntu-24.04` (`3a4413b`): GitHub moves `ubuntu-latest` to Ubuntu 26 on 2026-10-19.
- Offsets: 0x1000 bootloader, 0x8000 partition table, app at the first app partition of the **BUILT**
  table (0x20000 with `partitions_large.csv`). **The app offset is DERIVED, never hardcoded**
  (`31a6b99`): `flash-ticker.js`, `flash.sh`, `tools/flash-usbip.py` read
  `build/partition_table/partition-table.bin` — 32-byte entries, LE magic `0x50AA`, then type(1)
  subtype(1) offset(4) size(4) label(16), app type == 0. **Never parse `partitions_large.csv`** —
  every offset but nvs's is blank, so parsing yields `int('')`. `--list` refuses to guess when >1
  serial port exists (this machine has two Bluetooth COM ports that aren't the ESP32).
- esptool 5.x: `write-flash` is hyphenated; **`--verify` was dropped** (it always verifies). SPIFFS
  `storage` flashes separately: `flash_midi_data.sh` builds a `0x19000` image from `./data` at
  **`0x317000`** (derivable: nvs 0x011000, phy_init 0x017000, app0 0x020000, app1 0x1A0000, storage
  0x317000; app partitions round up to 0x10000).

### Hands-free flash: `python tools/flash-usbip.py`

`flash-ticker.js` needs BOOT/IO0 held — a WCH/CH340 driver limit, not the board; `flash-usbip.py`
talks CH341 over USB/IP (usbipd-win) behind a pyserial shim, so the WCH driver never loads (WSL2
/CH340 setup is in `CLAUDE.md`). Measured: exit 0, ~49 s, app in ~19.8 s at 460800, "Hash of data
verified", boots `boot:0x13`, no BOOT hold, no WSL.

1. **The D1 R32's auto-reset is DIFFERENTIAL.** EN low only at (DTR# high, RTS# low) `0x40`; IO0 only
   at (DTR# low, RTS# high) `0x20`; `0x60`/`0x00` conduct NEITHER — opposite of esptool's convention,
   hence `boot:0x13` on every esptool reset. Entry is the flip **0x40 -> 0x20** (`boot:0x3`). The
   CH341 handshake register is active-low: **bit6 = RTS#/EN, bit5 = DTR#/IO0** (once inverted, fixed
   `9ddd92f`).
2. **The CH341 withholds the first replies**: a sync frame goes unanswered for seconds, then surfaces
   behind a later FULL-SIZE OUT; prime with repeated framed syncs (`SYNC_TIMEOUT` 2 s).
3. **The first attach back often fails**: usbipd keeps the device claimed, so `OP_REP_IMPORT` returns
   `status=4`; `open_port()` retries (10 x 6 s).
4. **`USBIPD_BUSID` in `tools/ch341.py` is THIS MACHINE's busid (`2-2`)**, not a board property;
   re-read per host and after any replug, and `usbipd bind --busid <busid>` once per boot.

Also: **`--after no-reset-stub`, never `--after no-reset`**, and guard `esptool.main()` with `except
BaseException` (`09da1052`) — `no-reset` soft-resets at the flash baud while the ROM speaks 115200 ->
`FatalError: No more data to read` AFTER all three images verified, and `FatalError` isn't
`SystemExit`. That guard is a bounded retry loop (`FLASH_ATTEMPTS = 6`); "Serial data stream stopped"
right after "Connected to ESP32" proved transient. **`esp.read_mac()`/`flash_id()` are wrapped
non-fatal** (`579d4ef`): a `FatalError` from either aborted **after download-mode entry**, stranding
the board at `boot:0x3` — **nothing between sync and `boot_app()` may raise**. `UsbipPort.timeout` has
a **2.0 s floor** (`_min_timeout`) or a late CH341 reply reads as a dead stream. The ROM only speaks
115200 (`ROM_BAUD`); `--baud` applies only after the RAM stub re-times it (115200 -> 72 s vs 460800 ->
19 s; 921600 dies, PRD `flash-ticker-js-921600-unmeasured`). `UsbipPort()` from `tools/usbip-port.py`
MUST have `.init()` called (else 4096 zero bytes per bulk IN floods the output); import that
hyphenated name via `importlib.util.spec_from_file_location`.

## Mesh UDP protocol and MIDI emission

Payloads little-endian `int64`; ports are `#define`s in `link_sync.cpp`: `LINK_PHASE_PORT 20810`,
`LINK_TEMPO_PORT 20811`, `LINK_TTMP_PORT`/`LINK_STATUS_PORT` 20812, `TX_PROBE_PORT 34999`.

| Port | Direction | Payload | Purpose |
|---|---|---|---|
| 20810 | esp -> group ~50 Hz | `LCLK` + link-clock micros | phase for peers behind a unicast-RX wall |
| 20811 | looper -> esp | `LTMP` + microsPerBeat (+ beat0 micros + quantum microbeats) | looper sets tempo and phase |
| 20811 | request | `LRST` | `enable(false); enable(true); enableStartStopSync(false)` — no Wi-Fi teardown |
| 20812 | esp -> group ~10 Hz | `TTMP` + microsPerBeat + beatOrigin microbeats + timeOrigin micros | esp -> looper timeline |
| 20812 | req/resp | any datagram -> one-line JSON | status: peers/bpm/playing/beat/phase/quantum/ap/metro |
| 34999 | esp -> gw / group | diagnostic | `log_tx_probe` every 10 s + `log_send_failure_probe`; nothing replies |

- **aloop binds only `224.76.78.75:20808`** (Pi AF_PACKET capture), so 20810 and 20812 are dropped at
  its IP layer — `Udp: IgnoredMulti` ~57/s while `NoPorts` is frozen. **20808 stays pure Link**, so a
  peer wanting our phase must itself bind 20810/20812. The looper's bcm4343 delivers multicast but
  not unicast-to-self, so Link's ping/pong never completes there — that is what LCLK/TTMP exist for;
  aloop's bcm4343 DOES deliver unicast, so it doesn't reproduce that wall.
- **Every multicast send must iterate the netifs and set `IP_MULTICAST_IF` per interface**
  (`esp_netif_next_unsafe`): without it a raw socket exits the wrong interface on a dual-netif device
  — Link's own socket reached the peer while ours didn't.
- **LCLK/TTMP are subscriber-driven** (`0a8760c`): without a hit on 20812 or 20811 within
  `SUBSCRIBER_HOLD_US` (15 s), clock drops 50 Hz -> 1 Hz, timeline 10 Hz -> 0.2 Hz; clock is also
  rate-limited to one packet per 20 ms (timeline 100 ms). **`peers` readouts are not passive** —
  polling the ESP changes its broadcast rate for 15 s.
- Tempo bounded to **20..999 bpm** (`LINK_TEMPO_MAX_BPM 999.0`, `396092d`). `beatAtTime` is
  quantum-insensitive, `phaseAtTime` isn't — the timeline broadcast uses `LINK_QUANTUM`. LTMP/phase
  requests are applied on the Link task, not the listener task. Status is request/response, not
  broadcast; `MetroStats` rides in it as `metro` — only evidence of click lateness. NRPN sequence,
  note-off velocity 0, UART TX ring buffer, `syncToBpm` and the per-target table are in `CLAUDE.md`
  and the top of `link_sync.cpp`.

MIDI emission — one path, no per-device clock code; bytes are constants in `main.h` (`0xF8` clock,
`0xFA` start, `0xFC` stop, `0xFB` continue, `0xF2` SPP, CC123 all-notes-off);
`MIDI_PULSES_PER_QUARTER_NOTE = 24` and `MIDI_SPP_UNITS_PER_BEAT = 4` live in `link_sync.cpp`. SPP +
Start/Continue ONLY at the 16-bar boundary; Stop + CC123 on all 16 channels on transport stop and
peer loss. SPP counts MIDI beats (sixteenths): 1 Link beat = 4 units, 14-bit LSB first. Start deferred
to the next phrase boundary; `s_transport_running` records what gear was last TOLD, not `isPlaying()`.

## Observing the running mesh (2026-10-11)

1. **Pi shell** (`node C:/dev/mc-420/.gm/exec-ssh.js "<cmd>"`, root@192.168.137.100) then
   `printf 'PING' | timeout 3 nc -u -w1 192.168.4.19 20812`. Only way to reach 192.168.4.x; also how
   `LRST` is fired. Every datagram is a subscriber hit (above).
2. **aloop telemetry, straight from the PC**: UDP `status` -> `192.168.137.100:4445` -> JSON in ~190
   ms — one hop (the PC is on the ICS subnet), no Pi shell, and it costs the ESP nothing.

- **Pi clock reads 2024-04-09** — ~915 days behind; `ip route` has no default route, so no NTP.
- **`ip_forward` on the Pi is 0**, so no PC can route to 192.168.4.0/24 and `route add` is pointless
  twice over. Turning it on would expose the open SSID to 192.168.137.0/24; don't, unless asked.
- ESP status JSON `bpm` is **3 dp**; aloop `link.bpm` is **1 dp** and `groove.beat_len_samples` @48 kHz
  implies a third. Useful aloop fields: `xruns`, `link{synced,bpm,peers,phase_err_beats}`, `wifi`,
  `eff_speed`, `beat_mark{beat,late_ms}`, `groove`, `loopers`.
- **Tempo cross-check, unresolved**: ESP 118.631 vs aloop `link.bpm` 118.6 vs `beat_len_samples`
  24277 (⇒ 118.594), while mc-420's PASS run quoted 118.500. `phase_err_beats` 0.000 and `eff_speed`
  1.00000 say the boxes share one timeline, so this is readouts, not two tempos — pending mc-420's
  answer on whether `link.bpm` is session tempo or derived, and whether aloop imposes 118.500.

## Live state 2026-10-11 (`ac377d6` -> `2c8807a`)

- **The ENOMEM send failure did NOT reproduce; cause unknown.** 1500 s soak on `df18c36`
  (06:07:25Z..06:32:22Z): 298 `peers=1` samples, ZERO `peers=0`, zero drops/`sendfail`/`IP lost`; 50
  IGMP re-joins; `fin=55 fail=0`, `pingtx=3063 pingfail=0`, `pongerr` empty; send `ok=16307 fail=0
  giveup=0 errno=0`; heap 123284-136176 (min 114384), rssi -42..-56, `t_gw` 2065-19418 us; peers=1
  2.1 s after Link init and held. The only real ENOMEM evidence is one ~6 s burst at t=218.9 s in a
  600 s run (`err=12`, `fail=28 giveup=9`, inside an IGMP re-join); two other `e_*=12` sightings were
  the 75 s probe's own bugs (`99.168.4.1`, no `IP_MULTICAST_IF`).
- **Session bounce**: `enable(false)`/`enable(true)` drops peers to 0 and re-joins in **2540 ms**,
  adopting the peer's session id; beat monotonic, no tempo break. **The per-boot auto bounce was
  removed; only on-demand `LRST` to `192.168.4.19:20811` (or `224.76.78.75:20811`) remains.**
- **`bad` (badmagic) is benign**: Link's own `_link_v1` measurement traffic, 1:1 with `Link msr: rx`.
- Peer-loss instrumentation (`44ce414964`, on `main`, not yet flashed): `peerloss: flag= ap= ap_sta=
  ip= rssi= chan= igmp(ok= fail=) heap= min=` at the 1->0 transition + `Link peer back after N ms`;
  `rssi=0 chan=0` means `esp_wifi_sta_get_ap_info` failed.
- `esp_wifi_statis_dump(uint32_t modules)` is the only Wi-Fi buffer-statistics API (`esp_wifi.h`
  v6.1); no public enum for its bitmask, so dump all with `0xffffffff`. AMPDU has no runtime control —
  only `ampdu_tx_enable` at init (`3d703ce`).

## Bass generator: 8 approaches, per-bar shaping

`main/state_machine.cpp` maps pad -> approach, single/double tap (`DOUBLE_TAP_WINDOW_US = 300000`): 0
`APP_ROLL`/`APP_ACID`, 1 `APP_FUNK`/`APP_GLITCH`, 2 `APP_STAB`/`APP_DRIFT`, 3 `APP_PEDAL`/`APP_BREAK`.
`setApproach()` is the ONLY thing that activates the engine. **Every bar is shaped, not repeated** —
`shapeBar()` per bar from the 24-field `ApproachShape` (`bass_engine.h:80`); replaying four 16-step
motifs across 64 bars is the ringtone effect. **An approach is a VOICE, not just a preset**
(`0f1271f`): `dialsForApproach()` biases base dials -> `Dials::bandIndex` ->
`kGrooves[9]`/`ContourShape[8]`/`kScaleNames[9]`; a switch clears `m_hasPrevPhraseMotif` and sets
`m_regenPending`. PEDAL/DRIFT `minGate` 1.10/0.70 > 1 step = sustained voices. Asserts cap microtiming
(max-abs <= 0.17 step, SD <= 0.15) — don't raise `kMicroTimingFullDepth`.

## HTTP clip server (8080) and SPIFFS

`main/network_midi.cpp` IS in the build: `network_midi_init()` starts `esp_http_server` on 8080 —
`POST /upload/*` -> `/spiffs/loops/<name>`, `GET /info` -> `{"device_ip":...}`, `POST /clear` deletes
`*.mid`. Invisible at the call sites: **4 sockets** of 16 reserved; SoftAP allows **8 stations**
(`cfg.ap.max_connection = 8` = `MAX_AP_STA_IPS`) — also all the forwarder can unicast to, so a 9th
associates but gets no Link traffic; `WIFI_PS_NONE` (a dozing station misses multicast) — don't
re-enable. **SPIFFS IS mounted** (`9568578`): `mount_clip_storage()` runs first;
`format_if_mount_failed = false` — never formats. Contract: 503 unmounted, 413 body over `total -
used`, 400 bad/empty name/body, 500 incomplete (partial unlinked). URI wildcard is one segment
re-checked (`is_single_clip_name`), so `/` and `..` can't escape; `/info` re-reads the IP per request
(netif recreated on every role change). Open: no auth on an open SSID; `network_midi_start()`/`_stop()`
dead; `clear_handler` `filepath` truncation unchecked; `d_type == DT_REG` unproven.

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

Touch pads use the **legacy `driver/touch_pad.h` API** (IDF also ships `touch_sensor`); a pad reads LOW
when touched; ARP read first (lowest press latency). Pots: raw ADC 0-4095 -> MIDI 0-127; in the player
pot1 = note length (0..2), pot2 = velocity (0..1), accessors `getPot1Value`/`getPot2Value`.

## Constants whose constraint is invisible in the code

- **E-Slew must stay derivable from sheer, anchored at 104**: `reset` sets sheer 0, so a derivation
  not returning 104 at sheer 0 makes reset overwrite what `setSidechainPattern()` just sent.
  `gateESlewForSheer(sheer)` = `kGateESlewDefault + sheer*kHeadroomAboveDefault/kSheerMax`. 104 isn't
  magic; sheer 0 is.
- **Gate wet/dry must equal depth, not `127 - depth`**: `gateWetDryForDepth()` returns clamped depth
  and `SIDECHAIN_DEFAULT_DEPTH == _DEPTH_MAX == 127`, because MiniNova **CC 91 is wetness, 127 = full
  Gator** (Manual v1.01). All outside SRCS — nothing here runs on-device today.
- `kScales[3]` is `{0,3,5,7,10,3,5}` while `kScaleLens[3] == 5`: trailing `{3,5}` is padding so the row
  matches its neighbours; only the first five are read. Don't fix it.
- `kRegisterSpan = 15` (semitones, `bassline_interpreter.h`) must track the `anchorMotif` clamp in
  `bass_engine.cpp` — nowhere else; neither file includes the other.
- `kProgs` rows are **scale-degree indices**, not semitones, not MIDI notes (fixed `c0fc974`):
  `{0,5,3,6}` i-VI-iv-VII, `{0,6,5,6}` i-VII-VI-VII, `{0,3,6,2}` i-iv-VII-III, `{0,5,6,4}` i-VI-VII-V
  ("dark cadence"), `{0,2,6,3}` i-III-VII-iv, `{0,0,5,6}` pedal.
- `main.h` asserts `LINK_QUANTUM == METRONOME_ACCENT_CYCLE_BEATS`: the click's accent cycle IS the
  quantum — changing one alone breaks the build.

## Synth CC/NRPN facts not derivable from the code

- **MicroKorg**: LFO2 shape CC 75 (`kCcLfo2Shape`); delay sync CC 13 (`kCcDelaySync`), 0 = off, 1..32
  sync values.
- **MiniNova**: `LFO2_SYNC_ENABLE` NRPN (MSB 1, LSB 40) is an **assumption never confirmed on
  hardware**. Filter bypass (NRPN 0/60 value 0) is deliberately NOT sent by `deactivateFilter()`,
  which unpatches only the LFO. `main/nrpn_chart.txt` is the chart (was misspelled `nprn_chart.txt`,
  `3d42c32`); `main/lfo_constants.h` carries only NRPN 0/86 of the "Table 3" row behind
  `setDelaySyncRate`/`setLfoRateSync`.
- `SynthInterface` contracts live only in the implementations: `activateDelay()`/`activateReverb()`
  select Delay 1 / Reverb 1 in **FX Slot 1**; `setFxSlot1Level()` is **CC 91**; `activateFilter()`
  implicitly selects **LP24**; LFO methods assume **LFO2 -> Filter1 Freq via Mod Matrix Slot 1**.
- **LFO depth is bipolar**: `-64..+63` -> `0..127`, **64 = zero**, so `unpatchLfoFromFilter()` writes
  64, not 0. Note-off velocity 0 is the **default** (`sendNoteOff(uint8_t note, uint8_t velocity = 0)`
  across all three headers); the two surviving `= 64` defaults (cutoff, patch depth) must stay.

## Build shape

**NOT** in the build: `effect_arp.cpp`, `effect_filter.cpp`, `effect_handler.cpp`,
`effect_sidechain.cpp`, `midi_file.cpp` — nothing `#include`s a `.cpp`, so that cluster is reachable
only from itself; edits there can't break the build. `bassline_interpreter` must stay
**host-compilable**: only its own header plus `<algorithm>` `<cmath>` `<cstring>`, no ESP-IDF headers,
so `g++ -std=c++17` exercises its DP and scale logic off-device (MinGW: also `-D_USE_MATH_DEFINES`,
or `M_PI` is undefined). **`.gitattributes` pins EOL** (`7e8dacd`): `eol=lf` for
shell/py/cmake/csv/sdkconfig/Dockerfile, `eol=crlf` for `*.bat *.cmd`, `binary` for `*.bin *.elf
*.mid`; a CRLF-checked-out shell script dies in the Linux CI container.

## CI autobump: how it commits, and its hazard

`build.yml` commits the three `.bin`s: records the sha it built, `git reset --hard FETCH_HEAD`,
restores the `.bin`s, commits with a 3-path pathspec, up to 5 attempts (`111d902`: recommit on the
latest ref, never rebase — a rebase would MERGE the previous `.bin`s and die). **`reset --soft` is NOT
usable**: it leaves the index on the tree this checkout built, so the commit silently reverts every
source commit in between. `refuse_unless_bins_only()` (`8454302`) returns 1 on any path outside the
three; the binary commit is gated to `main` (`d0afa95`).

**Residual rule: re-verify any commit touching a non-`.bin` path against HEAD after a pull** — a green
build never proves a change survived; whole commits have vanished (one autobump dropped 264 lines of
AGENTS.md while shipping two `.bin`s). CI rewrites the `.bin`s on `main` on nearly every push, so
`remote_moved` is normal — push with `git_push {recover_remote_moved: true}`; `git_finalize`
auto-recovers.

`../aloopprime` is the opposite — **fetch-only checkout** (`origin.pushurl = no-push`, no local git
identity): `git_finalize` there commits and gm refuses the push with `push_disabled_by_config`.
Local-only commits are the intended end state; don't "fix" it by editing its `.git/config`.

**mc-420** (Pi-side peer agent): `SendMessage` to it is **permanently refused** and `ListAgents`
doesn't list it, so every reply is a file drop in `C:\dev\mc-420\.gm\` — don't retry SendMessage. Its
`reassoc-watch.js` logs only `Date.now()` deltas, so its absolute times must be reconstructed. Its Pi
ssh credential (`root`/`aloop`@192.168.137.100, from `C:\dev\mc-420\.gm\exec-ssh.js`) is for read-only
probing on our own device and must never be committed.

## ESP-IDF 6.x breaks already absorbed

Historical — none visible in current source, each silently re-breaks on a bump:
`esp_netif_next()` -> `esp_netif_next_unsafe()` (6.1, link's `ScanIpIfAddrs.hpp`); legacy
`driver/adc.h` removed (6.1) -> `hall_sensor_read()` stub in `main/io_helpers.cpp`;
`CONFIG_LWIP_PBUF_POOL_SIZE` dropped (not a v6.1 symbol).

## Comment sweep (INVARIANT 3)

Measured 2026-10-11: **0 comments** across `main/` and `tools/`. Recount before quoting a number.

## gm usage gotchas

- `codesearch` is the canonical search verb; `{"help":true}` lists every field. Exhaustive modes
  (`literal`/`regex`/`filename`) REFUSE unknown fields; `mode:"comments"` needs no query; scope with
  `path`/`files`/`glob` (`scope` is refused).
- `git_finalize` takes `message`/`paths`/`files`/`allow_whole_index`/`recover_remote_moved`/
  `source_ref`/`rev` — `body` or top level, `body` wins. **`paths` must be inside `body`** (top level
  -> `blanket_stage_refused`). Never `git add` a gitignored path (e.g. `build/*.bin`).
- `git_show` takes `rev`/`stat`/`path`; `git_log` takes `limit`/`range`/`path`. `bash`/`exec_js` take
  their script in `body` or `raw_body`. `session_id` is mandatory on every call; `fs_readdir` refuses
  out-of-root paths without `allowOutsideRoot:true`. `python3` is absent on this box — use node.
