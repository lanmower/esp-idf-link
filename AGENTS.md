# esp-idf-link — agent notes

`CLAUDE.md` includes this file. Durable cross-session facts belong here, not
inline in the code.

## esp-idf-link <-> aloop mesh: paired invariants (change BOTH or the mesh splits)

This project (the ESP32 "ticker" box) and `../aloop` (the Pi 4 hardware
looper) form ONE ad-hoc single-AP mesh so Ableton Link's multicast peer
discovery reaches every device. There is no credential provisioning:
exactly one device hosts the open SSID `ticker` and everyone else joins it
as a station — whichever device boots first wins, Pi or ESP32. Every value
below exists in BOTH trees; changing it in one project alone silently stops
the two from meshing, with no error on either side.

| Invariant | esp-idf-link | aloop |
|---|---|---|
| Mesh SSID | `main.cpp` `wifi_scan_best_bssid("ticker")` / `wifi_start_link_ap("ticker")` / `wifi_start_supervisor("ticker")` | `src/net/config/hostapd.conf` `ssid=ticker`, `wpa_supplicant.conf` `ssid="ticker"` |
| Auth | `wifi_connect_sta("ticker", "")` (open, empty password) | open (`key_mgmt=NONE`; `wpa=` lines commented out) |
| AP address / DHCP | `wifi_config.cpp` `esp_netif_set_ip_info` `192.168.4.1/255.255.255.0` | `192.168.4.1/24`, dnsmasq `.2-.20` |
| Channel | SoftAP ch6 | `hostapd.conf` `channel=6` |
| Link multicast | `224.76.78.75:20808` | same (hardcoded in Link itself — cannot drift) |
| Link quantum | `main.h` `#define LINK_QUANTUM 16.0` | `link_bridge.cpp` `quantum = 16.0` |
| Start/stop sync | `main.cpp` `g_link->enableStartStopSync(true)` | `link_bridge.cpp` `enableStartStopSync(true)` |
| Host election | lowest MAC/BSSID wins | lowest MAC/BSSID wins |

`PHRASE_BEATS 64.0` is NOT the Link quantum — it is this project's own
transport-correction/SPP boundary (16 bars). It intentionally differs from
`LINK_QUANTUM 16.0` and does not affect phase agreement with aloop.

### Host election must stay MAC-ordered on both sides

Two devices cold-booting together can each scan before the other's AP
exists, so a naive "nothing found -> host" makes BOTH host, producing two
isolated L2 domains Link can never cross. This project holds for a
duration strictly monotonic in its own STA MAC (`HOLD_MAX_MS = 6000`,
lowest MAC ≈ 0ms), rescanning every second and joining the instant a peer's
AP appears. `../aloop/src/net/autoap.sh` now implements the identical
scheme (same 0–6s bound, same lowest-wins convention) so a Pi and an ESP32
elect one host between them. Both supervisors also yield if another
`ticker` AP with a strictly-lower BSSID appears — but never while clients
are attached, since that would drop peers mid-session. If this convention
is ever inverted here (highest-wins), it MUST be inverted in aloop in the
same change.

### The SoftAP multicast gap is this project's, and may not be aloop's

The ESP32 SoftAP does not carry Link's multicast between host and stations,
which is why `link_multicast_relay_task` exists: it re-emits each Link
datagram to the group, to the AP's own IP, and unicast to every associated
station, preserving the original source IP (Link needs the true source for
direct peer connect). The equivalent gap on aloop's Broadcom `brcmfmac` AP
mode is UNVERIFIED — `ap_isolate=0` may be sufficient there. Do not assume
aloop needs this relay ported; see `../aloop/docs/LINK-MESH-TESTING.md`.
