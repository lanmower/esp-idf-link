# Flag-on netif role switch: plan, pre-registration and witness design

Row: `mesh-netif-no-teardown`. Revision 2, answering mc-420's review of revision 1 (`a005435`).
Status: **planned, not scheduled.** The board stays on `2a5162f` (certified: failover 2026-10-25) until
this revision is agreed with mc-420 and committed. This file is the pre-registration: the criteria are
fixed before any run, and the commit that adds it is the record.

## 1. The change

`WIFI_NO_TEARDOWN_ROLE_SWITCH` (`main/wifi_config.h:12`) is `0` on `main`, so the running switch is the
certified destroy-based one.

The candidate commit makes exactly two kinds of change, both inside the flag:

1. The flag `0` -> `1`.
2. **Log-only lines inside `#if WIFI_NO_TEARDOWN_ROLE_SWITCH` blocks**, which are the witnesses for H2 and
   H3 (section 4). No control flow changes. The flag-off image gains no code, because every new line sits
   inside the flag's `#if`.

With the flag on, the switch never calls `esp_netif_destroy*` or `esp_wifi_stop`:

- Both netifs are created once in `wifi_config_init` (`main/wifi_config.cpp:225-240`), and the mode is set to
  `WIFI_MODE_APSTA` once.
- `role_enforce` (`:213-220`) parks the out-of-role netif (`role_park_netif`, `:183`) and activates the
  in-role one (`role_activate_netif`, `:193`).

## 2. Hypotheses

- **H1 (convergence).** A 100 s host-AP outage with the flag on re-hosts, yields and rejoins, with no panic
  and no deferral, as the flag-off image does.
- **H2 (the AP beacons on a running APSTA).** `wifi_start_link_ap` (`:366`) reconfigures the AP with
  `esp_wifi_set_config(WIFI_IF_AP, ...)` (`:395`) while Wi-Fi runs. AGENTS.md lists this as unverified.
- **H3 (the parked netif is silent).** While the ESP is a station, the AP netif is parked: admin down, no IP,
  no DHCP server. While it is an AP, the STA netif is parked likewise.
- **H4 (no send failures after rejoin).** After rejoin, Link sends do not fail. The send counters do not
  grow after the IP line.

## 3. Procedure

1. Commit the candidate: flag `1` plus the log lines of section 4. CI builds it. Record the CI run id and the
   sha256 of the committed binary.
2. Run the reachability check at the candidate commit and record its output (section 5, P9):
   `python3 tools/flag_reachability.py main/wifi_config.cpp`. It must print `RESULT: no destroy or wifi-stop
   call is live with the flag on`.
3. Flash with the MAC check: `esptool` must print `e4:65:b8:77:0d:14` before anything is written.
4. One console open, held for the whole run. Record the compile stamp and the boot banner.
5. **P0 soak, 30 min.** No Pi scans during it (the Pi has one radio; a scan drops hostapd and invalidates the
   soak). The telemetry sampler must already meet P6 (section 6).
6. Only if P0 passes: one 100 s outage, detached on the Pi with the TERM-timeout trap, fired at a second
   mc-420 names and I acknowledge. The Pi scan runs only during the outage window (section 4, H2).
7. Evaluate P1 to P10. Record every number.
8. A single clean outage makes the flag-on image a **candidate only**. Accepting it as the default needs a
   second outage on a fresh boot, at least an hour later, and a 3 h soak. Each is a separate decision.

## 4. Witnesses for H2 and H3 (ESP-side, log-only, inside the flag)

Both are ESP-side because the Pi has one radio: a Pi scan while the ESP hosts drops the Pi's own AP, so
no Pi scan may run outside the outage window.

- **H3 authority:** a line printed at the end of every `role_enforce`:
  `role: in=<STA|AP> sta_up=<0|1> sta_ip=<a.b.c.d> ap_up=<0|1> ap_ip=<a.b.c.d>`,
  read with `esp_netif_is_netif_up` and `esp_netif_get_ip_info` for both netifs. H3 holds if every line with
  `in=STA` has `ap_up=0 ap_ip=0.0.0.0`, and every line with `in=AP` has `sta_up=0 sta_ip=0.0.0.0`.
- **H2 authority:** a line printed after `esp_wifi_set_config(WIFI_IF_AP, ...)` at `:395`:
  `ap cfg: ret=<esp_err> ssid=<ssid> ch=<channel> ap_mac=<mac>`, using `esp_wifi_get_config(WIFI_IF_AP)` and
  `esp_wifi_get_mac(WIFI_IF_AP)`. H2 holds if `ret=0`, `ssid=ticker`, and `ch=6`, and `WIFI_EVENT_AP_START`
  (`AP started`) follows within 5 s.
- **H2 external check, outage only:** a Pi-side scan every 10 s from `STOP` to `START`, which is the only
  window in which the Pi's radio is free. H2 holds if the scan lists `ticker` at the `ap_mac` from the ESP
  log line.

## 5. Pass criteria (pre-registered numbers)

A run passes only if every line holds. A failure is recorded as a failure, and nothing is retuned in the
same window.

| # | Criterion | Witness |
|---|---|---|
| P0 | Steady state, 30 min: peer retention ≥ 95%, zero drops, bpm spread ≤ 0.05, max \|phase_err\| ≤ 0.03, eff spread ≤ 0.005, sample gaps ≤ 10%, zero `peerloss`, zero panic | 1 Hz telemetry (P6 sampler), ESP console |
| P1 | No `Guru Meditation`, no `SW_CPU_RESET`, no `relay did not stop`, no `deferring` | ESP console |
| P2 | Re-host logged: `STA down past 30 tries`, then `AP started`, then DHCP on 192.168.4.1 | ESP console |
| P3 | Yield logged: `Lost dual-host tie-break`, then `connected with ticker`, then `IP:` within 3 s | ESP console |
| P4 | `Link peer back` within **10 s of the `IP:` line** | ESP console |
| P5 | Pi after-state at AFTER+60: `hostapd=1 ssid=ticker type=AP stations=1` | Pi `outage.out` |
| P6 | Sampler: replies ≥ 0.9 × expected over the window, misses ≤ 10% at a **2 s** receive timeout, sends on a timer (no wait for the reply before the next send). See section 6 | sampler log |
| P7 (H2) | `ap cfg: ret=0 ssid=ticker ch=6`, `AP started` within 5 s, and the outage-only Pi scan lists `ticker` at the logged `ap_mac` | ESP console, Pi scan |
| P8 (H3) | Every `role:` line obeys the H3 rule in section 4. No Pi scan is used | ESP console |
| P9 (C) | `tools/flag_reachability.py` prints no live destroy or wifi-stop call with the flag on, and `callers` agrees (section 7) | repo, at the candidate commit |
| P10 (B) | `sendfail` lines **outside** `[STOP, IP+3 s]` = 0. **Inside** the window ≤ 20 | ESP console |
| P11 (F, H4) | Over the window from `IP+3 s` to the end of the capture, the `fail=` and `giveup=` fields of `Link send:` lines do not change (delta 0). `errno=` is not required to be 0 but must not change | ESP console, `Link send:` line |

Definitions for P10 and P4. `STOP` is the Pi's `STOP` line (10:45:00Z for the certified run). `IP` is the
ESP's `IP:` line after the yield (10:46:47Z). The window is `[10:45:00Z, 10:46:50Z]`. The certified run had
10 `sendfail` lines, all inside the window and none outside (P10 holds), and P4 measured 3.7 s (10:46:47 to
10:46:50.7).

## 6. Telemetry sampler (P6), measured

The sampler must not wait for a reply before the next send, and it must use a receive timeout long enough for
a real round trip. Requirement: replies ≥ 0.9 × expected over the window, misses ≤ 10% at a **2 s** timeout.

Measured so far:
- This host, node sampler, eth0 to the Pi: 91 replies in 90 s, **1.000 Hz**, 0 gaps over 1.5 s, max interval
  1.00 s. This is a node process on the ICS host, not the Pi-side poller the soak uses.
- Pi-side static poller (mc-420), 500 ms ceiling, send then wait: 17 replies in 20 s, **0.85 Hz**, 3 misses.
  A 500 ms ceiling is close to the observed round trip, so these misses are not evidence of lost samples.
  Revalidate with a 2 s timeout and a timer-driven send, over at least 5 min, and post the miss count and
  the RTT spread. P0 does not start until that is posted and meets P6.

## 7. Reachability (P9, C), and what it proves

`callers` at the commit this revision is based on (`codeinsight` index complete):
- `esp_netif_destroy_default_wifi`: callers `wifi_config.cpp:1263` and `:1332`.
- `esp_wifi_stop`: callers `wifi_config.cpp:1255` and `:1319`.
- `esp_netif_destroy`: no caller in our code. Its only callers are in a managed component's test.

`tools/flag_reachability.py`, run on this revision with the flag at `0` and at `1`: all four calls are live
with the flag off and **not live with the flag on**. The check follows preprocessor conditionals, so the
result holds for the flag's `#if` branches, including the chain through `role_enforce`, `role_park_netif`,
`ensure_sta_started`, the re-host branch (`:1244-1270`) and the yield branch (`:1302-1340`).

The candidate commit must rerun the checker and record its output. A call found live with the flag on fails P9.

## 8. Rollback

- **Image:** the certified `2a5162f` binary is the committed blob at `227125e`, which is CI's binary commit for
  source `2a5162f`:
  - `git show 227125e:build/link-idf-example.bin`
  - size 1313456 bytes
  - **sha256 `b42b05b03a4018ad7a01833aa550c49118d61bc71f429693d89f06ece7b48b8f`**
  - Confirmed: the flash at 08:49:46Z to 08:50:11Z wrote this sha (it was printed before the flash). The
    working-tree copy has since changed (`ddea0177…`) because later CI binary commits replaced it, so the
    rollback must read from the blob at `227125e`, never from the working tree.
  - Flash record: MAC `e4:65:b8:77:0d:14`, 0x20000, bootloader and partition table unchanged.
- **Trigger:** any failure of P0 or P1 ends the run. The rollback is flashed by the person who ran it, with the
  MAC check, and recorded.
- **Retune:** never in the same window. A changed image is a new row with a new pre-registration.

## 9. Known risks

- H2 (a running APSTA AP reconfigured by `esp_wifi_set_config`) is the main unknown. If it fails, P7 fails and
  the AP does not beacon after the re-host.
- At the yield, `ensure_sta_started()` (`:1317`, `:1336`) follows `g_ap_active = false`. If a relay send is
  still in flight when `role_enforce(g_sta_netif)` parks the AP, it can reach the parked netif. The defer at
  `:1311` guards the stop, not the order. P10 and P11 would show it.
- `esp_netif_set_default_netif` is set on `IP_EVENT_STA_GOT_IP` (`:129-134`) and in `role_activate_netif`. A
  down event in between could leave the default on a parked netif. P4 and P11 would show it.
- Per-tick parking (`:1231`, `:1273`) is new load on every 2 s tick. P0 measures it.
- The mc-420 sampler (P6) is the only telemetry that gates P0. If it is not validated, P0 does not start.
