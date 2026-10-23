# Flashing the ticker from the Windows box (WSL2 + CH340 over USB/IP)

Extracted from AGENTS.md 2026-10-23. On Linux this is unnecessary: plain esptool
`--before default-reset` reaches the bootloader.

### Hands-free flash on the Windows box: `python tools/flash-usbip.py`

`flash-ticker.js` needs BOOT/IO0 held — a WCH/CH340 driver limit, not the board; `flash-usbip.py` talks CH341
over USB/IP (usbipd-win) behind a pyserial shim, so the WCH driver never loads. Measured: exit 0, ~49 s, app
~19.8 s @460800, "Hash of data verified", `boot:0x13`, no BOOT hold, no WSL.
1. **The D1 R32's auto-reset is DIFFERENTIAL.** EN low only at (DTR# high, RTS# low) `0x40`; IO0 only at
   (DTR# low, RTS# high) `0x20`; `0x60`/`0x00` conduct NEITHER — opposite of esptool's convention, hence
   `boot:0x13` on every esptool reset there. Entry is the flip **0x40 -> 0x20** (`boot:0x3`). The CH341
   handshake register is active-low: **bit6 = RTS#/EN, bit5 = DTR#/IO0** (inverted once, `9ddd92f`).
   On Linux the plain `default-reset` reaches the bootloader, so this only applies to that Windows path.
2. **The CH341 withholds the first replies**: a sync frame goes unanswered for seconds, then surfaces behind a
   later FULL-SIZE OUT; prime with repeated framed syncs (`SYNC_TIMEOUT` 2 s).
3. **The first attach back often fails**: usbipd keeps the device claimed, so `OP_REP_IMPORT` returns
   `status=4`; `open_port()` retries (10 x 6 s).
4. **`USBIPD_BUSID` in `tools/ch341.py` is THAT MACHINE's busid (`2-2`)**, not a board property; re-read per
   host and after any replug; `usbipd bind --busid <busid>` once per boot.
Also: **`--after no-reset-stub`, never `--after no-reset`**; guard `esptool.main()` with `except
BaseException` (`09da1052`) — `no-reset` soft-resets at the flash baud while the ROM speaks 115200 ->
`FatalError: No more data to read` AFTER all three images verified, and `FatalError` isn't `SystemExit`; the
guard is a bounded retry loop (`FLASH_ATTEMPTS = 6`). **`esp.read_mac()`/`flash_id()` are wrapped non-fatal**
(`579d4ef`): a `FatalError` from either aborted **after download-mode entry**, stranding the board at
`boot:0x3` — **nothing between sync and `boot_app()` may raise**. `UsbipPort.timeout` has a **2.0 s floor**
(`_min_timeout`). The ROM speaks only 115200 (`ROM_BAUD`); `--baud` applies only after the RAM stub re-times
it (115200 -> 72 s, 460800 -> 19 s; 921600 dies, PRD `flash-ticker-js-921600-unmeasured`). `UsbipPort()` from
`tools/usbip-port.py` MUST have `.init()` called (else 4096 zero bytes per bulk IN floods the output); import
that hyphenated name via `importlib.util.spec_from_file_location`.

**WSL2 / CH340 host setup (Windows box)**: CH340 is seen by the kernel but no `/dev/ttyUSB*` node appears
until `ch341` loads: `/etc/modprobe.d/wsl-ch340.conf` loads `usbserial` + `ch341`; `sudo modprobe ch341`
before first use. `flash.sh` auto-detects `/dev/ttyUSB*`, needs sudo, runs `python3.12 -m esptool` with a
PYTHONPATH override (system esptool 4.7.0 lacks `stub_flasher_32.json`; the `esptool` binary ignores
PYTHONPATH).

**Build performance**: `build.sh` is incremental by default; `./build.sh clean` for a full rebuild; Docker
layer caching makes code-only changes 10-100x faster. **WSL2 Docker is broken**: `docker compose` build hangs
>30 min, bare `docker run` hangs (3 min timeout) when only sources change — root cause unknown. Workaround:
pre-built binary, iterate over serial.

