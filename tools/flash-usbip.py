"""Flash the ticker over USB/IP -- no BOOT button, no esptool line guessing.

Why this exists (measured, not assumed). The board is a Wemos D1 R32
(ESPDuino-32). Its auto-reset circuit is DIFFERENTIAL: each transistor is driven
by the DTR#-RTS# difference, so

    EN  is pulled low only when (DTR# high, RTS# low)  -> handshake byte 0x40
    IO0 is pulled low only when (DTR# low, RTS# high)  -> handshake byte 0x20
    both asserted (0x60) or both clear (0x00) -> NEITHER transistor conducts

That is the opposite of the convention esptool hard-codes, which is why every
esptool reset sequence over 75+ trials produced boot:0x13. The entry is the
two-state flip 0x40 -> 0x20: EN is released (its RC rises) while IO0 is already
low. Verified: boot:0x3 (DOWNLOAD_BOOT) and SYNC OK mac=E4:65:B8:77:0D:14.

Everything -- the EN edge, the IO0 level and the UART -- goes over one USB/IP
connection to usbipd, so the WCH driver is never involved and Windows cannot
re-assert a line. Requires `usbipd bind --busid 2-2` once (already shared).

    python tools/flash-usbip.py --dry      # enter download mode, prove sync, boot back
    python tools/flash-usbip.py            # flash the committed build images
"""
import argparse, importlib.util, os, re, sys, time

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("usbip_port",
                                               os.path.join(_HERE, "usbip-port.py"))
_m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_m)
UsbipPort = _m.UsbipPort

ROOT = os.path.dirname(_HERE)
import struct
SYNC_BODY = (struct.pack("<BBHI", 0x00, 0x08, 36, 0)
             + b"\x07\x07\x12\x20" + b"\x55" * 32)   # esptool framing: hdr + payload
HOLD_EN, HOLD_IO0 = 0x40, 0x20          # EN low / IO0 low, as 0xA4 control bits
OFF_BOOTLOADER, OFF_PARTITIONS = "0x1000", "0x8000"


def ch341_divisor(baud):
    """Word for the 0x9A divisor request -- mirrors ch341_set_baudrate() in Linux."""
    factor, div = 1532620800 // baud, 3
    while factor > 0xFFF0 and div:
        factor >>= 3
        div -= 1
    if factor > 0xFFF0:
        return None
    return ((0x10000 - factor) & 0xFF00) | 0x80 | div


class Port(UsbipPort):
    """Adds the one thing esptool needs and ch341.py lacked: a baud setter.

    esptool re-times the port after sync; without this the CH341 would stay at
    115200 while the chip moved, and every later frame would be garbage.
    """

    def __init__(self, timeout=3.0):
        self._ready = False            # before super(): its __init__ sets .baudrate
        self._baud = 115200
        UsbipPort.__init__(self, timeout=timeout)

    @property
    def baudrate(self):
        return self._baud

    @baudrate.setter
    def baudrate(self, baud):
        self._baud = baud
        if not self._ready or not baud:
            return
        word = ch341_divisor(baud)
        if word is not None:
            self.ctl(0x40, 0x9A, 0x1312, word)


def app_offset(root=ROOT):
    """First app partition's offset, read from the BUILT table -- never guess.

    partitions_large.csv leaves every offset but nvs's empty (the build computes
    them), so the CSV cannot answer this. The built image can: 32-byte entries,
    little-endian magic 0x50AA, then type, subtype, offset, size, label. App
    entries are type 0.
    """
    bin_ = os.path.join(root, "build", "partition_table", "partition-table.bin")
    if not os.path.exists(bin_):
        raise SystemExit("no built partition table at %s" % bin_)
    d = open(bin_, "rb").read()
    for i in range(0, len(d) - 32, 32):
        e = d[i:i + 32]
        magic, typ = struct.unpack("<HB", e[:3])
        if magic != 0x50AA:
            break
        if typ == 0:
            return "0x%x" % struct.unpack("<I", e[4:8])[0]
    raise SystemExit("no app partition in %s" % bin_)


def open_port(tries=10, gap=6.0):
    for i in range(tries):
        try:
            return Port()
        except OSError as e:
            print("  attach %d/%d: %s" % (i + 1, tries, e), flush=True)
            time.sleep(gap)
    raise SystemExit("no attach")


def enter_download(port, banner=True):
    """0x40 -> 0x20. Returns the boot line."""
    port.hs(HOLD_EN)                    # EN low, IO0 high: chip in reset
    port.drain(0.6)
    port.hs(HOLD_IO0)                   # EN released (RC rises), IO0 low
    txt = port.drain(2.0 if banner else 0.2)
    line = ""
    for l in txt.split("\n"):
        if "boot:" in l:
            line = l.strip()
            break
    return line, txt


def boot_app(port):
    """From download mode: pulse EN with IO0 high -> boot:0x13."""
    port.hs(HOLD_EN)
    port.drain(0.3)
    port.hs(0x00)
    return port.drain(8.0)


def slip(body):
    return (b"\xc0" + body.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc")
            + b"\xc0")


def prime(port, frames=12, wait=0.4):
    """Send sync frames until the ROM's reply actually arrives.

    The CH341 withholds the first replies: a single frame gets no answer for
    seconds, then several past replies come back at once behind the next
    full-size OUT. After the first reply lands, latency is ~20ms and stays
    there, so priming is what makes esptool's one-frame sync (deadline 0.1s)
    viable at all. Returns the number of frames it took, 0 if never.
    """
    for i in range(frames):
        port.reset_input_buffer()
        port.write(slip(SYNC_BODY))
        if port.drain(wait):
            port.reset_input_buffer()    # leave no stale reply for the next command
            return i + 1
    return 0


def enter_and_sync(port, baud, attempts=8):
    """Enter download mode, prime the pipe, sync."""
    last = ""
    import esptool.loader
    esptool.loader.SYNC_TIMEOUT = 2.0     # 0.1s is under one primed round trip
    from esptool.targets.esp32 import ESP32ROM
    for i in range(attempts):
        line, txt = enter_download(port, banner=(i == 0))
        if i == 0:
            print("  %s" % (line or "no boot line"), flush=True)
            if any("waiting for download" in l for l in txt.split("\n")):
                print("  ROM: waiting for download", flush=True)
        n = prime(port)
        esp = ESP32ROM(port, baud)
        try:
            esp.sync()
            print("  sync ok (primed in %d frame%s)" % (n, "" if n == 1 else "s"),
                  flush=True)
            return esp
        except Exception as e:
            last = str(e).splitlines()[0][:60]
            print("  attempt %d/%d: primed=%d %s" % (i + 1, attempts, n, last), flush=True)
    raise SystemExit("could not sync after %d attempts (%s)" % (attempts, last))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true",
                    help="prove download-mode entry and sync, then boot the app")
    ap.add_argument("--baud", default="115200")
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--no-stub", action="store_true", help="skip the RAM stub upload")
    args = ap.parse_args()

    port = open_port()
    port.init()
    port._ready = True
    port.baudrate = int(args.baud)
    print("devid=0x%08x  baud=%d" % (port.devid, port.baudrate), flush=True)

    print("=== enter download mode: 0x40 -> 0x20 ===", flush=True)
    esp = enter_and_sync(port, int(args.baud))
    print("  sync ok  chip=esp32  mac=%s" %
          ":".join("%02X" % b for b in esp.read_mac()), flush=True)
    print("  flash id=0x%06x" % esp.flash_id(), flush=True)

    if args.dry:
        print("=== dry run: booting back into the app ===", flush=True)
        txt = boot_app(port)
        for l in txt.split("\n"):
            if "boot:" in l or "MAIN:" in l or "WIFI:" in l:
                print("  | %s" % l.strip()[:76], flush=True)
        port.close()
        return 0

    def image(*parts):
        return os.path.join(ROOT, "build", *parts)

    images = [(OFF_BOOTLOADER, image("bootloader", "bootloader.bin")),
              (OFF_PARTITIONS, image("partition_table", "partition-table.bin")),
              (app_offset(), image("link-idf-example.bin"))]
    missing = [p for _off, p in images if not os.path.exists(p)]
    if missing:
        raise SystemExit("missing image(s): %s" % ", ".join(missing))

    argv = ["--before", "no-reset", "--after", "no-reset", "--baud", args.baud,
            "write-flash"]                       # esptool 5.x spells it hyphenated
    if args.verify:
        argv.append("--verify")
    if args.no_stub:
        argv.append("--no-stub")                 # ROM only: slow, but no stub upload
    for off, path in images:
        argv += [off, path]
    print("=== flashing: %s ===" % " ".join(argv[6:]), flush=True)
    import esptool
    try:
        esptool.main(argv, esp=esp)
    except SystemExit as e:
        print("esptool exited %s" % e, flush=True)

    print("=== booting the new firmware (EN pulse, IO0 high) ===", flush=True)
    txt = boot_app(port)
    for l in txt.split("\n"):
        if "boot:" in l or "MAIN:" in l or "WIFI:" in l:
            print("  | %s" % l.strip()[:76], flush=True)
    port.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
