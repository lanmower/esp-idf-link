import argparse, importlib.util, os, re, sys, time

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("usbip_port",
                                               os.path.join(_HERE, "usbip-port.py"))
_m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_m)
UsbipPort = _m.UsbipPort

ROOT = os.path.dirname(_HERE)
import struct
from ch341 import (REG_DIVISOR, REG_PRESCALER, REQ_WRITE_REG, VENDOR_OUT)

SLIP_REQUEST, CMD_SYNC = 0x00, 0x08
SYNC_PAYLOAD = b"\x07\x07\x12\x20" + b"\x55" * 32
SYNC_BODY = (struct.pack("<BBHI", SLIP_REQUEST, CMD_SYNC, len(SYNC_PAYLOAD), 0)
             + SYNC_PAYLOAD)
HOLD_EN, HOLD_IO0 = 0x40, 0x20
OFF_BOOTLOADER, OFF_PARTITIONS = "0x1000", "0x8000"
ROM_BAUD = 115200
PARTITION_MAGIC, PARTITION_ENTRY_SIZE, PARTITION_TYPE_APP = 0x50AA, 32, 0


def ch341_divisor(baud):
    factor, div = 1532620800 // baud, 3
    while factor > 0xFFF0 and div:
        factor >>= 3
        div -= 1
    if factor > 0xFFF0:
        return None
    return ((0x10000 - factor) & 0xFF00) | 0x80 | div


class Port(UsbipPort):
    _ready = False
    _baud = ROM_BAUD

    def __init__(self, timeout=3.0):
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
            self.ctl(VENDOR_OUT, REQ_WRITE_REG, (REG_DIVISOR << 8) | REG_PRESCALER,
                     word)


def app_offset(root=ROOT):
    bin_ = os.path.join(root, "build", "partition_table", "partition-table.bin")
    if not os.path.exists(bin_):
        raise SystemExit("no built partition table at %s" % bin_)
    d = open(bin_, "rb").read()
    for i in range(0, len(d) - PARTITION_ENTRY_SIZE, PARTITION_ENTRY_SIZE):
        e = d[i:i + PARTITION_ENTRY_SIZE]
        magic, typ = struct.unpack("<HB", e[:3])
        if magic != PARTITION_MAGIC:
            break
        if typ == PARTITION_TYPE_APP:
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
    port.hs(HOLD_EN)
    port.drain(0.6)
    port.hs(HOLD_IO0)
    txt = port.drain(2.0 if banner else 0.2)
    line = ""
    for l in txt.split("\n"):
        if "boot:" in l:
            line = l.strip()
            break
    return line, txt


def boot_app(port):
    port.baudrate = ROM_BAUD
    port.hs(HOLD_EN)
    port.drain(0.3)
    port.hs(0x00)
    return port.drain(8.0)


def slip(body):
    return (b"\xc0" + body.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc")
            + b"\xc0")


def prime(port, frames=12, wait=0.4):
    for i in range(frames):
        port.reset_input_buffer()
        port.write(slip(SYNC_BODY))
        if port.drain(wait):
            port.reset_input_buffer()
            return i + 1
    return 0


def enter_and_sync(port, baud, attempts=8):
    last = ""
    import esptool.loader
    esptool.loader.SYNC_TIMEOUT = 2.0
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
    ap.add_argument("--baud", default="460800",
                    help="flash baud (measured: 115200=72s, 460800=20s, 921600 fails); "
                         "the ROM sync below always runs at 115200")
    ap.add_argument("--no-stub", action="store_true", help="skip the RAM stub upload")
    args = ap.parse_args()

    port = open_port()
    port.init()
    port._ready = True
    print("devid=0x%08x  rom baud=%d" % (port.devid, port.baudrate), flush=True)

    print("=== enter download mode: 0x40 -> 0x20 ===", flush=True)
    esp = enter_and_sync(port, ROM_BAUD)
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

    argv = ["--before", "no-reset", "--after", "no-reset", "--baud", args.baud]
    if args.no_stub:
        argv.append("--no-stub")
    argv.append("write-flash")
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
