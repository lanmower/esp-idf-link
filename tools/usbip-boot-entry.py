import importlib.util, os, sys, time

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("usbip_port",
                                               os.path.join(_HERE, "usbip-port.py"))
_m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_m)
UsbipPort = _m.UsbipPort


sys.path.insert(0, _HERE)
from ch341 import HOLD_EN, HOLD_IO0, RELEASE_BOTH

PRE_TRIAL_SETTLE_SECONDS = 9.0
BOOT_LOG_SECONDS = 6.0
LINE_SETTLE_SECONDS = 0.3


def trial(port, label, steps, hi=0xFF, settle=PRE_TRIAL_SETTLE_SECONDS):
    port.drain(settle)
    for control, hold in steps:
        port.hs(control, hi=hi)
        if hold:
            port.drain(hold)
    txt = port.drain(BOOT_LOG_SECONDS)
    print("  %-30s %-22s %s" % (label, port.verdict(txt), port.sync()), flush=True)
    for line in txt.split("\n"):
        if "boot:" in line or "waiting for download" in line:
            print("      | %s" % line.strip()[:70], flush=True)
            break
    port.hs(RELEASE_BOTH, hi=hi)
    port.drain(LINE_SETTLE_SECONDS)


def open_port(tries=10, gap=6.0):
    for i in range(tries):
        try:
            return UsbipPort()
        except OSError as e:
            print("  attach %d/%d: %s" % (i + 1, tries, e), flush=True)
            time.sleep(gap)
    raise SystemExit("no attach")


def main():
    port = open_port()
    print("devid=0x%08x version=%s" % (port.devid, port.init().hex()), flush=True)

    print("=== control: EN edge with IO0 never low (must stay boot:0x13) ===", flush=True)
    trial(port, "A 0x%02x -> 0x%02x" % (HOLD_EN, RELEASE_BOTH),
          [(HOLD_EN, 0.6), (RELEASE_BOTH, 0.0)])

    print("=== predicted win: EN released while IO0 already low ===", flush=True)
    trial(port, "B 0x%02x -> 0x%02x" % (HOLD_EN, HOLD_IO0),
          [(HOLD_EN, 0.6), (HOLD_IO0, 0.0)])
    trial(port, "C 0x%02x -> 0x%02x -> 0x%02x" % (HOLD_EN, HOLD_IO0, RELEASE_BOTH),
          [(HOLD_EN, 0.6), (HOLD_IO0, 1.0), (RELEASE_BOTH, 0.0)])
    trial(port, "D 0x%02x -> 0x%02x (long hold)" % (HOLD_EN, HOLD_IO0),
          [(HOLD_EN, 1.5), (HOLD_IO0, 0.0)])

    print("=== same, with the wValue high byte cleared (ch341.c sends no hi byte) ===",
          flush=True)
    trial(port, "E hi=0x00 0x%02x -> 0x%02x" % (HOLD_EN, HOLD_IO0),
          [(HOLD_EN, 0.6), (HOLD_IO0, 0.0)], hi=0x00)
    trial(port, "F hi=0x00 0x%02x -> 0x%02x" % (HOLD_EN, RELEASE_BOTH),
          [(HOLD_EN, 0.6), (RELEASE_BOTH, 0.0)], hi=0x00)

    print("=== full cycle, then a second edge ===", flush=True)
    trial(port, "G 0x%02x->0x%02x->0x%02x->0x%02x"
          % (RELEASE_BOTH, HOLD_IO0, HOLD_EN, HOLD_IO0),
          [(RELEASE_BOTH, 0.2), (HOLD_IO0, 0.3), (HOLD_EN, 0.6), (HOLD_IO0, 0.0)])
    port.close()


if __name__ == "__main__":
    main()
