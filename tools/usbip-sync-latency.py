import importlib.util, os, socket, struct, sys, time

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("usbip_port",
                                               os.path.join(_HERE, "usbip-port.py"))
_m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_m)
UsbipPort = _m.UsbipPort
EP_IN, EP_OUT = _m.EP_IN, _m.EP_OUT

sys.path.insert(0, _HERE)
from ch341 import HOLD_EN, HOLD_IO0, RELEASE_BOTH

import struct
ESPTOOL_SYNC_FRAME = (b"\xc0" + struct.pack("<BBHI", 0x00, 0x08, 36, 0)
                      + b"\x07\x07\x12\x20" + b"\x55" * 32 + b"\xc0")

EN_LOW_SECONDS = 0.6
BOOT_LOG_SECONDS = 6.0
BULK_IN_REQUEST_SIZE = 4096
REPLY_POLL_SECONDS = 0.25
REPLY_POLLS_PER_FRAME = 8
SYNC_FRAMES_PER_RUN = 4


def open_port(tries=10, gap=6.0):
    for i in range(tries):
        try:
            return UsbipPort()
        except OSError as e:
            print("  attach %d/%d: %s" % (i + 1, tries, e), flush=True)
            time.sleep(gap)
    raise SystemExit("no attach")


def out_reporting_status(port, data, label):
    port.s.sendall(struct.pack(">IIIIIIIIII", 1, port.seq, port.devid,
                               0, EP_OUT, 0, len(data), 0, 0, 0)
                   + b"\0" * 8 + data)
    port.seq += 1
    hdr = _m.recvn(port.s, 48)
    f = struct.unpack(">IIIIIIIIII", hdr[:40])
    print("  OUT %-12s len=%-3d status=%d actual=%d" % (label, len(data), f[5], f[6]),
          flush=True)


def pump(port, sec):
    try:
        port.s.sendall(struct.pack(">IIIIIIIIII", 1, port.seq, port.devid,
                                   1, EP_IN, 0, BULK_IN_REQUEST_SIZE, 0, 0, 0) + b"\0" * 8)
        port.seq += 1
        port.s.settimeout(sec)
        f = struct.unpack(">IIIIIIIIII", _m.recvn(port.s, 48)[:40])
    except (socket.timeout, TimeoutError, EOFError):
        return False
    p = _m.recvn(port.s, f[6]) if f[6] else b""
    if p:
        port.rxbuf.extend(p)
    return bool(p)


def main():
    port = open_port()
    print("devid=0x%08x version=%s" % (port.devid, port.init().hex()), flush=True)
    port.drain(BOOT_LOG_SECONDS)

    print("=== enter download: 0x%02x -> 0x%02x ===" % (HOLD_EN, HOLD_IO0), flush=True)
    port.hs(HOLD_EN)
    port.drain(EN_LOW_SECONDS)
    port.hs(HOLD_IO0)
    txt = port.drain(BOOT_LOG_SECONDS)
    print("  %s" % port.verdict(txt), flush=True)

    print("=== one entry, then a sync frame every 0.5s: when does the ROM answer? ===",
          flush=True)
    t0 = time.time()
    hit = -1
    for i in range(SYNC_FRAMES_PER_RUN):
        port.rxbuf.clear()
        tw = time.time()
        out_reporting_status(port, ESPTOOL_SYNC_FRAME, "frame%d" % i)
        nothing = True
        for k in range(REPLY_POLLS_PER_FRAME):
            if pump(port, REPLY_POLL_SECONDS):
                print("     frame%d: reply after %.2fs %s"
                      % (i, time.time() - tw, repr(port.rxbuf[:20])), flush=True)
                nothing = False
                break
        if nothing:
            print("     frame%d: nothing for 2s -- kicking with a bare 0xC0" % i,
                  flush=True)
            out_reporting_status(port, b"\xc0", "kick")
            for k in range(REPLY_POLLS_PER_FRAME):
                if pump(port, REPLY_POLL_SECONDS):
                    print("     frame%d: reply %.2fs after the kick %s"
                          % (i, time.time() - tw, repr(port.rxbuf[:20])), flush=True)
                    nothing = False
                    break
        if not nothing:
            hit = i
            break
    print("  first answer at %s" % ("frame %d" % hit if hit >= 0 else "never"), flush=True)

    print("=== esptool sync on the same port ===", flush=True)
    print("  %s" % port.sync(), flush=True)
    port.hs(RELEASE_BOTH)
    port.close()


if __name__ == "__main__":
    main()
