"""Is the bulk-OUT write path actually reaching the ESP32?

flash-usbip.py gets "waiting for download" (so entry works) but the ROM never
answers a sync. usbip-enter.py trial B synced fine with the same write path, so
the difference must be isolated: run the exact trial-B path, print the OUT URB
status (silent on error elsewhere), then write the SLIP sync frame by hand and
show whatever comes back.
"""
import importlib.util, os, socket, struct, sys, time

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("usbip_port",
                                               os.path.join(_HERE, "usbip-port.py"))
_m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_m)
UsbipPort = _m.UsbipPort

import struct
SYNC = (b"\xc0" + struct.pack("<BBHI", 0x00, 0x08, 36, 0)
        + b"\x07\x07\x12\x20" + b"\x55" * 32 + b"\xc0")   # esptool framing: hdr+payload


def open_port(tries=10, gap=6.0):
    for i in range(tries):
        try:
            return UsbipPort()
        except OSError as e:
            print("  attach %d/%d: %s" % (i + 1, tries, e), flush=True)
            time.sleep(gap)
    raise SystemExit("no attach")


def out(port, data, label):
    """One OUT URB, status printed -- _bulk_out swallows it."""
    port.s.sendall(struct.pack(">IIIIIIIIII", 1, port.seq, port.devid,
                               0, 0x02, 0, len(data), 0, 0, 0)
                   + b"\0" * 8 + data)
    port.seq += 1
    hdr = _m.recvn(port.s, 48)
    f = struct.unpack(">IIIIIIIIII", hdr[:40])
    print("  OUT %-12s len=%-3d status=%d actual=%d" % (label, len(data), f[5], f[6]),
          flush=True)   # RET_SUBMIT carries no payload for an OUT


def pump(port, sec):
    """One IN URB with a short socket timeout -- returns True if bytes landed."""
    try:
        port.s.sendall(struct.pack(">IIIIIIIIII", 1, port.seq, port.devid,
                                   1, 0x82, 0, 4096, 0, 0, 0) + b"\0" * 8)
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
    port.drain(6.0)

    print("=== enter download: 0x40 -> 0x20 ===", flush=True)
    port.hs(0x40)
    port.drain(0.6)
    port.hs(0x20)
    txt = port.drain(6.0)
    print("  %s" % port.verdict(txt), flush=True)

    print("=== one entry, then a sync frame every 0.5s: when does the ROM answer? ===",
          flush=True)
    t0 = time.time()
    hit = -1
    for i in range(4):
        port.rxbuf.clear()
        tw = time.time()
        out(port, SYNC, "frame%d" % i)
        nothing = True
        for k in range(8):                       # 8 x 0.25s waiting for the reply
            if pump(port, 0.25):
                print("     frame%d: reply after %.2fs %s"
                      % (i, time.time() - tw, repr(port.rxbuf[:20])), flush=True)
                nothing = False
                break
        if nothing:
            print("     frame%d: nothing for 2s -- kicking with a bare 0xC0" % i,
                  flush=True)
            out(port, b"\xc0", "kick")           # empty SLIP frame: ignored by both ends
            for k in range(8):
                if pump(port, 0.25):
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
    port.hs(0x00)
    port.close()


if __name__ == "__main__":
    main()
