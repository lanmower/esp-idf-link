import serial, time, sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200

SYNC_DATA = b"\x07\x07\x12\x20" + b"\x55" * 32
HDR = (bytes([0x00, 0x08]) + len(SYNC_DATA).to_bytes(2, "little")
       + (sum(SYNC_DATA) & 0xFF).to_bytes(4, "little"))
FRAME = b"\xc0" + HDR + SYNC_DATA + b"\xc0"


def slip_decode(buf):
    out = bytearray()
    i = 0
    while i < len(buf):
        b = buf[i]
        if b == 0xDB and i + 1 < len(buf):
            out.append(buf[i + 1] ^ 0x20)
            i += 2
        elif b == 0xC0:
            if out:
                yield bytes(out)
            out = bytearray()
            i += 1
        else:
            out.append(b)
            i += 1
    if out:
        yield bytes(out)


def try_sync(s, attempts=12, per=0.25):
    buf = bytearray()
    for _ in range(attempts):
        s.write(FRAME)
        end = time.time() + per
        while time.time() < end:
            d = s.read(4096)
            if d:
                buf.extend(d)
                for pkt in slip_decode(bytes(buf)):
                    if len(pkt) >= 2 and pkt[0] == 0x01 and pkt[1] == 0x08:
                        return True, bytes(buf)
    return False, bytes(buf)


def fresh(dtr, rts):
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = 0.05
    s.dtr = dtr
    s.rts = rts
    s.open()
    return s


def reopen_seq(name, states):
    s = None
    for dtr, rts, d in states:
        if s:
            s.close()
        s = fresh(dtr, rts)
        time.sleep(d)
    ok, buf = try_sync(s)
    s.close()
    print("%-36s %-9s bytes=%-5d %s" % (
        name, "DOWNLOAD" if ok else "no-sync", len(buf), buf[:40].hex()), flush=True)
    return ok


def handle_seq(name, steps):
    s = serial.Serial(PORT, BAUD, timeout=0.05, dsrdtr=False, rtscts=False)
    time.sleep(0.05)
    for dtr, rts, d in steps:
        s.setDTR(dtr)
        s.setRTS(rts)
        time.sleep(d)
    ok, buf = try_sync(s)
    s.close()
    print("%-36s %-9s bytes=%-5d %s" % (
        name, "DOWNLOAD" if ok else "no-sync", len(buf), buf[:40].hex()), flush=True)
    return ok


T = [
    ("H1 handle: assert both -> rel RTS",  [(True, True, .15), (True, False, 0)]),
    ("H2 handle: assert both -> rel RTS",  [(True, True, .30), (True, False, .05), (False, False, 0)]),
    ("H3 handle: esptool default",         [(False, True, .15), (True, False, .05), (False, False, 0)]),
    ("H4 handle: reset -> rel all (ctl)",  [(False, True, .15), (False, False, 0)]),
    ("S1 reopen: assert both -> rel RTS",  [(True, True, .15), (True, False, 0)]),
    ("S3 reopen: reset -> io0low+rel",     [(False, True, .15), (True, False, 0)]),
]

if __name__ == "__main__":
    handle_seq(*T[0][0:1] + (T[0][1],)) if False else None
    handle_seq(T[0][0], T[0][1])
    handle_seq(T[1][0], T[1][1])
    handle_seq(T[2][0], T[2][1])
    handle_seq(T[3][0], T[3][1])
    reopen_seq(T[4][0], T[4][1])
    reopen_seq(T[5][0], T[5][1])
