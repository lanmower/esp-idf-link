import socket, struct, sys, time

HOST, PORT = "127.0.0.1", 3240
BUSID = "2-2"
REQ_MODEM = 0xA4
BIT_RTS, BIT_DTR = 1 << 5, 1 << 6
ENDPOINT_DESCRIPTOR = 0x05


def connect():
    s = socket.create_connection((HOST, PORT), timeout=8)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return s


def recv(s, n):
    buf = b""
    while len(buf) < n:
        c = s.recv(n - len(buf))
        if not c:
            raise EOFError("closed at %d/%d" % (len(buf), n))
        buf += c
    return buf


def import_dev(s):
    s.sendall(struct.pack(">HHI", 0x0111, 0x8003, 0) + BUSID.encode().ljust(32, b"\0"))
    ver, cmd, status = struct.unpack(">HHI", recv(s, 8))
    if status:
        raise OSError("import status=%d" % status)
    d = recv(s, 312)
    busnum, devnum = struct.unpack(">II", d[288:296])
    return (busnum << 16) | devnum


def submit(s, devid, seq, ep, direction, setup, data=b"", length=None):
    n = len(data) if length is None else length
    hdr = struct.pack(">IIIIIIIIII", 1, seq, devid, direction, ep, 0, n, 0, 0, 0)
    s.sendall(hdr + setup + data)
    r = recv(s, 48)
    cmd, rq, did, direc, rep, status, actual, sf, nop, errc = struct.unpack(
        ">IIIIIIIIII", r[:40])
    payload = recv(s, actual) if actual else b""
    return status, payload


def get_descriptor(s, devid, seq, wvalue, length):
    setup = struct.pack("<BBHHH", 0x80, 6, wvalue, 0, length)
    return submit(s, devid, seq, 0, 1, setup, b"", length)


def hs(s, devid, seq, control):
    setup = struct.pack("<BBHHH", 0x40, REQ_MODEM, ~control & 0xFF, 0, 0)
    return submit(s, devid, seq, 0, 0, setup)[0]


def endpoints(cfg):
    out, i = [], 0
    while i + 1 < len(cfg):
        blen, btype = cfg[i], cfg[i + 1]
        if blen < 2:
            break
        if btype == ENDPOINT_DESCRIPTOR and blen >= 7:
            out.append((cfg[i + 2], cfg[i + 3]))
        i += blen
    return out


def main():
    s = connect()
    devid = import_dev(s)
    print("devid=0x%08x" % devid, flush=True)
    seq = 1
    st, cfg = get_descriptor(s, devid, seq, 0x0200, 255)
    seq += 1
    print("config descriptor status=%d len=%d" % (st, len(cfg)), flush=True)
    eps = endpoints(cfg)
    bulk_in = [a for a, at in eps if (at & 0x03) == 0x02 and (a & 0x80)]
    bulk_out = [a for a, at in eps if (at & 0x03) == 0x02 and not (a & 0x80)]
    print("endpoints: %s" % [(hex(a), hex(at)) for a, at in eps], flush=True)
    print("bulk IN %s  bulk OUT %s" % ([hex(x) for x in bulk_in], [hex(x) for x in bulk_out]),
          flush=True)
    if not bulk_in:
        s.close()
        return
    epin = bulk_in[0]

    def tap(label, sec):
        buf = bytearray()
        t0 = time.time()
        st_all = 0
        while time.time() - t0 < sec:
            st, p = submit(s, devid, seq_next(), epin, 1, b"\0" * 8, b"", 4096)
            if p:
                buf.extend(p)
            else:
                time.sleep(0.05)
        txt = bytes(buf)
        print("  %-26s %6d bytes | %s" % (label, len(txt),
              repr(txt[:110])), flush=True)
        return txt

    counter = [seq]

    def seq_next():
        counter[0] += 1
        return counter[0]

    print("=== idle: is the app's log reaching us at all? ===", flush=True)
    tap("idle 3s", 3.0)

    print("=== hold each line 1.5s, watch the log stop ===", flush=True)
    for control in (0x00, 0x20, 0x40, 0x60):
        hs(s, devid, seq_next(), control)
        tap("control=0x%02x" % control, 2.0)
        hs(s, devid, seq_next(), 0x00)
        tap("  released", 2.0)
    s.close()


if __name__ == "__main__":
    main()
