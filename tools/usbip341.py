import socket, struct, sys, time

HOST, PORT = "127.0.0.1", 3240
BUSID = "2-2"
REQ = 0xA4
BIT_RTS, BIT_DTR = 1 << 6, 1 << 5


def connect():
    s = socket.create_connection((HOST, PORT), timeout=8)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return s


def recv(s, n):
    buf = b""
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            raise EOFError("server closed after %d/%d bytes" % (len(buf), n))
        buf += chunk
    return buf


def devlist(s):
    s.sendall(struct.pack(">HHI", 0x0111, 0x8005, 0))
    ver, cmd, status = struct.unpack(">HHI", recv(s, 8))
    n = struct.unpack(">I", recv(s, 4))[0]
    out = []
    for _ in range(n):
        d = recv(s, 312)
        busid = d[256:288].rstrip(b"\0").decode()
        busnum, devnum = struct.unpack(">II", d[288:296])
        vid, pid = struct.unpack(">HH", d[300:304])
        nif = d[311]
        recv(s, 4 * nif)
        out.append((busid, vid, pid, busnum, devnum, nif))
    return out


def import_dev(s, busid):
    s.sendall(struct.pack(">HHI", 0x0111, 0x8003, 0) + busid.encode().ljust(32, b"\0"))
    ver, cmd, status = struct.unpack(">HHI", recv(s, 8))
    if status:
        raise OSError("OP_REP_IMPORT status=%d" % status)
    d = recv(s, 312)
    busnum, devnum = struct.unpack(">II", d[288:296])
    return (busnum << 16) | devnum


def control_out(s, devid, seq, wvalue, windex=0, request=REQ, bmrt=0x40):
    setup = struct.pack("<BBHHH", bmrt, request, wvalue, windex, 0)
    hdr = struct.pack(">IIIIIIIIII", 0x00000001, seq, devid, 0, 0, 0, 0, 0, 0, 0)
    s.sendall(hdr + setup)
    cmd, rq, did, direc, ep, status, actual, sf, nop, errc = struct.unpack(
        ">IIIIIIIIII", recv(s, 48)[:40])
    return status


def handshake(s, devid, seq, control, inverted=True):
    return control_out(s, devid, seq, (~control & 0xFF) if inverted else control)


def main():
    s = connect()
    devs = devlist(s)
    print("exported devices:")
    for busid, vid, pid, bn, dn, nif in devs:
        print("  %-6s %04x:%04x bus %d dev %d ifs %d" % (busid, vid, pid, bn, dn, nif))
    s.close()

    s = connect()
    devid = import_dev(s, BUSID)
    print("imported %s devid=0x%08x" % (BUSID, devid))
    seq = 1
    for control in (BIT_RTS, 0, BIT_DTR | BIT_RTS, BIT_DTR, 0):
        st = handshake(s, devid, seq, control)
        print("  handshake control=0x%02x -> urb status=%d" % (control, st), flush=True)
        seq += 1
        time.sleep(0.4)
    s.close()
    print("connection closed (device should return to Windows)")


if __name__ == "__main__":
    main()
