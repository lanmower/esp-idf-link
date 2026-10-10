"""A CH341 serial port over raw USB/IP: no Windows driver, no WSL, no elevation.

Init follows drivers/usb/serial/ch341.c exactly -- 0x5F version, 0xA1 serial
init, 0x9A divisor/lcr, then 0xA4 for the modem lines. The divisor word goes in
wIndex (0xCC83 = 115200 with BIT7 set, since version 0x31 > 0x27), not wValue --
every earlier tap put it in the wrong field and read garbage.

This makes the lines observable for the first time: assert the line wired to EN
and the app's log must stop dead; release it and the ROM banner must appear.
"""
import socket, struct, time, sys

HOST, PORT, BUSID = "127.0.0.1", 3240, "2-2"
BIT_DTR, BIT_RTS = 0x20, 0x40
DIV115200, LCR = 0xCC83, 0x00C3   # (0x100-52)<<8 | 0<<2 | 3 | BIT7,  RX|TX|CS8

def recvn(s, n):
    b = b""
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c: raise EOFError("closed at %d/%d" % (len(b), n))
        b += c
    return b

class Ch341:
    def __init__(self):
        self.s = socket.create_connection((HOST, PORT), timeout=8)
        self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.s.sendall(struct.pack(">HHI", 0x0111, 0x8003, 0) + BUSID.encode().ljust(32, b"\0"))
        v, c, st = struct.unpack(">HHI", recvn(self.s, 8))
        if st: raise OSError("OP_REP_IMPORT status=%d" % st)
        d = recvn(self.s, 312)
        bn, dn = struct.unpack(">II", d[288:296])
        self.devid = (bn << 16) | dn
        self.seq = 1

    def ctl(self, bmrt, req, wval, widx=0, rlen=0):
        setup = struct.pack("<BBHHH", bmrt, req, wval, widx, rlen)
        self.s.sendall(struct.pack(">IIIIIIIIII", 1, self.seq, self.devid, 0, 0, 0, 0, 0, 0, 0)
                       + setup)
        self.seq += 1
        f = struct.unpack(">IIIIIIIIII", recvn(self.s, 48)[:40])
        return f[5], (recvn(self.s, f[6]) if f[6] else b"")

    def init(self, div=DIV115200):
        self.ctl(0x00, 0x09, 1, 0)                 # SET_CONFIGURATION: bulk IN is
        ver = self.ctl(0xC0, 0x5F, 0, 0, 2)[1]     # dead until this lands
        self.ctl(0x40, 0xA1, 0, 0)                 # SERIAL_INIT
        self.ctl(0x40, 0x9A, 0x1312, div)          # divisor/prescaler/factor
        self.ctl(0x40, 0x9A, 0x2518, LCR)          # LCR2/LCR
        return ver

    def hs(self, control, hi=0xFF):
        return self.ctl(0x40, 0xA4, (hi << 8) | (~control & 0xFF), 0)[0]

    def read(self, sec=3.0, blk=4096, idle_break=0.4):
        buf = bytearray(); t0 = time.time(); last = time.time()
        while time.time() - t0 < sec:
            self.s.settimeout(1.0)
            self.s.sendall(struct.pack(">IIIIIIIIII", 1, self.seq, self.devid,
                                       1, 0x82, 0, blk, 0, 0, 0) + b"\0" * 8)
            self.seq += 1
            try:
                f = struct.unpack(">IIIIIIIIII", recvn(self.s, 48)[:40])
            except (socket.timeout, EOFError):
                break
            p = recvn(self.s, f[6]) if f[6] else b""
            if p:
                buf.extend(p); last = time.time()
            elif buf and time.time() - last > idle_break:
                break
        return bytes(buf)

    def close(self):
        try: self.s.close()
        except Exception: pass

def show(tag, txt, n=120):
    t = txt.decode("latin1").replace("\r", "")
    print("  %-28s %6dB | %s" % (tag, len(txt), repr(t[:n])), flush=True)
    return t

if __name__ == "__main__":
    d = Ch341(); print("devid=0x%08x" % d.devid, flush=True)
    print("version:", d.init().hex(), flush=True)
    print("=== 1. idle: is the app's log legible? (proves baud + bulk) ===", flush=True)
    show("idle 4s", d.read(4.0))
    print("=== 2. assert RTS (EN low): the log must stop dead ===", flush=True)
    d.hs(BIT_RTS); show("RTS held 3s", d.read(3.0))
    print("=== 3. release RTS: ROM banner must appear ===", flush=True)
    d.hs(0); show("released 4s", d.read(4.0))
    print("=== 4. IO0 low across the EN edge ===", flush=True)
    d.hs(BIT_RTS | BIT_DTR); time.sleep(0.3)
    d.hs(BIT_DTR); time.sleep(0.5)
    d.hs(0)
    show("after boot edge 5s", d.read(5.0))
    d.close()
