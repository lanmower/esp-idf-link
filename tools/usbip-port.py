import os, socket, struct, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ch341 import (Ch341, recvn, CONTROL_BIT7, RELEASE_BOTH, HOLD_EN, HOLD_IO0)

EP_IN, EP_OUT = 0x82, 0x02
BOTH_HELD = HOLD_EN | HOLD_IO0
BIT7_IO0 = CONTROL_BIT7 | HOLD_IO0
BIT7_BOTH = CONTROL_BIT7 | BOTH_HELD


class UsbipPort(Ch341):
    def __init__(self, timeout=1.0):
        Ch341.__init__(self)
        self.rxbuf = bytearray()
        self.timeout = timeout
        self.write_timeout = None
        self.baudrate = 115200
        self.port = "usbip"
        self.name = "usbip:ch341"
        self.dtr = False
        self.rts = False
        self.is_open = True

    def _bulk_out(self, data, chunk=64):
        for i in range(0, len(data), chunk):
            part = data[i:i + chunk]
            self.s.sendall(struct.pack(">IIIIIIIIII", 1, self.seq, self.devid,
                                       0, EP_OUT, 0, len(part), 0, 0, 0)
                           + b"\0" * 8 + part)
            self.seq += 1
            try:
                recvn(self.s, 48)
            except (socket.timeout, EOFError):
                return
        return len(data)

    def write(self, data):
        return self._bulk_out(bytes(data))

    def _pump(self):
        try:
            self.s.sendall(struct.pack(">IIIIIIIIII", 1, self.seq, self.devid,
                                       1, EP_IN, 0, 4096, 0, 0, 0) + b"\0" * 8)
            self.seq += 1
            self.s.settimeout(max(self.timeout, 0.2))
            f = struct.unpack(">IIIIIIIIII", recvn(self.s, 48)[:40])
        except (socket.timeout, EOFError):
            return False
        p = recvn(self.s, f[6]) if f[6] else b""
        if p:
            self.rxbuf.extend(p)
        return bool(p)

    def read(self, n=1):
        end = time.time() + (self.timeout if self.timeout else 0.2)
        while len(self.rxbuf) < n:
            if time.time() > end:
                break
            self._pump()
        out = bytes(self.rxbuf[:n])
        del self.rxbuf[:n]
        return out

    def inWaiting(self):
        self._pump()
        return len(self.rxbuf)

    @property
    def in_waiting(self):
        return self.inWaiting()

    def flushInput(self):
        self.rxbuf.clear()

    def reset_input_buffer(self):
        self.rxbuf.clear()

    def flushOutput(self):
        pass

    def reset_output_buffer(self):
        pass

    def flush(self):
        pass

    def isOpen(self):
        return self.is_open

    def open(self):
        self.is_open = True

    def close(self):
        self.is_open = False

    def drain(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            self._pump()
        txt = bytes(self.rxbuf).decode("latin1").replace("\r", "")
        self.rxbuf.clear()
        return txt

    def verdict(self, txt):
        for line in txt.split("\n"):
            if "waiting for download" in line:
                return "WAITING-FOR-DOWNLOAD"
            if "boot:" in line:
                return line.strip()[:48]
        return "silent" if not txt else "chatter(%dB)" % len(txt)

    def sync(self, tries=3):
        from esptool.targets.esp32 import ESP32ROM
        last = ""
        for _ in range(tries):
            try:
                self.reset_input_buffer()
                self.timeout = 1.0
                esp = ESP32ROM(self, 115200)
                esp.sync()
                mac = ":".join("%02X" % b for b in esp.read_mac())
                self.timeout = 1.0
                return "SYNC OK mac=%s" % mac
            except Exception as e:
                last = str(e).splitlines()[0][:52]
                time.sleep(0.3)
        return "no rom (%s)" % last


def trial(port, label, steps, settle=10.0):
    port.drain(settle)
    for control, hold in steps:
        port.hs(control)
        if hold:
            port.drain(hold)
    txt = port.drain(6.0)
    print("  %-34s %-20s %s" % (label, port.verdict(txt), port.sync()), flush=True)
    if "boot:" in txt or "waiting" in txt:
        for line in txt.split("\n"):
            if "boot:" in line or "waiting for download" in line:
                print("      | %s" % line.strip()[:70], flush=True)
                break
    port.hs(RELEASE_BOTH)
    port.drain(0.3)


def main():
    port = UsbipPort()
    print("devid=0x%08x version=%s" % (port.devid, port.init().hex()), flush=True)
    print("=== baseline: is the app legible, and does a plain EN edge boot? ===",
          flush=True)
    trial(port, "A idle only (control)", [(RELEASE_BOTH, 3.0)])
    trial(port, "B EN hold -> release all", [(HOLD_EN, 1.0), (RELEASE_BOTH, 0.0)])
    print("=== IO0 held across the EN edge (bit5=DTR, bit6=RTS) ===", flush=True)
    trial(port, "C IO0+EN -> release EN only", [(BOTH_HELD, 1.0), (HOLD_IO0, 0.0)])
    trial(port, "D IO0+EN -> EN rel -> IO0 rel", [(BOTH_HELD, 1.0), (HOLD_IO0, 0.6), (RELEASE_BOTH, 0.0)])
    trial(port, "E IO0 first, then EN, then rel", [(HOLD_IO0, 0.5), (BOTH_HELD, 1.0), (HOLD_IO0, 0.0)])
    trial(port, "F EN hold, IO0 added, EN rel", [(HOLD_EN, 0.8), (BOTH_HELD, 0.5), (HOLD_IO0, 0.0)])
    print("=== bit7 also held EN in the sweep: try it as the EN line ===", flush=True)
    trial(port, "G bit7 EN + bit5 IO0", [(BIT7_IO0, 1.0), (HOLD_IO0, 0.0)])
    trial(port, "H bit7+bit6 EN + bit5 IO0", [(BIT7_BOTH, 1.0), (HOLD_IO0, 0.0)])
    port.close()


if __name__ == "__main__":
    main()
