"""A real serial port on top of the CH341 over USB/IP -- Windows never touched.

ch341.py proved bulk IN works (the app's log is legible) and that the 0xA4
control byte moves EN (bit6) and IO0 (bit5). It had no write path, so esptool
could never talk over this channel and every entry attempt had to hand the
device back to the WCH driver -- whose DTR has never moved in 75+ trials.

This adds bulk OUT and the pyserial subset esptool needs, so one process holds
the EN edge, the IO0 level and the UART at the same time. That removes the last
confound: no reclaim, no driver line state, no reopen glitch.

    python tools/usbip-port.py           # sweep entry sequences, sync each
"""
import os, socket, struct, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ch341 import Ch341, recvn

BIT_DTR, BIT_RTS = 0x20, 0x40          # 0xA4 control byte, as ch341.c drives it
EP_IN, EP_OUT = 0x82, 0x02


class UsbipPort(Ch341):
    """Ch341 + bulk OUT + the pyserial surface esptool uses."""

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

    # -- bulk OUT ------------------------------------------------------
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

    # -- bulk IN -------------------------------------------------------
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

    # -- pyserial surface ----------------------------------------------
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

    # -- helpers -------------------------------------------------------
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
    """steps: list of (control_byte, hold_seconds). Then read the ROM, then sync."""
    port.drain(settle)                       # let the app get chatty first
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
    port.hs(0x00)
    port.drain(0.3)


def main():
    port = UsbipPort()
    print("devid=0x%08x version=%s" % (port.devid, port.init().hex()), flush=True)
    print("=== baseline: is the app legible, and does a plain EN edge boot? ===",
          flush=True)
    trial(port, "A idle only (control)", [(0x00, 3.0)])
    trial(port, "B EN hold -> release all", [(0x40, 1.0), (0x00, 0.0)])
    print("=== IO0 held across the EN edge (bit5=DTR, bit6=RTS) ===", flush=True)
    trial(port, "C IO0+EN -> release EN only", [(0x60, 1.0), (0x20, 0.0)])
    trial(port, "D IO0+EN -> EN rel -> IO0 rel", [(0x60, 1.0), (0x20, 0.6), (0x00, 0.0)])
    trial(port, "E IO0 first, then EN, then rel", [(0x20, 0.5), (0x60, 1.0), (0x20, 0.0)])
    trial(port, "F EN hold, IO0 added, EN rel", [(0x40, 0.8), (0x60, 0.5), (0x20, 0.0)])
    print("=== bit7 also held EN in the sweep: try it as the EN line ===", flush=True)
    trial(port, "G bit7 EN + bit5 IO0", [(0xA0, 1.0), (0x20, 0.0)])
    trial(port, "H bit7+bit6 EN + bit5 IO0", [(0xE0, 1.0), (0x20, 0.0)])
    port.close()


if __name__ == "__main__":
    main()
