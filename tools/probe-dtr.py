import sys, time, re, ctypes, serial
from ctypes import wintypes
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200
k32 = ctypes.WinDLL("kernel32", use_last_error=True)


class DCB(ctypes.Structure):
    _fields_ = [("DCBlength", wintypes.DWORD), ("BaudRate", wintypes.DWORD),
                ("flags", wintypes.DWORD), ("wReserved", wintypes.WORD),
                ("XonLim", wintypes.WORD), ("XoffLim", wintypes.WORD),
                ("ByteSize", ctypes.c_ubyte), ("Parity", ctypes.c_ubyte),
                ("StopBits", ctypes.c_ubyte), ("XonChar", ctypes.c_char),
                ("XoffChar", ctypes.c_char), ("ErrorChar", ctypes.c_char),
                ("EofChar", ctypes.c_char), ("EvtChar", ctypes.c_char),
                ("wReserved1", wintypes.WORD)]


k32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
k32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.c_void_p]


def dcb_lines(s, dtr, rts):
    h = s._port_handle
    d = DCB()
    d.DCBlength = ctypes.sizeof(DCB)
    k32.GetCommState(h, ctypes.byref(d))
    d.flags = (d.flags & ~(3 << 4)) | ((1 if dtr else 0) << 4)
    d.flags = (d.flags & ~(3 << 12)) | ((1 if rts else 0) << 12)
    k32.SetCommState(h, ctypes.byref(d))
    s._dtr_state = dtr
    s._rts_state = rts


def opened():
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = 0.05
    s.dtr = False
    s.rts = False
    s.open()
    time.sleep(0.15)
    return s


def watch(s, sec=2.5):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        b = s.read(4096)
        if b:
            buf.extend(b)
    txt = bytes(buf).decode("latin1")
    m = re.search(r"boot:0x([0-9a-f]+)", txt)
    r = re.search(r"rst:0x[0-9a-f]+ \((\w+)\)", txt)
    return len(buf), (m.group(1) if m else None), (r.group(1) if r else None)


def sync(s):
    try:
        esp = ESP32ROM(s, BAUD)
        esp.connect("no-reset")
        return ":".join("%02X" % b for b in esp.read_mac())
    except Exception as e:
        return "fail(%s)" % str(e).splitlines()[0][:44]


def trial(tag, fn):
    s = opened()
    try:
        fn(s)
        n, boot, rst = watch(s)
        mac = sync(s)
        print("%-26s bytes=%-6d boot=%-5s rst=%-14s sync=%s"
              % (tag, n, boot, rst, mac), flush=True)
    finally:
        try:
            s.setDTR(False)
            s.setRTS(False)
        except Exception:
            pass
        s.close()


def dtr_pulse(s):
    s.setDTR(True); time.sleep(0.30); s.setDTR(False)


def rts_pulse(s):
    s.setRTS(True); time.sleep(0.30); s.setRTS(False)


def dtr_pulse_dcb(s):
    dcb_lines(s, True, False); time.sleep(0.30); dcb_lines(s, False, False)


def dtr_hold_rts_pulse(s):
    s.setDTR(True); time.sleep(0.10)
    s.setRTS(True); time.sleep(0.30); s.setRTS(False)


def rts_hold_dtr_pulse(s):
    s.setRTS(True); time.sleep(0.10)
    s.setDTR(True); time.sleep(0.30); s.setDTR(False)
    time.sleep(0.05); s.setRTS(False)


def classic_reapply(s):
    s.setDTR(False); s.setRTS(True); s.setDTR(False); time.sleep(0.15)
    s.setDTR(True); s.setRTS(False); s.setDTR(True); time.sleep(0.05)


def both_held_rel_rts_reapply(s):
    s.setDTR(True); s.setRTS(True); s.setDTR(True); time.sleep(0.20)
    s.setRTS(False); s.setDTR(True); time.sleep(0.05)


T = [
    ("1 ctl nothing", lambda s: None),
    ("2 DTR pulse", dtr_pulse),
    ("3 RTS pulse", rts_pulse),
    ("4 DTR pulse (DCB)", dtr_pulse_dcb),
    ("5 DTR held + RTS pulse", dtr_hold_rts_pulse),
    ("6 RTS held + DTR pulse", rts_hold_dtr_pulse),
    ("7 classic + DTR reapply", classic_reapply),
    ("8 both held, rel RTS reap", both_held_rel_rts_reapply),
]

if __name__ == "__main__":
    for t, f in T:
        trial(t, f)
        time.sleep(0.4)
