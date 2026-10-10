"""Two-phase entry, the shape that survives every defect seen so far.

Phase 1: drive the EN edge with IO0 held (whatever DTR does), and leave the
        chip alone -- do NOT read through a port that has DTR asserted, because
        the WCH 3.9 read path is the documented defect here.
Phase 2: close the port, reopen it with DTR and RTS clear, and sync with
        --before no_reset. The ROM latched download mode at the EN edge, so
        releasing IO0 afterwards cannot lose it -- and the reopen clears any
        DTR-related RX wedge before a single byte is read.

Trial 6 of probe-map was SILENT where trial 5 booted: the first real sign DTR
changes anything. Chase it.
"""
import sys, time, ctypes, serial
from ctypes import wintypes
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200
SETRTS, CLRRTS, SETDTR, CLRDTR = 3, 4, 5, 6
D0, D1, R0, R1 = 0, 1, 0, 1

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
k32.EscapeCommFunction.argtypes = [wintypes.HANDLE, wintypes.DWORD]
k32.EscapeCommFunction.restype = wintypes.BOOL
k32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
k32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.c_void_p]


class DCB(ctypes.Structure):
    _fields_ = [("DCBlength", wintypes.DWORD), ("BaudRate", wintypes.DWORD),
                ("flags", wintypes.DWORD), ("wReserved", wintypes.WORD),
                ("XonLim", wintypes.WORD), ("XoffLim", wintypes.WORD),
                ("ByteSize", ctypes.c_ubyte), ("Parity", ctypes.c_ubyte),
                ("StopBits", ctypes.c_ubyte), ("XonChar", ctypes.c_char),
                ("XoffChar", ctypes.c_char), ("ErrorChar", ctypes.c_char),
                ("EofChar", ctypes.c_char), ("EvtChar", ctypes.c_char),
                ("wReserved1", wintypes.WORD)]


def both(h, dtr, rts):
    d = DCB()
    d.DCBlength = ctypes.sizeof(DCB)
    if not k32.GetCommState(h, ctypes.byref(d)):
        raise OSError("GetCommState err=%d" % ctypes.get_last_error())
    d.flags = (d.flags & ~(3 << 4)) | ((dtr & 3) << 4)
    d.flags = (d.flags & ~(3 << 12)) | ((rts & 3) << 12)
    if not k32.SetCommState(h, ctypes.byref(d)):
        raise OSError("SetCommState err=%d" % ctypes.get_last_error())


def esc(h, fn):
    if not k32.EscapeCommFunction(h, fn):
        raise OSError("escape err=%d" % ctypes.get_last_error())


def opened(dtr=False, rts=False, timeout=0.2):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORT, BAUD, timeout
    s.dtr, s.rts = dtr, rts
    s.open()
    time.sleep(0.25)
    return s


def reopen(dtr=False, rts=False):
    """Phase 2: fresh handle, lines clear, no reset."""
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORT, BAUD, 0.3
    s.dtr, s.rts = dtr, rts
    s.open()
    time.sleep(0.15)
    return s


def sync(s, tries=4):
    for i in range(tries):
        try:
            s.reset_input_buffer()
            esp = ESP32ROM(s, BAUD)
            esp.sync()
            return "SYNC OK mac=" + ":".join("%02X" % b for b in esp.read_mac())
        except Exception as e:
            last = str(e).splitlines()[0][:44]
            time.sleep(0.2)
    return "no rom (%s)" % last


def peek(s, sec=1.5):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        b = s.read(8192)
        if b:
            buf.extend(b)
    txt = bytes(buf).decode("latin1").replace("\r", "")
    for line in txt.split("\n"):
        if "waiting for download" in line:
            return "WAITING-FOR-DOWNLOAD"
        if "boot:" in line:
            return line.strip()[:44]
    return "silent" if not txt else "chatter"


def two_phase(tag, phase1, reopen_state=None):
    """Run phase1 on one handle, then re-open clean and sync."""
    s = opened()
    try:
        phase1(s)
    except OSError as e:
        print("  %-40s %s" % (tag, e), flush=True)
        s.close()
        return
    s.close()
    time.sleep(0.35)
    rs = reopen_state or {}
    s2 = reopen(**rs)
    print("  %-40s %-22s %s" % (tag, peek(s2), sync(s2)), flush=True)
    s2.close()
    time.sleep(0.7)


def p_escape_dtr(s):
    h = s._port_handle
    esc(h, SETDTR)
    time.sleep(0.10)
    esc(h, SETRTS)
    time.sleep(0.25)
    esc(h, CLRRTS)
    time.sleep(0.30)


def p_escape_dtr_clear(s):
    h = s._port_handle
    esc(h, SETDTR)
    time.sleep(0.10)
    esc(h, SETRTS)
    time.sleep(0.25)
    esc(h, CLRRTS)
    time.sleep(0.25)
    esc(h, CLRDTR)
    time.sleep(0.25)


def p_dcb_dtr(s):
    h = s._port_handle
    both(h, D1, R0)
    time.sleep(0.10)
    both(h, D1, R1)
    time.sleep(0.25)
    both(h, D1, R0)
    time.sleep(0.35)


def p_dcb_dtr_clear(s):
    h = s._port_handle
    both(h, D1, R0)
    time.sleep(0.10)
    both(h, D1, R1)
    time.sleep(0.25)
    both(h, D1, R0)
    time.sleep(0.25)
    both(h, D0, R0)
    time.sleep(0.25)


def p_open_dtr(s_unused):
    pass


def p_control(s):
    h = s._port_handle
    esc(h, SETRTS)
    time.sleep(0.25)
    esc(h, CLRRTS)
    time.sleep(0.30)


print("=== two-phase: enter with DTR, then reopen clean and sync ===", flush=True)
two_phase("A escape DTR, RTS edge, keep DTR", p_escape_dtr)
two_phase("B escape DTR, RTS edge, clear DTR", p_escape_dtr_clear)
two_phase("C dcb DTR, RTS edge, keep DTR", p_dcb_dtr)
two_phase("D dcb DTR, RTS edge, clear DTR", p_dcb_dtr_clear)
two_phase("E control: RTS edge only", p_control)
two_phase("F edge with DTR, reopen WITH DTR held", p_dcb_dtr, {"dtr": True, "rts": False})
two_phase("G edge with DTR, reopen DTR+RTS held", p_dcb_dtr, {"dtr": True, "rts": True})

print("=== one-phase control: same handle, no reopen ===", flush=True)
s = opened()
p_dcb_dtr(s)
print("  %-40s %-22s %s" % ("H dcb DTR + edge, no reopen", peek(s), sync(s)), flush=True)
s.close()
