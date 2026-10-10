import sys, time, ctypes, serial
from ctypes import wintypes
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200
MAXDWORD = 0xFFFFFFFF

SETRTS, CLRRTS, SETDTR, CLRDTR = 3, 4, 5, 6
GET_DTRRTS = 0x001B0020
FN = {SETDTR: "SETDTR", CLRDTR: "CLRDTR", SETRTS: "SETRTS", CLRRTS: "CLRRTS"}

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
k32.EscapeCommFunction.argtypes = [wintypes.HANDLE, wintypes.DWORD]
k32.EscapeCommFunction.restype = wintypes.BOOL
k32.SetCommTimeouts.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
k32.DeviceIoControl.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.LPVOID,
                                wintypes.DWORD, wintypes.LPVOID, wintypes.DWORD,
                                ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID]


class CT(ctypes.Structure):
    _fields_ = [("ReadIntervalTimeout", wintypes.DWORD),
                ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
                ("ReadTotalTimeoutConstant", wintypes.DWORD),
                ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
                ("WriteTotalTimeoutConstant", wintypes.DWORD)]


def escape(h, fn):
    ok = k32.EscapeCommFunction(h, fn)
    return "ok=%d err=%d" % (ok, ctypes.get_last_error())


def lines(h):
    v = wintypes.DWORD()
    got = wintypes.DWORD()
    if not k32.DeviceIoControl(h, GET_DTRRTS, None, 0, ctypes.byref(v), 4,
                               ctypes.byref(got), None):
        return "GET_DTRRTS err=%d" % ctypes.get_last_error()
    return "DTR=%d RTS=%d" % (bool(v.value & 1), bool(v.value & 2))


def set_timeouts(h, interval, const):
    t = CT()
    t.ReadIntervalTimeout = interval
    t.ReadTotalTimeoutMultiplier = 0
    t.ReadTotalTimeoutConstant = const
    t.WriteTotalTimeoutMultiplier = 0
    t.WriteTotalTimeoutConstant = 0
    return "SetCommTimeouts=%d" % k32.SetCommTimeouts(h, ctypes.byref(t))


def opened(timeout=0.15):
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = timeout
    s.dtr = False
    s.rts = False
    s.open()
    time.sleep(0.15)
    return s


def read(s, sec):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        b = s.read(8192)
        if b:
            buf.extend(b)
    return bytes(buf)


def sync(s, tries=2):
    for i in range(tries):
        try:
            esp = ESP32ROM(s, BAUD)
            esp.sync()
            return "SYNC OK mac=" + ":".join("%02X" % b for b in esp.read_mac())
        except Exception as e:
            last = str(e).splitlines()[0][:52]
    return "no rom (%s)" % last


print("--- A. does the driver accept EscapeCommFunction, and does it believe it ---", flush=True)
s = opened()
h = s._port_handle
print("  after open            : %s" % lines(h), flush=True)
for fn in (SETDTR, SETRTS, CLRDTR, CLRRTS, SETDTR, SETRTS):
    print("  %-6s -> %-14s  %s" % (FN[fn], escape(h, fn), lines(h)), flush=True)
    time.sleep(0.05)
s.close()

print("--- B. same via pyserial's own setDTR/setRTS ---", flush=True)
s = opened()
print("  start                 : %s" % lines(s._port_handle), flush=True)
s.setDTR(True)
print("  setDTR(True)          : %s" % lines(s._port_handle), flush=True)
s.setRTS(True)
print("  setRTS(True)          : %s" % lines(s._port_handle), flush=True)
s.setDTR(False)
print("  setDTR(False)         : %s" % lines(s._port_handle), flush=True)
s.setRTS(False)
s.close()

print("--- C. reset into (hoped) download mode, then re-apply comm timeouts ---", flush=True)
for tag, interval, const in [("default timeouts", None, None),
                             ("interval=MAXDWORD const=2000", MAXDWORD, 2000),
                             ("interval=0 const=3000", 0, 3000)]:
    s = opened()
    h = s._port_handle
    escape(h, SETDTR)
    time.sleep(0.10)
    escape(h, SETRTS)
    time.sleep(0.30)
    escape(h, CLRRTS)
    time.sleep(0.15)
    if interval is not None:
        set_timeouts(h, interval, const)
    txt = read(s, 3.0)
    banner = "BANNER" if b"waiting for download" in txt else "      "
    print("  %-28s %5d bytes %s %s" % (tag, len(txt), banner, sync(s)), flush=True)
    s.close()
    time.sleep(0.4)

print("--- D. control: RTS pulse only (no DTR) ---", flush=True)
s = opened()
h = s._port_handle
escape(h, SETRTS)
time.sleep(0.30)
escape(h, CLRRTS)
txt = read(s, 3.0)
print("  %5d bytes  %s" % (len(txt), sync(s)), flush=True)
s.close()
