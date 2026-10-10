import sys, time, ctypes, serial
from ctypes import wintypes
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200

SETRTS, CLRRTS, SETDTR, CLRDTR = 3, 4, 5, 6
NAMES = {0: "DISABLE", 1: "ENABLE", 2: "HANDSHAKE", 3: "?3"}

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


def get(h):
    d = DCB()
    d.DCBlength = ctypes.sizeof(DCB)
    if not k32.GetCommState(h, ctypes.byref(d)):
        return None, None
    return (d.flags >> 4) & 3, (d.flags >> 12) & 3


def put(h, dtr=None, rts=None):
    d = DCB()
    d.DCBlength = ctypes.sizeof(DCB)
    k32.GetCommState(h, ctypes.byref(d))
    if dtr is not None:
        d.flags = (d.flags & ~(3 << 4)) | ((dtr & 3) << 4)
    if rts is not None:
        d.flags = (d.flags & ~(3 << 12)) | ((rts & 3) << 12)
    return k32.SetCommState(h, ctypes.byref(d))


def show(tag, h):
    d, r = get(h)
    print("   %-34s fDtrControl=%-9s fRtsControl=%s" % (tag, NAMES[d], NAMES[r]), flush=True)


def opened(**kw):
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = 0.2
    s.dtr = kw.get("dtr", False)
    s.rts = kw.get("rts", False)
    s.open()
    time.sleep(0.2)
    return s


def read(s, sec):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        b = s.read(8192)
        if b:
            buf.extend(b)
    return bytes(buf)


def try_sync(s):
    try:
        esp = ESP32ROM(s, BAUD)
        esp.sync()
        return "SYNC OK mac=" + ":".join("%02X" % b for b in esp.read_mac())
    except Exception as e:
        return "no rom (%s)" % str(e).splitlines()[0][:40]


print("=== 1. what does the driver think fDtrControl is, and does it stick? ===", flush=True)
s = opened()
h = s._port_handle
show("at open (dtr=False, rts=False)", h)
k32.EscapeCommFunction(h, SETDTR)
show("after EscapeCommFunction(SETDTR)", h)
k32.EscapeCommFunction(h, CLRDTR)
show("after EscapeCommFunction(CLRDTR)", h)
print("   SetCommState dtr=ENABLE -> %d" % put(h, dtr=1), flush=True)
show("after SetCommState dtr=ENABLE", h)
k32.EscapeCommFunction(h, SETRTS)
show("after EscapeCommFunction(SETRTS)", h)
k32.EscapeCommFunction(h, CLRRTS)
show("after EscapeCommFunction(CLRRTS)", h)
k32.EscapeCommFunction(h, SETDTR)
show("after EscapeCommFunction(SETDTR)", h)
s.close()

print("=== 2. does pyserial open(dtr=True) reach the hardware? ===", flush=True)
for tag, kw in [("open dtr=True, rts=False", {"dtr": True, "rts": False}),
                ("open dtr=True, rts=True", {"dtr": True, "rts": True})]:
    s = opened(**kw)
    show(tag, s._port_handle)
    time.sleep(0.3)
    txt = read(s, 1.5)
    print("   %5d bytes  %s" % (len(txt), try_sync(s)), flush=True)
    s.close()
    time.sleep(0.5)

print("=== 3. DCB dtr=ENABLE held, RTS pulsed, then sync ===", flush=True)
s = opened()
h = s._port_handle
put(h, dtr=1)
show("dtr=ENABLE", h)
time.sleep(0.10)
k32.EscapeCommFunction(h, SETRTS)
time.sleep(0.30)
k32.EscapeCommFunction(h, CLRRTS)
time.sleep(0.30)
txt = read(s, 2.5)
print("   %5d bytes  %s" % (len(txt), try_sync(s)), flush=True)
s.close()
time.sleep(0.5)

print("=== 4. control: plain open, RTS pulse only ===", flush=True)
s = opened()
k32.EscapeCommFunction(s._port_handle, SETRTS)
time.sleep(0.30)
k32.EscapeCommFunction(s._port_handle, CLRRTS)
txt = read(s, 2.5)
print("   %5d bytes  %s" % (len(txt), try_sync(s)), flush=True)
s.close()
