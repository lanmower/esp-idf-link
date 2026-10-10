"""The one combination never yet run: hold DTR from the moment the port opens,
THEN pulse EN with RTS.

Linux's ch341 driver asserts DTR and RTS at open (tty_port_raise_dtr_rts) and
sends both line bits in one atomic vendor request, which is the one structural
difference from Windows. Every Windows trial so far toggled DTR after the port
was already open -- and in probe-dcb2 the dtr=True trials never pulsed RTS at
all, so the chip was never reset while IO0 was held.

Also tries DTR_CONTROL_HANDSHAKE, where the driver itself asserts DTR as long
as the receive buffer is below XoffLim -- another way to get the pin driven
without relying on EscapeCommFunction.
"""
import sys, time, ctypes, serial
from ctypes import wintypes
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200

SETRTS, CLRRTS, SETDTR, CLRDTR = 3, 4, 5, 6
SET_RTS, CLR_RTS = 0x001B002C, 0x001B0030
DTR_ENABLE, DTR_HANDSHAKE = 1, 2
RTS_DISABLE, RTS_ENABLE = 0, 1

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
k32.EscapeCommFunction.argtypes = [wintypes.HANDLE, wintypes.DWORD]
k32.EscapeCommFunction.restype = wintypes.BOOL
k32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
k32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
k32.DeviceIoControl.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.LPVOID,
                                wintypes.DWORD, wintypes.LPVOID, wintypes.DWORD,
                                ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID]
OBUF = ctypes.create_string_buffer(64)


class DCB(ctypes.Structure):
    _fields_ = [("DCBlength", wintypes.DWORD), ("BaudRate", wintypes.DWORD),
                ("flags", wintypes.DWORD), ("wReserved", wintypes.WORD),
                ("XonLim", wintypes.WORD), ("XoffLim", wintypes.WORD),
                ("ByteSize", ctypes.c_ubyte), ("Parity", ctypes.c_ubyte),
                ("StopBits", ctypes.c_ubyte), ("XonChar", ctypes.c_char),
                ("XoffChar", ctypes.c_char), ("ErrorChar", ctypes.c_char),
                ("EofChar", ctypes.c_char), ("EvtChar", ctypes.c_char),
                ("wReserved1", wintypes.WORD)]


class CT(ctypes.Structure):
    _fields_ = [("ReadIntervalTimeout", wintypes.DWORD),
                ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
                ("ReadTotalTimeoutConstant", wintypes.DWORD),
                ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
                ("WriteTotalTimeoutConstant", wintypes.DWORD)]


def opened(dtr=False, rts=False, timeout=0.2):
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = timeout
    s.dtr = dtr
    s.rts = rts
    s.open()
    time.sleep(0.3)
    return s


def dcb(h, dtr=None, rts=None, xoff=None):
    d = DCB()
    d.DCBlength = ctypes.sizeof(DCB)
    k32.GetCommState(h, ctypes.byref(d))
    if dtr is not None:
        d.flags = (d.flags & ~(3 << 4)) | ((dtr & 3) << 4)
    if rts is not None:
        d.flags = (d.flags & ~(3 << 12)) | ((rts & 3) << 12)
    if xoff is not None:
        d.XoffLim = xoff
    return k32.SetCommState(h, ctypes.byref(d))


def ioctl(h, code):
    got = wintypes.DWORD()
    if not k32.DeviceIoControl(h, code, None, 0, OBUF, 64, ctypes.byref(got), None):
        raise OSError("0x%08X err=%d" % (code, ctypes.get_last_error()))


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


def report(tag, s, txt):
    print("  %-38s %5d bytes  %s" % (tag, len(txt), try_sync(s)), flush=True)
    for line in txt.decode("latin1").replace("\r", "").split("\n"):
        if "boot:" in line or "rst:" in line or "waiting" in line:
            print("        | " + line[:100], flush=True)


TRIALS = [
    ("A open dtr=1, escape RTS pulse", lambda s: (
        k32.EscapeCommFunction(s._port_handle, SETRTS), time.sleep(0.30),
        k32.EscapeCommFunction(s._port_handle, CLRRTS), time.sleep(0.30))),
    ("B open dtr=1, ioctl RTS pulse", lambda s: (
        ioctl(s._port_handle, SET_RTS), time.sleep(0.30),
        ioctl(s._port_handle, CLR_RTS), time.sleep(0.30))),
    ("C open dtr=1 rts=1, escape CLRRTS", lambda s: (
        k32.EscapeCommFunction(s._port_handle, CLRRTS), time.sleep(0.40))),
    ("D open dtr=1 rts=1, dcb rts=DISABLE", lambda s: (
        dcb(s._port_handle, rts=RTS_DISABLE), time.sleep(0.40))),
    ("E open dtr=1, dcb handshake + RTS pulse", lambda s: (
        dcb(s._port_handle, dtr=DTR_HANDSHAKE, xoff=4096), time.sleep(0.20),
        k32.EscapeCommFunction(s._port_handle, SETRTS), time.sleep(0.30),
        k32.EscapeCommFunction(s._port_handle, CLRRTS), time.sleep(0.30))),
    ("F control: open dtr=0, RTS pulse", lambda s: (
        k32.EscapeCommFunction(s._port_handle, SETRTS), time.sleep(0.30),
        k32.EscapeCommFunction(s._port_handle, CLRRTS), time.sleep(0.30))),
]

OPEN = {
    "A": dict(dtr=True, rts=False),
    "B": dict(dtr=True, rts=False),
    "C": dict(dtr=True, rts=True),
    "D": dict(dtr=True, rts=True),
    "E": dict(dtr=False, rts=False),
    "F": dict(dtr=False, rts=False),
}

for tag, fn in TRIALS:
    kw = OPEN[tag[0]]
    s = opened(**kw)
    try:
        fn(s)
    except OSError as e:
        print("  %-38s IOCTL ERROR %s" % (tag, e), flush=True)
        s.close()
        time.sleep(0.5)
        continue
    report(tag, s, read(s, 2.5))
    s.close()
    time.sleep(0.7)
