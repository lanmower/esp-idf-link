import sys, time, ctypes, serial
from ctypes import wintypes
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200

SETRTS, CLRRTS, SETDTR, CLRDTR = 3, 4, 5, 6
SET_DTR, CLR_DTR = 0x001B0024, 0x001B0028
SET_RTS, CLR_RTS = 0x001B002C, 0x001B0030
GET_DTRRTS = 0x001B0020

DTR_DISABLE, DTR_ENABLE = 0, 1
RTS_DISABLE, RTS_ENABLE = 0, 1

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
k32.EscapeCommFunction.argtypes = [wintypes.HANDLE, wintypes.DWORD]
k32.EscapeCommFunction.restype = wintypes.BOOL
k32.DeviceIoControl.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.LPVOID,
                                wintypes.DWORD, wintypes.LPVOID, wintypes.DWORD,
                                ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID]
k32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
k32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.c_void_p]

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


def opened(timeout=0.2):
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = timeout
    s.dtr = False
    s.rts = False
    s.open()
    time.sleep(0.2)
    return s


def ioctl(h, code):
    got = wintypes.DWORD()
    if not k32.DeviceIoControl(h, code, None, 0, OBUF, 64, ctypes.byref(got), None):
        raise OSError("0x%08X err=%d" % (code, ctypes.get_last_error()))


def dcb_lines(h, dtr=None, rts=None):
    d = DCB()
    d.DCBlength = ctypes.sizeof(DCB)
    if not k32.GetCommState(h, ctypes.byref(d)):
        return "GetCommState err=%d" % ctypes.get_last_error()
    if dtr is not None:
        d.flags = (d.flags & ~(3 << 4)) | ((dtr & 3) << 4)
    if rts is not None:
        d.flags = (d.flags & ~(3 << 12)) | ((rts & 3) << 12)
    if dtr is not None or rts is not None:
        if not k32.SetCommState(h, ctypes.byref(d)):
            return "SetCommState err=%d" % ctypes.get_last_error()
    return "dtr=%d rts=%d" % ((d.flags >> 4) & 3, (d.flags >> 12) & 3)


def escape(h, fn):
    if not k32.EscapeCommFunction(h, fn):
        return "escape err=%d" % ctypes.get_last_error()
    return "ok"


def lines(h):
    v = wintypes.DWORD()
    got = wintypes.DWORD()
    if k32.DeviceIoControl(h, GET_DTRRTS, None, 0, ctypes.byref(v), 64,
                           ctypes.byref(got), None):
        return "DTR=%d RTS=%d" % (bool(v.value & 1), bool(v.value & 2))
    return "dcb:" + dcb_lines(h)


def read(s, sec):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        b = s.read(8192)
        if b:
            buf.extend(b)
    return bytes(buf)


def verdict(s, txt):
    if b"waiting for download" in txt:
        return "BANNER=DOWNLOAD"
    try:
        esp = ESP32ROM(s, BAUD)
        esp.sync()
        return "SYNC OK mac=" + ":".join("%02X" % b for b in esp.read_mac())
    except Exception as e:
        return "no rom, %d bytes" % len(txt)


print("=== 1. does changing RTS release DTR? (go-serial#35) ===", flush=True)
for tag, setter in [("EscapeCommFunction", "escape"),
                    ("DeviceIoControl", "ioctl"),
                    ("SetCommState DCB", "dcb")]:
    s = opened()
    h = s._port_handle
    print("  -- %s" % tag, flush=True)
    if setter == "escape":
        escape(h, SETDTR)
        print("     after SETDTR            : %s" % lines(h), flush=True)
        escape(h, SETRTS)
        print("     after SETRTS            : %s   <-- did DTR drop?" % lines(h), flush=True)
        escape(h, CLRRTS)
        print("     after CLRRTS            : %s" % lines(h), flush=True)
    elif setter == "ioctl":
        ioctl(h, SET_DTR)
        print("     after SET_DTR           : %s" % lines(h), flush=True)
        ioctl(h, SET_RTS)
        print("     after SET_RTS           : %s   <-- did DTR drop?" % lines(h), flush=True)
        ioctl(h, CLR_RTS)
        print("     after CLR_RTS           : %s" % lines(h), flush=True)
    else:
        dcb_lines(h, dtr=DTR_ENABLE)
        print("     after dtr=ENABLE        : %s" % lines(h), flush=True)
        dcb_lines(h, rts=RTS_ENABLE)
        print("     after rts=ENABLE        : %s   <-- did DTR drop?" % lines(h), flush=True)
        dcb_lines(h, rts=RTS_DISABLE)
        print("     after rts=DISABLE       : %s" % lines(h), flush=True)
    time.sleep(0.3)
    txt = read(s, 2.0)
    print("     boot output: %d bytes  %s" % (len(txt), verdict(s, txt)), flush=True)
    s.close()
    time.sleep(0.5)

print("=== 2. assert DTR, reset, then CLEAR DTR and read on the same handle ===", flush=True)
print("    (the ROM latches download mode at boot, so releasing IO0 after the")
print("     edge cannot lose it -- and it un-wedges RX if DTR was the cause)", flush=True)
TRIALS = [
    ("A escape DTR + ioctl RTS", lambda h: (escape(h, SETDTR), time.sleep(0.10),
                                            ioctl(h, SET_RTS), time.sleep(0.30),
                                            ioctl(h, CLR_RTS), time.sleep(0.20),
                                            escape(h, CLRDTR), time.sleep(0.20))),
    ("B dcb DTR + ioctl RTS", lambda h: (dcb_lines(h, dtr=DTR_ENABLE), time.sleep(0.10),
                                        ioctl(h, SET_RTS), time.sleep(0.30),
                                        ioctl(h, CLR_RTS), time.sleep(0.20),
                                        dcb_lines(h, dtr=DTR_DISABLE), time.sleep(0.20))),
    ("C dcb DTR + dcb RTS", lambda h: (dcb_lines(h, dtr=DTR_ENABLE), time.sleep(0.10),
                                      dcb_lines(h, rts=RTS_ENABLE), time.sleep(0.30),
                                      dcb_lines(h, rts=RTS_DISABLE), time.sleep(0.20),
                                      dcb_lines(h, dtr=DTR_DISABLE), time.sleep(0.20))),
    ("D ioctl DTR + ioctl RTS", lambda h: (ioctl(h, SET_DTR), time.sleep(0.10),
                                          ioctl(h, SET_RTS), time.sleep(0.30),
                                          ioctl(h, CLR_RTS), time.sleep(0.20),
                                          ioctl(h, CLR_DTR), time.sleep(0.20))),
    ("E escape DTR + escape RTS", lambda h: (escape(h, SETDTR), time.sleep(0.10),
                                            escape(h, SETRTS), time.sleep(0.30),
                                            escape(h, CLRRTS), time.sleep(0.20),
                                            escape(h, CLRDTR), time.sleep(0.20))),
    ("F control: RTS pulse only", lambda h: (ioctl(h, SET_RTS), time.sleep(0.30),
                                            ioctl(h, CLR_RTS), time.sleep(0.20))),
]
for tag, fn in TRIALS:
    s = opened()
    try:
        fn(s._port_handle)
    except OSError as e:
        print("  %-28s IOCTL ERROR %s" % (tag, e), flush=True)
        s.close()
        continue
    txt = read(s, 2.5)
    print("  %-28s %5d bytes  %s" % (tag, len(txt), verdict(s, txt)), flush=True)
    for line in txt.decode("latin1").replace("\r", "").split("\n"):
        if "boot:" in line or "rst:" in line or "waiting" in line:
            print("        | " + line[:100], flush=True)
    s.close()
    time.sleep(0.6)
