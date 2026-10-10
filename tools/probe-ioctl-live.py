"""Before blaming the driver, prove the instrument moves pins at all.

Every "IOCTL DTR did nothing" result is worthless until an IOCTL has been shown
to move a pin -- so drive RTS (EN) through the exact same DeviceIoControl path
and demand a reset. If that resets the chip, the path is live and the DTR
result is real evidence. If it does not, the IOCTL path is inert and every
IOCTL trial so far was a no-op.

Then sweep all three mechanisms for the one thing that has never been seen:
boot:0x3. Each trial holds the IO0 line through the EN edge, waits well past
the strapping sample, releases it, and only then syncs.
"""
import sys, time, ctypes, serial
from ctypes import wintypes
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200
SETRTS, CLRRTS, SETDTR, CLRDTR = 3, 4, 5, 6
SET_DTR, CLR_DTR, SET_RTS, CLR_RTS = 0x001B0024, 0x001B0028, 0x001B002C, 0x001B0030

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


def opened(dtr=False, rts=False, timeout=0.25):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORT, BAUD, timeout
    s.dtr, s.rts = dtr, rts
    s.open()
    time.sleep(0.25)
    return s


def esc(h, fn):
    ok = k32.EscapeCommFunction(h, fn)
    if not ok:
        raise OSError("escape err=%d" % ctypes.get_last_error())
    return ok


def ioctl(h, code):
    got = wintypes.DWORD()
    ok = k32.DeviceIoControl(h, code, None, 0, OBUF, 64, ctypes.byref(got), None)
    if not ok:
        raise OSError("ioctl 0x%08X err=%d" % (code, ctypes.get_last_error()))
    return got.value


def dcb(h, dtr, rts):
    d = DCB()
    d.DCBlength = ctypes.sizeof(DCB)
    if not k32.GetCommState(h, ctypes.byref(d)):
        raise OSError("GetCommState err=%d" % ctypes.get_last_error())
    d.flags = (d.flags & ~(3 << 4)) | ((dtr & 3) << 4)
    d.flags = (d.flags & ~(3 << 12)) | ((rts & 3) << 12)
    if not k32.SetCommState(h, ctypes.byref(d)):
        raise OSError("SetCommState err=%d" % ctypes.get_last_error())


def sync(s, tries=3):
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


def banner(s, sec=2.5):
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
            return line.strip()[:46]
    return "app(no banner)" if txt else "silent"


def run(tag, open_kw, script):
    s = opened(**open_kw)
    h = s._port_handle
    try:
        for fn, d in script:
            fn(h)
            time.sleep(d)
    except OSError as e:
        print("  %-44s %s" % (tag, e), flush=True)
        s.close()
        time.sleep(0.5)
        return
    print("  %-44s %-22s %s" % (tag, banner(s), sync(s)), flush=True)
    s.close()
    time.sleep(0.6)


print("=== 1. does the DeviceIoControl path move ANY pin? ===", flush=True)
run("A ioctl RTS pulse only (proves path live)", {},
    [(lambda h: ioctl(h, SET_RTS), 0.30), (lambda h: ioctl(h, CLR_RTS), 2.0)])
run("B ioctl DTR pulse only", {},
    [(lambda h: ioctl(h, SET_DTR), 0.30), (lambda h: ioctl(h, CLR_DTR), 2.0)])
run("C escape RTS pulse only (control)", {},
    [(lambda h: esc(h, SETRTS), 0.30), (lambda h: esc(h, CLRRTS), 2.0)])

print("=== 2. IO0 low across the EN edge, then released, then sync ===", flush=True)
HOLD = [
    (lambda h: ioctl(h, SET_DTR), 0.15),
    (lambda h: ioctl(h, SET_RTS), 0.30),
    (lambda h: ioctl(h, CLR_RTS), 1.50),
    (lambda h: ioctl(h, CLR_DTR), 0.60),
]
run("D ioctl: SET_DTR + EN edge, release DTR late", {}, HOLD)

HOLD2 = [
    (lambda h: esc(h, SETDTR), 0.15),
    (lambda h: esc(h, SETRTS), 0.30),
    (lambda h: esc(h, CLRRTS), 1.50),
    (lambda h: esc(h, CLRDTR), 0.60),
]
run("E escape: SET_DTR + EN edge, release DTR late", {}, HOLD2)

HOLD3 = [
    (lambda h: esc(h, CLRDTR), 0.15),
    (lambda h: esc(h, SETRTS), 0.30),
    (lambda h: esc(h, CLRRTS), 1.50),
    (lambda h: esc(h, SETDTR), 0.60),
]
run("F escape: CLR_DTR + EN edge (other polarity)", {}, HOLD3)

print("=== 3. IO0 held low, port reopened clean, then sync ===", flush=True)


def two_phase(tag, script):
    s = opened()
    try:
        for fn, d in script:
            fn(s._port_handle)
            time.sleep(d)
    except OSError as e:
        print("  %-44s %s" % (tag, e), flush=True)
        s.close()
        return
    s.close()
    time.sleep(0.3)
    s2 = opened()
    print("  %-44s %-22s %s" % (tag, banner(s2, 2.0), sync(s2)), flush=True)
    s2.close()
    time.sleep(0.6)


two_phase("G ioctl SET_DTR + EN edge, reopen clean", HOLD[:3] + [(lambda h: None, 0.4)])
two_phase("H escape SET_DTR + EN edge, reopen clean", HOLD2[:3] + [(lambda h: None, 0.4)])
