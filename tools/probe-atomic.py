import sys, time, ctypes, serial
from ctypes import wintypes
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200
DTR_DISABLE, DTR_ENABLE = 0, 1
RTS_DISABLE, RTS_ENABLE = 0, 1

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
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


def both(h, dtr=None, rts=None):
    d = DCB()
    d.DCBlength = ctypes.sizeof(DCB)
    if not k32.GetCommState(h, ctypes.byref(d)):
        raise OSError("GetCommState err=%d" % ctypes.get_last_error())
    if dtr is not None:
        d.flags = (d.flags & ~(3 << 4)) | ((dtr & 3) << 4)
    if rts is not None:
        d.flags = (d.flags & ~(3 << 12)) | ((rts & 3) << 12)
    if not k32.SetCommState(h, ctypes.byref(d)):
        raise OSError("SetCommState err=%d" % ctypes.get_last_error())


def opened():
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORT, BAUD, 0.2
    s.dtr, s.rts = False, False
    s.open()
    time.sleep(0.3)
    return s


def read(s, sec=2.5):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        b = s.read(8192)
        if b:
            buf.extend(b)
    return bytes(buf)


def sync(s, tries=3):
    for i in range(tries):
        try:
            s.reset_input_buffer()
            esp = ESP32ROM(s, BAUD)
            esp.sync()
            return "SYNC OK mac=" + ":".join("%02X" % b for b in esp.read_mac())
        except Exception as e:
            last = str(e).splitlines()[0][:44]
            time.sleep(0.15)
    return "no rom (%s)" % last


def run(tag, script, read_first=True):
    s = opened()
    h = s._port_handle
    try:
        for op, d in script:
            both(h, **op)
            time.sleep(d)
    except OSError as e:
        print("  %-40s %s" % (tag, e), flush=True)
        s.close()
        return
    out = "no banner"
    synced = ""
    if read_first:
        txt = read(s)
        for line in txt.decode("latin1").replace("\r", "").split("\n"):
            if "boot:" in line:
                out = line.strip()[:52]
                break
        synced = sync(s)
    else:
        synced = sync(s)
        txt = read(s, 1.5)
        for line in txt.decode("latin1").replace("\r", "").split("\n"):
            if "boot:" in line:
                out = line.strip()[:52]
                break
    print("  %-40s %-52s %s" % (tag, out, synced), flush=True)
    s.close()
    time.sleep(0.8)


print("=== atomic SetCommState: both lines in one call ===", flush=True)
run("A atomic dtr=1,rts=1 -> dtr=1,rts=0",
    [({"dtr": DTR_ENABLE, "rts": RTS_ENABLE}, 0.30),
     ({"dtr": DTR_ENABLE, "rts": RTS_DISABLE}, 0.50)])
run("B atomic dtr=1,rts=1 -> dtr=1,rts=0 (sync first)",
    [({"dtr": DTR_ENABLE, "rts": RTS_ENABLE}, 0.30),
     ({"dtr": DTR_ENABLE, "rts": RTS_DISABLE}, 0.50)], read_first=False)
run("C control atomic dtr=0,rts=1 -> dtr=0,rts=0",
    [({"dtr": DTR_DISABLE, "rts": RTS_ENABLE}, 0.30),
     ({"dtr": DTR_DISABLE, "rts": RTS_DISABLE}, 0.50)])
run("D atomic dtr=1,rts=1 -> dtr=1,rts=0 -> dtr=0",
    [({"dtr": DTR_ENABLE, "rts": RTS_ENABLE}, 0.30),
     ({"dtr": DTR_ENABLE, "rts": RTS_DISABLE}, 0.30),
     ({"dtr": DTR_DISABLE, "rts": RTS_DISABLE}, 0.30)])
run("E hold dtr=1 in EVERY call, pulse rts",
    [({"dtr": DTR_ENABLE}, 0.20),
     ({"dtr": DTR_ENABLE, "rts": RTS_ENABLE}, 0.30),
     ({"dtr": DTR_ENABLE, "rts": RTS_DISABLE}, 0.50)])
run("F dtr=1 only, then rts=1, then release rts via escape",
    [({"dtr": DTR_ENABLE}, 0.20),
     ({"dtr": DTR_ENABLE, "rts": RTS_ENABLE}, 0.30),
     ({"dtr": DTR_ENABLE, "rts": RTS_DISABLE}, 0.50)])
