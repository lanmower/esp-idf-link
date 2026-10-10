import sys, time, ctypes, serial
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


def lines(s, dtr, rts):
    h = s._port_handle
    d = DCB()
    d.DCBlength = ctypes.sizeof(DCB)
    if not k32.GetCommState(h, ctypes.byref(d)):
        raise OSError("GetCommState %s" % ctypes.WinError(ctypes.get_last_error()))
    d.flags = (d.flags & ~(3 << 4)) | ((1 if dtr else 0) << 4)
    d.flags = (d.flags & ~(3 << 12)) | ((1 if rts else 0) << 12)
    if not k32.SetCommState(h, ctypes.byref(d)):
        raise OSError("SetCommState %s" % ctypes.WinError(ctypes.get_last_error()))
    s._dtr_state = dtr
    s._rts_state = rts


def sync(s, tag):
    try:
        esp = ESP32ROM(s, BAUD)
        esp.connect("no-reset")
        mac = ":".join("%02X" % b for b in esp.read_mac())
        print("%-28s DOWNLOAD  mac=%s" % (tag, mac), flush=True)
        return True
    except Exception as e:
        print("%-28s fail: %s" % (tag, str(e).splitlines()[0][:60]), flush=True)
        return False


def trial(tag, steps):
    s = serial.Serial(PORT, BAUD, timeout=0.05, dsrdtr=False, rtscts=False)
    try:
        lines(s, False, False)
        time.sleep(0.20)
        n = len(s.read(4096))
        for dtr, rts, d in steps:
            lines(s, dtr, rts)
            time.sleep(d)
        extra = 0
        t0 = time.time()
        while time.time() - t0 < 0.25:
            b = s.read(4096)
            if not b:
                break
            extra += len(b)
        ok = sync(s, tag)
        print("%-28s   pre=%d post=%d bytes" % ("", n, extra), flush=True)
        return ok
    finally:
        try:
            lines(s, False, False)
        except Exception:
            pass
        s.close()


T = [
    ("W1 both->relRTS", [(True, True, .15), (True, False, 0)]),
    ("W2 both->relRTS->relALL", [(True, True, .15), (True, False, .05), (False, False, 0)]),
    ("W3 esptool classic", [(False, True, .15), (True, False, .05), (False, False, 0)]),
    ("W4 reset->relALL ctl", [(False, True, .15), (False, False, 0)]),
    ("W5 io0low->reset->relRTS", [(True, False, .15), (True, True, .15), (True, False, 0)]),
    ("W6 both->relDTR", [(True, True, .15), (False, True, 0)]),
]

if __name__ == "__main__":
    for t, st in T:
        trial(t, st)
        time.sleep(0.5)
