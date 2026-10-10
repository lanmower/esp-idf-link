import sys, time, ctypes, serial
from ctypes import wintypes
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200
D0, D1, R0, R1 = 0, 1, 0, 1

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


def both(h, dtr, rts):
    d = DCB()
    d.DCBlength = ctypes.sizeof(DCB)
    if not k32.GetCommState(h, ctypes.byref(d)):
        raise OSError("GetCommState err=%d" % ctypes.get_last_error())
    d.flags = (d.flags & ~(3 << 4)) | ((dtr & 3) << 4)
    d.flags = (d.flags & ~(3 << 12)) | ((rts & 3) << 12)
    if not k32.SetCommState(h, ctypes.byref(d)):
        raise OSError("SetCommState err=%d" % ctypes.get_last_error())


def opened(dtr=False, rts=False):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORT, BAUD, 0.2
    s.dtr, s.rts = dtr, rts
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


def banner(txt):
    for line in txt.decode("latin1").replace("\r", "").split("\n"):
        if "boot:" in line:
            return line.strip()[:56]
    for line in txt.decode("latin1").replace("\r", "").split("\n"):
        if line.startswith("I (") or line.startswith("E ("):
            return "app, no boot banner  " + line.strip()[:34]
    return "SILENT (chip not talking)"


def sync(s, tries=2):
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


def run(tag, open_state, script):
    s = opened(**open_state)
    h = s._port_handle
    try:
        for dtr, rts, d in script:
            both(h, dtr, rts)
            time.sleep(d)
    except OSError as e:
        print("  %-36s %s" % (tag, e), flush=True)
        s.close()
        return
    txt = read(s)
    print("  %-36s %-56s %s" % (tag, banner(txt), sync(s)), flush=True)
    s.close()
    time.sleep(0.8)


print("=== does toggling DTR alone reset the chip? (RTS untouched) ===", flush=True)
run("1 open dtr=0 -> dtr=1 (hold)", {"dtr": False, "rts": False}, [(D1, R0, 2.5)])
run("2 open dtr=1 -> dtr=0 (hold)", {"dtr": True, "rts": False}, [(D0, R0, 2.5)])
run("3 dtr pulse 0->1->0", {"dtr": False, "rts": False},
    [(D1, R0, 0.3), (D0, R0, 2.2)])
run("4 dtr pulse 1->0->1", {"dtr": True, "rts": False},
    [(D0, R0, 0.3), (D1, R0, 2.2)])

print("=== and does DTR override RTS on EN? ===", flush=True)
run("5 rts=1 -> rts=0, dtr=0 throughout", {"dtr": False, "rts": False},
    [(D0, R1, 0.30), (D0, R0, 2.2)])
run("6 rts=1 -> rts=0, dtr=1 throughout", {"dtr": True, "rts": False},
    [(D1, R1, 0.30), (D1, R0, 2.2)])
run("7 rts=1 -> rts=0, dtr 0->1 at the same instant", {"dtr": False, "rts": False},
    [(D1, R1, 0.30), (D1, R0, 2.2)])
run("8 rts=1 -> rts=0, dtr 1->0 at the same instant", {"dtr": True, "rts": False},
    [(D0, R1, 0.30), (D0, R0, 2.2)])
