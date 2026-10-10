"""Last admin-free hypothesis: the WCH driver may implement the serial IOCTLs
(IOCTL_SERIAL_SET_DTR/CLR_DTR/SET_RTS/CLR_RTS) even though EscapeCommFunction --
what pyserial and therefore esptool use -- does not reach GPIO0.

Each trial applies a line sequence through raw DeviceIoControl, closes the
handle, reopens it clean, reads, then asks the ROM to answer sync. In download
mode the ROM answers sync forever; the running app just prints.
"""
import sys, time, ctypes, serial
from ctypes import wintypes
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200

SET_DTR, CLR_DTR = 0x001B0024, 0x001B0028
SET_RTS, CLR_RTS = 0x001B002C, 0x001B0030

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
k32.DeviceIoControl.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.LPVOID,
                                wintypes.DWORD, wintypes.LPVOID, wintypes.DWORD,
                                ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID]
k32.DeviceIoControl.restype = wintypes.BOOL


OBUF = ctypes.create_string_buffer(64)


def ioctl(h, code):
    got = wintypes.DWORD()
    ok = k32.DeviceIoControl(h, code, None, 0, OBUF, 64, ctypes.byref(got), None)
    if not ok:
        raise OSError("DeviceIoControl 0x%08X failed: %d" % (code, ctypes.get_last_error()))


NAMES = {SET_DTR: "SET_DTR", CLR_DTR: "CLR_DTR", SET_RTS: "SET_RTS", CLR_RTS: "CLR_RTS"}


def opened(timeout=0.1):
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = timeout
    s.dtr = False
    s.rts = False
    s.open()
    return s


def read(s, sec):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        b = s.read(8192)
        if b:
            buf.extend(b)
    return bytes(buf)


def trial(tag, script):
    s = opened()
    time.sleep(0.15)
    h = s._port_handle
    note = ""
    try:
        for code, d in script:
            ioctl(h, code)
            time.sleep(d)
    except OSError as e:
        note = "IOCTL ERR %s" % e
    s.close()

    time.sleep(0.2)
    s = opened()
    time.sleep(0.25)
    txt = read(s, 2.0).decode("latin1").replace("\r", "")
    verdict = "-"
    try:
        esp = ESP32ROM(s, BAUD)
        esp.sync()
        mac = ":".join("%02X" % b for b in esp.read_mac())
        verdict = "ROM DOWNLOAD MODE mac=%s  <== WIN" % mac
    except Exception as e:
        verdict = "no rom (%s)" % str(e).splitlines()[0][:38]
    rom_banner = "BANNER" if "waiting for download" in txt else "      "
    print("%-34s %5d %s %-38s %s"
          % (tag, len(txt), rom_banner, verdict, note), flush=True)
    if txt and len(txt) < 400:
        for line in txt.strip().split("\n")[:6]:
            print("      | " + line[:96])
    s.close()


T = [
    ("1 ctl RTS pulse", [(SET_RTS, 0.30), (CLR_RTS, 0.20)]),
    ("2 SET_DTR + RTS pulse", [(SET_DTR, 0.10), (SET_RTS, 0.30), (CLR_RTS, 0.20)]),
    ("3 SET_DTR + RTS pulse + CLR_DTR",
     [(SET_DTR, 0.10), (SET_RTS, 0.30), (CLR_RTS, 0.20), (CLR_DTR, 0.10)]),
    ("4 esptool classic via IOCTL",
     [(SET_RTS, 0.15), (SET_DTR, 0.05), (CLR_RTS, 0.05), (CLR_DTR, 0.05)]),
    ("5 SET_DTR long + RTS pulse",
     [(SET_DTR, 0.60), (SET_RTS, 0.30), (CLR_RTS, 0.30)]),
    ("6 SET_DTR, RTS pulse x2",
     [(SET_DTR, 0.10), (SET_RTS, 0.30), (CLR_RTS, 0.30), (SET_RTS, 0.30), (CLR_RTS, 0.30)]),
    ("7 SET_DTR then CLR_DTR before EN rel",
     [(SET_DTR, 0.10), (SET_RTS, 0.30), (CLR_DTR, 0.10), (CLR_RTS, 0.30)]),
]

if __name__ == "__main__":
    print("codes: " + ", ".join("%s=0x%08X" % (v, k) for k, v in NAMES.items()), flush=True)
    for tag, script in T:
        trial(tag, script)
        time.sleep(0.6)
