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
OBUF = ctypes.create_string_buffer(64)


def ioctl(h, code):
    got = wintypes.DWORD()
    if not k32.DeviceIoControl(h, code, None, 0, OBUF, 64, ctypes.byref(got), None):
        raise OSError("0x%08X -> %d" % (code, ctypes.get_last_error()))


def opened(timeout=0.15):
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


def run(tag, script, read_sec=3.0, sync_tries=3):
    s = opened()
    time.sleep(0.15)
    for code, d in script:
        ioctl(s._port_handle, code)
        time.sleep(d)
    s.close()
    time.sleep(0.2)

    s = opened()
    time.sleep(0.25)
    txt = read(s, read_sec).decode("latin1").replace("\r", "")
    print("=" * 70, flush=True)
    print("%s  -> %d bytes" % (tag, len(txt)), flush=True)
    for line in txt.split("\n"):
        if line.strip():
            print("   | " + line[:110], flush=True)
    for i in range(sync_tries):
        try:
            esp = ESP32ROM(s, BAUD)
            esp.sync()
            mac = ":".join("%02X" % b for b in esp.read_mac())
            print("   SYNC OK  mac=%s   <== DOWNLOAD MODE" % mac, flush=True)
            break
        except Exception as e:
            print("   sync try %d: %s" % (i + 1, str(e).splitlines()[0][:60]), flush=True)
    s.close()


T = [
    ("A SET_DTR .1 + RTS pulse (DTR left set)",
     [(SET_DTR, 0.10), (SET_RTS, 0.30), (CLR_RTS, 0.20)]),
    ("B SET_DTR .6 + RTS pulse (DTR left set)",
     [(SET_DTR, 0.60), (SET_RTS, 0.30), (CLR_RTS, 0.30)]),
    ("C control: RTS pulse only", [(SET_RTS, 0.30), (CLR_RTS, 0.20)]),
]

if __name__ == "__main__":
    for tag, script in T:
        run(tag, script)
        time.sleep(0.8)
