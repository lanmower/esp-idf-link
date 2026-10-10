import sys, time, ctypes, serial
from ctypes import wintypes
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200
SETRTS, CLRRTS = 3, 4

k32 = ctypes.WinDLL("kernel32", use_last_error=True)
k32.EscapeCommFunction.argtypes = [wintypes.HANDLE, wintypes.DWORD]
k32.EscapeCommFunction.restype = wintypes.BOOL


def opened(dtr=False, rts=False, timeout=0.2):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = PORT, BAUD, timeout
    s.dtr, s.rts = dtr, rts
    s.open()
    time.sleep(0.3)
    return s


def pulse(s):
    k32.EscapeCommFunction(s._port_handle, SETRTS)
    time.sleep(0.30)
    k32.EscapeCommFunction(s._port_handle, CLRRTS)
    time.sleep(0.30)


def sync(s, tries=3, flush=True):
    for i in range(tries):
        try:
            if flush:
                s.reset_input_buffer()
            esp = ESP32ROM(s, BAUD)
            esp.sync()
            return "SYNC OK mac=" + ":".join("%02X" % b for b in esp.read_mac())
        except Exception as e:
            last = str(e).splitlines()[0][:46]
            time.sleep(0.15)
    return "no rom (%s)" % last


def dump(tag, s, n=300):
    txt = b""
    t0 = time.time()
    while time.time() - t0 < 1.2:
        b = s.read(8192)
        if b:
            txt += b
        elif txt:
            break
    print("  %s  %d bytes" % (tag, len(txt)), flush=True)
    print("    head: " + repr(txt[:96]), flush=True)
    printable = "".join(chr(c) if 32 <= c < 127 else "." for c in txt[:n])
    for i in range(0, len(printable), 96):
        print("    | " + printable[i:i + 96], flush=True)


print("=== 1. sync immediately after reset, no pre-drain ===", flush=True)
for tag, kw in [("dtr=1 at open + RTS pulse", {"dtr": True, "rts": False}),
                ("dtr=0 at open + RTS pulse (control)", {"dtr": False, "rts": False})]:
    s = opened(**kw)
    pulse(s)
    print("  %-34s %s" % (tag, sync(s)), flush=True)
    dump("    after:", s)
    s.close()
    time.sleep(0.7)

print("=== 2. what the DTR state actually emits, start to finish ===", flush=True)
s = opened(dtr=True)
pulse(s)
dump("  dtr=1, post-pulse", s, 600)
s.close()
time.sleep(0.7)

print("=== 3. control ===", flush=True)
s = opened(dtr=False)
pulse(s)
dump("  dtr=0, post-pulse", s, 600)
s.close()
