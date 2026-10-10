"""Does DTR reach GPIO0? Immune to the 'DTR kills this handle's RX' confound:
apply the sequence, close the handle, reopen clean, then ask the ROM to answer.
In download mode the ROM answers sync forever; the running app just prints.
"""
import sys, time, serial
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200


def opened(dtr=False, rts=False, timeout=0.1):
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = timeout
    s.dtr = dtr
    s.rts = rts
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


def attempt(tag, script):
    s = opened()
    time.sleep(0.2)
    for dtr, rts, d in script:
        s.setDTR(dtr)
        s.setRTS(rts)
        time.sleep(d)
    s.close()

    s = opened()
    time.sleep(0.25)
    txt = read(s, 2.0).decode("latin1").replace("\r", "")
    verdict = "-"
    try:
        esp = ESP32ROM(s, BAUD)
        esp.sync()
        mac = ":".join("%02X" % b for b in esp.read_mac())
        verdict = "ROM DOWNLOAD MODE  mac=%s  <== WIN" % mac
    except Exception as e:
        verdict = "not in download mode (%s)" % str(e).splitlines()[0][:40]
    print("%-32s %4d bytes  %s" % (tag, len(txt), verdict), flush=True)
    if "waiting for download" in txt:
        print("    ROM banner seen", flush=True)
    s.close()


T = [
    ("1 ctl rts pulse", [(False, True, 0.30), (False, False, 0.05)]),
    ("2 dtr hold + rts pulse", [(True, False, 0.15), (True, True, 0.30), (True, False, 0.05)]),
    ("3 both held rel rts", [(True, True, 0.30), (True, False, 0.05)]),
    ("4 dtr held long + rts pulse", [(True, False, 0.60), (True, True, 0.30), (True, False, 0.20)]),
    ("5 esptool classic", [(False, True, 0.15), (True, False, 0.05), (False, False, 0.05)]),
    ("6 rts hold, rel dtr last", [(True, False, 0.05), (True, True, 0.30), (False, True, 0.05), (False, False, 0.05)]),
]

if __name__ == "__main__":
    for tag, script in T:
        attempt(tag, script)
        time.sleep(0.8)
