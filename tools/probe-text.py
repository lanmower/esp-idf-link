"""Print the real boot text after each DTR/RTS sequence.

Byte counts were ambiguous (the app logs in bursts). 'waiting for download' is
the only unambiguous proof the ROM bootloader is running.
"""
import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200


def opened():
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = 0.1
    s.dtr = False
    s.rts = False
    s.open()
    return s


def run(tag, steps, wait=3.0):
    s = opened()
    time.sleep(0.15)
    s.reset_input_buffer()
    out = bytearray()
    for dtr, rts, d in steps:
        s.setDTR(dtr)
        s.setRTS(rts)
        t0 = time.time()
        while time.time() - t0 < d:
            b = s.read(8192)
            if b:
                out.extend(b)
    t0 = time.time()
    while time.time() - t0 < wait:
        b = s.read(8192)
        if b:
            out.extend(b)
    txt = bytes(out).decode("latin1").replace("\r", "")
    verdict = "DOWNLOAD <== WIN" if "waiting for download" in txt else "-"
    print("=== %-26s %5d bytes  %s" % (tag, len(out), verdict), flush=True)
    for line in txt.split("\n")[:8]:
        print("    " + line[:110])
    print(flush=True)
    try:
        s.setDTR(False)
        s.setRTS(False)
    except Exception:
        pass
    s.close()


T = [
    ("1 ctl rts pulse", [(False, True, 0.30), (False, False, 0.0)]),
    ("2 dtr only", [(True, False, 0.30), (False, False, 0.0)]),
    ("3 dtr hold + rts pulse", [(True, False, 0.10), (True, True, 0.30), (True, False, 0.0)]),
    ("4 both held rel rts", [(True, True, 0.30), (True, False, 0.0)]),
    ("5 esptool classic", [(False, True, 0.15), (True, False, 0.05), (False, False, 0.0)]),
    ("6 rts pulse then dtr low", [(True, False, 0.05), (True, True, 0.30), (True, False, 0.0)]),
]

if __name__ == "__main__":
    for tag, steps in T:
        run(tag, steps)
        time.sleep(1.0)
