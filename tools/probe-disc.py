import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200


def opened(timeout=0.1):
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = timeout
    s.dtr = False
    s.rts = False
    s.open()
    return s


def steps(s, script, wait=3.0):
    out = bytearray()
    for dtr, rts, d in script:
        if dtr is not None:
            s.setDTR(dtr)
        if rts is not None:
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
    return bytes(out).decode("latin1").replace("\r", "")


def run(tag, script, wait=3.0):
    s = opened()
    time.sleep(0.15)
    txt = steps(s, script, wait)
    print("=== %-34s %5d bytes" % (tag, len(txt)), flush=True)
    for line in txt.split("\n")[:4]:
        print("    " + line[:100])
    print(flush=True)
    try:
        s.setDTR(False)
        s.setRTS(False)
    except Exception:
        pass
    s.close()


T = [
    ("C control: rts pulse only", [(False, True, 0.30), (False, False, 3.0)]),
    ("A dtr held, rts pulse, release dtr",
     [(True, None, 0.50), (None, True, 0.30), (None, False, 0.50), (False, None, 3.0)]),
    ("B sustained dtr then release",
     [(True, None, 0.60), (False, None, 3.0)]),
    ("D dtr held long, rts pulse, rel dtr",
     [(True, None, 1.00), (None, True, 0.30), (None, False, 1.00), (False, None, 3.0)]),
]

if __name__ == "__main__":
    for tag, script in T:
        run(tag, script)
        time.sleep(1.0)
