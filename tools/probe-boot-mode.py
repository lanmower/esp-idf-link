import serial, time, re, sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200


def read_banner(s, secs=3.0):
    end = time.time() + secs
    buf = b""
    while time.time() < end:
        d = s.read(4096)
        if d:
            buf += d
        else:
            time.sleep(0.01)
    return buf.decode("latin1")


def verdict(tag, txt):
    m = re.search(r"boot:0x([0-9a-f]+)", txt)
    if m:
        v = int(m.group(1), 16)
        print("%-40s boot=0x%-3s %s" % (tag, m.group(1),
              "DOWNLOAD_BOOT ***" if v & 1 == 0 else "flash_boot"))
    else:
        print("%-40s no banner (%d bytes)" % (tag, len(txt)))
    return txt


def fresh(dtr, rts):
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = 0.05
    s.dtr = dtr
    s.rts = rts
    s.open()
    return s


def c_none(s):
    pass


def c_modem(s):
    s.getCD()


def c_write(s):
    s.write(b"")


def c_purge(s):
    s.reset_input_buffer()


def c_state(s):
    s._reconfigure_port()


COMMITS = [("none", c_none), ("modemstatus", c_modem), ("write0", c_write),
           ("purge", c_purge), ("reconfigure", c_state)]

DEFAULT = [(False, True, 0.10), (True, False, 0.05), (False, False, 0)]


def with_commit(tag, steps, commit):
    s = serial.Serial(PORT, BAUD, timeout=0.05, dsrdtr=False, rtscts=False)
    time.sleep(0.05)
    for dtr, rts, d in steps:
        s.setDTR(dtr)
        commit(s)
        s.setRTS(rts)
        commit(s)
        time.sleep(d)
    verdict(tag, read_banner(s))
    s.close()


def reopen_seq(tag, states):
    s = None
    for dtr, rts, d in states:
        if s:
            s.close()
        s = fresh(dtr, rts)
        time.sleep(d)
    txt = read_banner(s)
    verdict(tag, txt)
    s.close()
    return txt


print("=== A: commit action after each line change (esptool default seq) ===")
for name, fn in COMMITS:
    with_commit("A commit=%s" % name, DEFAULT, fn)

print("=== B: line state applied at port OPEN (close/reopen) ===")
reopen_seq("B1 reset -> io0low+en-release", [(False, True, .15), (True, False, 0)])
reopen_seq("B2 reset -> all released (ctl)", [(False, True, .15), (False, False, 0)])
reopen_seq("B3 io0low -> reset -> en-release", [(True, False, .15), (True, True, .15), (True, False, 0)])
reopen_seq("B4 both-assert -> en-release", [(True, True, .15), (True, False, 0)])
reopen_seq("B5 io0low -> reset -> all rel", [(True, False, .15), (True, True, .15), (False, False, 0)])
