import sys, time, serial
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200


def op(dtr, rts):
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = 0.05
    s.dtr = dtr
    s.rts = rts
    s.open()
    return s


def sync(s, tag):
    try:
        esp = ESP32ROM(s, BAUD)
        esp.connect("no-reset")
        mac = ":".join("%02X" % b for b in esp.read_mac())
        print("%-30s DOWNLOAD  mac=%s" % (tag, mac), flush=True)
        return True
    except Exception as e:
        print("%-30s fail: %s" % (tag, str(e).splitlines()[0][:66]), flush=True)
        return False


def handle(tag, steps):
    s = op(False, False)
    time.sleep(0.05)
    try:
        for dtr, rts, d in steps:
            s.setDTR(dtr)
            s.setRTS(rts)
            time.sleep(d)
        return sync(s, tag)
    finally:
        s.close()


def reopen(tag, states):
    s = None
    try:
        for dtr, rts, d in states:
            if s:
                s.close()
            s = op(dtr, rts)
            time.sleep(d)
        return sync(s, tag)
    finally:
        if s:
            s.close()


HS = [
    ("B esptool-default", [(False, True, .15), (True, False, .05), (False, False, 0)]),
    ("C both->relRTS", [(True, True, .15), (True, False, 0)]),
    ("D both->relRTS->relALL", [(True, True, .30), (True, False, .05), (False, False, 0)]),
    ("E reset->io0low->relRTS", [(False, True, .15), (True, True, .05), (True, False, 0)]),
    ("F reset->relALL (ctl)", [(False, True, .15), (False, False, 0)]),
    ("G both->relDTR", [(True, True, .15), (False, True, 0)]),
    ("H io0low->reset->relRTS", [(True, False, .15), (True, True, .15), (True, False, 0)]),
    ("I io0low->reset->relALL", [(True, False, .15), (True, True, .15), (False, False, 0)]),
]
RS = [
    ("J reopen T,T->T,F", [(True, True, .15), (True, False, 0)]),
    ("K reopen F,T->T,F", [(False, True, .15), (True, False, 0)]),
    ("L reopen T,T->T,F .30", [(True, True, .30), (True, False, 0)]),
]

if __name__ == "__main__":
    handle("A ctl open-released", [])
    time.sleep(0.4)
    for t, st in HS:
        handle(t, st)
        time.sleep(0.4)
    for t, st in RS:
        reopen(t, st)
        time.sleep(0.4)
