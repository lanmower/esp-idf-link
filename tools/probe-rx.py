import sys, time, serial
from esptool.targets.esp32 import ESP32ROM

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
BAUD = 115200


def open_with(dtr, rts, timeout=0.2):
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = timeout
    s.dtr = dtr
    s.rts = rts
    s.open()
    return s


def raw(s, sec):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        b = s.read(4096)
        if b:
            buf.extend(b)
    return bytes(buf)


def enter(s):
    s.setDTR(True)
    s.setRTS(True)
    s.setDTR(True)
    time.sleep(0.20)
    s.setRTS(False)
    s.setDTR(True)
    time.sleep(0.10)


def try_sync(s, tag):
    try:
        esp = ESP32ROM(s, BAUD)
        esp.sync()
        mac = ":".join("%02X" % b for b in esp.read_mac())
        print("%-24s SYNC OK  mac=%s" % (tag, mac), flush=True)
        return True
    except Exception as e:
        print("%-24s %s" % (tag, str(e).splitlines()[0][:70]), flush=True)
        return False


def main():
    s = open_with(False, False)
    time.sleep(0.2)
    print("baseline app bytes:", len(raw(s, 1.5)), flush=True)
    enter(s)
    print("after enter bytes :", len(raw(s, 2.0)), flush=True)
    try_sync(s, "sync same handle")
    print("sync residue      :", raw(s, 2.0)[:60].hex(), flush=True)
    s.close()

    time.sleep(0.3)
    s2 = open_with(True, False)
    time.sleep(0.3)
    print("reopen(D1,R0) bytes:", len(raw(s2, 2.0)), flush=True)
    try_sync(s2, "sync reopen D1,R0")
    print("reopen residue      :", raw(s2, 2.0)[:60].hex(), flush=True)
    s2.close()

    time.sleep(0.3)
    s3 = open_with(False, False)
    time.sleep(0.3)
    print("reopen(D0,R0) bytes:", len(raw(s3, 2.0)), flush=True)
    try_sync(s3, "sync reopen D0,R0")
    s3.close()


if __name__ == "__main__":
    main()
