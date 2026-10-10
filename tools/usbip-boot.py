import socket, struct, sys, time, serial
from esptool.targets.esp32 import ESP32ROM

HOST, PORT = "127.0.0.1", 3240
BUSID = "2-2"
COMPORT = sys.argv[1] if len(sys.argv) > 1 else "COM12"
REQ = 0xA4
BIT_RTS, BIT_DTR = 1 << 5, 1 << 6


def connect():
    s = socket.create_connection((HOST, PORT), timeout=8)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return s


def recv(s, n):
    buf = b""
    while len(buf) < n:
        chunk = s.recv(n - len(buf))
        if not chunk:
            raise EOFError("closed at %d/%d" % (len(buf), n))
        buf += chunk
    return buf


def import_dev(s):
    s.sendall(struct.pack(">HHI", 0x0111, 0x8003, 0) + BUSID.encode().ljust(32, b"\0"))
    ver, cmd, status = struct.unpack(">HHI", recv(s, 8))
    if status:
        raise OSError("import status=%d" % status)
    d = recv(s, 312)
    busnum, devnum = struct.unpack(">II", d[288:296])
    return (busnum << 16) | devnum


def hs(s, devid, seq, control):
    setup = struct.pack("<BBHHH", 0x40, REQ, ~control & 0xFF, 0, 0)
    s.sendall(struct.pack(">IIIIIIIIII", 1, seq, devid, 0, 0, 0, 0, 0, 0, 0) + setup)
    return struct.unpack(">IIIIIIIIII", recv(s, 48)[:40])[5]


def read_rom(sec=12.0):
    try:
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = COMPORT, 115200, 0.4
        s.dtr, s.rts = False, False
        s.open()
    except Exception as e:
        return "NO PORT (%s)" % str(e)[:34], ""
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < sec:
        b = s.read(8192)
        if b:
            buf.extend(b)
        elif buf:
            break
    txt = bytes(buf).decode("latin1").replace("\r", "")
    verdict = "silence"
    for line in txt.split("\n"):
        if "waiting for download" in line:
            verdict = "WAITING-FOR-DOWNLOAD"
            break
        if "boot:" in line:
            verdict = line.strip()[:46]
            break
    rom = ""
    if verdict in ("WAITING-FOR-DOWNLOAD",) or "0x3" in verdict:
        try:
            s.reset_input_buffer()
            esp = ESP32ROM(s, 115200)
            esp.sync()
            rom = "SYNC OK mac=" + ":".join("%02X" % b for b in esp.read_mac())
        except Exception as e:
            rom = "sync fail (%s)" % str(e).splitlines()[0][:36]
    s.close()
    return verdict, rom


def trial(hold, release):
    tag = "hold=0x%02x rel=0x%02x" % (hold, release)
    try:
        s = connect()
        devid = import_dev(s)
        hs(s, devid, 1, hold)
        time.sleep(0.30)
        hs(s, devid, 2, release)
        time.sleep(0.60)
        hs(s, devid, 3, release)
        time.sleep(0.40)
        s.close()
    except Exception as e:
        print("  %-22s ERROR %s" % (tag, str(e)[:60]), flush=True)
        return
    time.sleep(1.2)
    verdict, rom = read_rom()
    print("  %-22s %-46s %s" % (tag, verdict, rom), flush=True)
    time.sleep(0.6)


print("=== sweep: hold EN, release EN, keep IO0 ===", flush=True)
for hold in (0x00, 0x20, 0x40, 0x60):
    trial(hold, hold ^ BIT_RTS)
print("=== sweep: flip DTR instead of RTS (in case the nets are swapped) ===", flush=True)
for hold in (0x00, 0x20, 0x40, 0x60):
    trial(hold, hold ^ BIT_DTR)
