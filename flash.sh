#!/bin/bash
set -e

DEVICE="${1:-}"

if [ ! -f "build/bootloader/bootloader.bin" ]; then
    echo "Error: Bootloader binary not found. Run ./build.sh first"
    exit 1
fi

if [ ! -f "build/partition_table/partition-table.bin" ]; then
    echo "Error: Partition table not found. Run ./build.sh first"
    exit 1
fi

app_offset() {
    python3.12 - "build/partition_table/partition-table.bin" <<'PY'
import struct, sys
ENTRY_BYTES, MAGIC, TYPE_APP = 32, 0x50AA, 0
table = open(sys.argv[1], "rb").read()
for i in range(0, len(table) - ENTRY_BYTES + 1, ENTRY_BYTES):
    magic, ptype = struct.unpack("<HB", table[i:i + 3])
    if magic != MAGIC: break
    if ptype == TYPE_APP:
        print("0x%x" % struct.unpack("<I", table[i + 4:i + 8])[0])
        sys.exit(0)
sys.exit("no app partition in %s" % sys.argv[1])
PY
}

APP_OFFSET=$(app_offset)

find_device() {
    for port in /dev/ttyUSB* /dev/ttyACM* /dev/ttyS*; do
        if [ -e "$port" ]; then
            echo "$port"
            return 0
        fi
    done
    return 1
}

if [ -z "$DEVICE" ]; then
    if DEVICE=$(find_device); then
        echo "Auto-detected device: $DEVICE"
    else
        echo "Error: No serial device found"
        echo ""
        echo "WSL2 + usbipd setup required:"
        echo "1. On Windows (PowerShell): usbipd bind -b <busid>"
        echo "2. On Windows (PowerShell): usbipd attach -b <busid> -w"
        echo "3. In WSL2: sudo bash setup-ch340.sh"
        echo ""
        echo "Available serial devices:"
        ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  (none found)"
        echo ""
        echo "Usage: $0 [device]"
        echo "Example: $0 /dev/ttyUSB0"
        exit 1
    fi
fi

if [ ! -e "$DEVICE" ]; then
    echo "Error: Device $DEVICE not found"
    exit 1
fi

echo "Flashing to $DEVICE (app partition at $APP_OFFSET)..."
sudo PYTHONPATH=/home/user/.local/lib/python3.12/site-packages python3.12 /home/user/.local/bin/esptool \
    --chip esp32 --port "$DEVICE" -b 460800 \
    --before default-reset --after hard-reset \
    write-flash --flash-mode dio --flash-size 4MB --flash-freq 40m \
    0x1000 build/bootloader/bootloader.bin \
    0x8000 build/partition_table/partition-table.bin \
    $APP_OFFSET build/link-idf-example.bin

echo ""
echo "[OK] Flash complete!"
