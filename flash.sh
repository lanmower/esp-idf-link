#!/bin/bash
set -e

DEVICE="${1:-}"

if [ ! -f "build/bootloader/bootloader.bin" ]; then
    echo "Error: Bootloader binary not found. Run ./build.sh first"
    exit 1
fi

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

echo "Flashing to $DEVICE..."
sudo bash -c "PYTHONPATH=/home/user/.local/lib/python3.12/site-packages:\$PYTHONPATH python3.12 -m esptool \
    --chip esp32 --port '$DEVICE' -b 460800 \
    --before default-reset --after hard-reset \
    write-flash \
    --flash-mode dio --flash-size 4MB --flash-freq 40m \
    0x1000 build/bootloader/bootloader.bin \
    0x8000 build/partition_table/partition-table.bin \
    0x20000 build/link-idf-example.bin"

echo ""
echo "✓ Flash complete!"
