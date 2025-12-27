#!/bin/bash

DEVICE="${1:-/dev/ttyUSB0}"

if [ ! -e "$DEVICE" ]; then
    echo "Error: Device $DEVICE not found"
    exit 1
fi

echo "Erasing NVS (WiFi credentials and settings)..."
python -m esptool --chip esp32 --port "$DEVICE" erase-region 0x9000 0x6000

echo ""
echo "✓ NVS erased! Device will now enter provisioning mode on next boot."
echo ""
echo "Waiting for device to reset..."
sleep 2
