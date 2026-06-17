#!/bin/bash

echo "Looking for ESP32 device..."
echo ""

DEVICE=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1)

if [ -z "$DEVICE" ]; then
    echo "[FAIL] No serial device found"
    echo ""
    echo "Make sure to:"
    echo "  1. Connect the ESP32 device via USB"
    echo "  2. In Windows PowerShell (Admin): usbipd list"
    echo "  3. Then: usbipd attach --busid <busid>"
    exit 1
fi

echo "[OK] Found device: $DEVICE"
echo ""
echo "Quick start:"
echo "  ./build.sh       # Build the firmware"
echo "  ./flash.sh       # Flash to device"
echo "  ./monitor.sh     # Monitor serial output"
