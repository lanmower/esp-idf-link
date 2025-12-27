#!/bin/bash

DEVICE="${1:-/dev/ttyUSB0}"
BAUD="${2:-115200}"

if [ ! -e "$DEVICE" ]; then
    echo "Device $DEVICE not found, attempting to reload USB driver..."
    sudo modprobe -r usbserial 2>/dev/null
    sleep 1
    sudo modprobe ch341 2>/dev/null
    sleep 2
    
    if [ ! -e "$DEVICE" ]; then
        echo "✗ Error: Device still not found after driver reload"
        echo ""
        echo "Available serial devices:"
        ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  (none found)"
        echo ""
        echo "Usage: $0 [device] [baud]"
        echo "Example: $0 /dev/ttyUSB0 115200"
        exit 1
    fi
fi

sudo chmod 666 "$DEVICE"
echo "Monitoring $DEVICE at $BAUD baud (Ctrl+C to exit)..."
stty -F "$DEVICE" "$BAUD" raw -echo
cat "$DEVICE"
