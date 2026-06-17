#!/bin/bash
set -e

echo "Setting up CH340 USB-Serial driver for ESP32 in WSL2..."
echo ""

if [ "$EUID" -ne 0 ]; then
    echo "This script must be run with sudo"
    exit 1
fi

echo "Loading kernel modules (usbserial, ch341)..."
modprobe usbserial || echo "WARNING: usbserial module load failed (may be expected)"
modprobe ch341 || echo "WARNING: ch341 module load failed"

echo "[OK] Kernel modules loaded"

sleep 2

if ls /dev/ttyUSB* 2>/dev/null | grep -q .; then
    echo "[OK] Serial device(s) available:"
    ls -la /dev/ttyUSB*

    echo ""
    echo "To make this permanent, add to /etc/modprobe.d/wsl-ch340.conf:"
    echo "  install usbserial /sbin/modprobe --ignore-install usbserial && /sbin/modprobe ch341"
    echo ""
    exit 0
fi

echo ""
echo "No serial device found yet. Checking device detection..."

if dmesg | tail -50 | grep -i "1a86.*7523\|ch340\|ch341" > /dev/null; then
    echo "[OK] CH340/CH341 device detected in kernel"
    echo ""
    echo "Device was found but no /dev/ttyUSB* appeared. Options:"
    echo "  1. Reattach via usbipd on Windows (PowerShell):"
    echo "       usbipd detach -b <busid>"
    echo "       usbipd attach -b <busid> -w"
    echo "  2. Re-run this script after reattachment"
else
    echo "WARNING: CH340/CH341 device not detected"
    echo ""
    echo "Ensure device is attached via usbipd on Windows (PowerShell):"
    echo "  1. Find device: usbipd list"
    echo "  2. Attach device: usbipd attach -b <busid> -w"
fi

echo ""
echo "Setup complete!"
