#!/bin/bash

set -e

echo "ESP-IDF Setup Script for WSL"
echo "============================="
echo ""

if [ -d "$HOME/esp/esp-idf" ]; then
    echo "ESP-IDF already exists at $HOME/esp/esp-idf"
    read -p "Do you want to reinstall? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Skipping ESP-IDF installation"
        exit 0
    fi
    rm -rf $HOME/esp/esp-idf
fi

echo "Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

echo ""
echo "Creating ESP directory..."
mkdir -p $HOME/esp
cd $HOME/esp

echo ""
echo "Cloning ESP-IDF (this may take a while)..."
git clone --recursive https://github.com/espressif/esp-idf.git

cd esp-idf

echo ""
echo "Installing ESP-IDF tools for ESP32..."
./install.sh esp32

echo ""
echo "Setting up shell environment..."
if ! grep -q "get_idf" "$HOME/.bashrc"; then
    echo "" >> $HOME/.bashrc
    echo "# ESP-IDF environment" >> $HOME/.bashrc
    echo "alias get_idf='. \$HOME/esp/esp-idf/export.sh'" >> $HOME/.bashrc
    echo "Added 'get_idf' alias to ~/.bashrc"
else
    echo "'get_idf' alias already exists in ~/.bashrc"
fi

echo ""
echo "Adding user to dialout group for serial port access..."
sudo usermod -a -G dialout $USER

echo ""
echo "============================="
echo "ESP-IDF Setup Complete!"
echo "============================="
echo ""
echo "IMPORTANT: You need to log out and log back in for group changes to take effect."
echo ""
echo "Next steps:"
echo "1. Log out and log back in (for dialout group)"
echo "2. In each new terminal, run: get_idf"
echo "3. Then you can use: idf.py build, idf.py flash, etc."
echo ""
echo "For USB device access from Windows, install usbipd-win:"
echo "  In PowerShell (as Admin): winget install --interactive --exact dorssel.usbipd-win"
echo "  Then: usbipd list"
echo "  Then: usbipd bind --busid <BUSID>"
echo "  Then: usbipd attach --wsl --busid <BUSID>"
echo ""
echo "To start using ESP-IDF now (in this terminal only):"
echo "  source $HOME/esp/esp-idf/export.sh"
echo ""
