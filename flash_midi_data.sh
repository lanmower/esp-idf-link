#!/bin/bash

DEVICE="${1:-/dev/ttyUSB0}"

echo "Creating SPIFFS image with MIDI files from the data directory..."

if [ ! -d "data" ]; then
  echo "Error: data directory not found"
  exit 1
fi

echo "Building SPIFFS image..."
python $IDF_PATH/components/spiffs/spiffsgen.py 0x19000 data midi_data.bin

if [ ! -f "midi_data.bin" ]; then
  echo "Error: SPIFFS image creation failed"
  exit 1
fi

echo "Flashing SPIFFS image to ESP32..."
sudo PYTHONPATH=/home/user/.local/lib/python3.12/site-packages python3.12 /home/user/.local/bin/esptool \
    --chip esp32 --port "$DEVICE" --baud 460800 \
    --before default-reset --after hard-reset \
    write-flash --flash-size 4MB 0x317000 midi_data.bin

echo "MIDI files flashed successfully. Reset the ESP32 to use the new files." 