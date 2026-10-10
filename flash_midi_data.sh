#!/bin/bash

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
python -m esptool --chip esp32 --port /dev/ttyUSB0 --baud 460800 --before default_reset --after hard_reset write_flash 0x317000 midi_data.bin

echo "MIDI files flashed successfully. Reset the ESP32 to use the new files." 