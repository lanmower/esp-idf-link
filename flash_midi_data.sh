#!/bin/bash

SPIFFS_IMAGE_SIZE=0x19000
SPIFFS_PARTITION_OFFSET=0x317000
DEVICE="${1:-/dev/ttyUSB0}"

echo "Creating SPIFFS image with MIDI files from the data directory..."

if [ ! -d "data" ]; then
  echo "Error: data directory not found"
  exit 1
fi

echo "Building SPIFFS image..."
python $IDF_PATH/components/spiffs/spiffsgen.py $SPIFFS_IMAGE_SIZE data midi_data.bin

if [ ! -f "midi_data.bin" ]; then
  echo "Error: SPIFFS image creation failed"
  exit 1
fi

echo "Flashing SPIFFS image to ESP32..."
python -m esptool --chip esp32 --port "$DEVICE" --baud 460800 --before default_reset --after hard_reset write_flash $SPIFFS_PARTITION_OFFSET midi_data.bin

echo "MIDI files flashed successfully. Reset the ESP32 to use the new files." 