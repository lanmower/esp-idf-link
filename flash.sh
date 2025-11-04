#!/bin/bash
# Skip the mkspiffs step since we're creating MIDI files through C code
# mkspiffs -c data -b 4096 -p 256 -s 0x19000 spiffs.bin

# Build the project
idf.py build

# Flash everything
idf.py -p /dev/ttyUSB0 flash

# Flash just the SPIFFS partition (comment out the above line and uncomment this if you only want to update SPIFFS)
# idf.py -p /dev/ttyUSB0 --no-build flash-only spiffs.bin 0x500000
