#!/bin/bash

export IDF_PATH='/opt/esp/idf' 
'/opt/esp/python_env/idf5.4_py3.12_env/bin/python' '/opt/esp/idf/tools/idf_monitor.py' \
  -p /dev/ttyUSB0 \
  -b 115200 \
  --toolchain-prefix xtensa-esp32-elf- \
  --make ''/opt/esp/python_env/idf5.4_py3.12_env/bin/python' '/opt/esp/idf/tools/idf.py'' \
  --target esp32 \
  '/workspaces/esp-idf-link/build/link-idf-example.elf'
