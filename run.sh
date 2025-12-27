#!/bin/bash
docker run --rm -v $(pwd):/project -w /project --device=/dev/ttyUSB0:/dev/ttyUSB0 -it espressif/idf:latest /bin/bash