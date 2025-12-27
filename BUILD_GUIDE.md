# ESP-IDF Link Build Guide

## Quick Start

### Prerequisites

- Docker installed and running
- USB device connected for flashing (optional, only for flashing to hardware)

### Initial Setup (First Time Only)

```bash
bash setup.sh
```

This script will:
- Check for Docker installation
- Build the Docker image with all ESP-IDF dependencies pre-configured
- Initialize all git submodules

### Building the Project

```bash
bash build.sh
```

Build artifacts will be created in `build/` directory.

## Using Docker Directly

### Build with bash scripts (recommended)

```bash
bash setup.sh   # One time
bash build.sh   # Every build
```

### Build with docker-compose

```bash
# Build the project
docker-compose run --rm build

# Interactive shell
docker-compose run --rm shell

# Flash to device (requires USB at /dev/ttyUSB0)
docker-compose run --rm flash

# Monitor serial output
docker-compose run --rm monitor
```

### Build with raw docker command

```bash
# Build Docker image
docker build -t esp-idf-link .

# Run build
docker run --rm \
  -v $(pwd):/project \
  -w /project \
  esp-idf-link \
  bash -c "source /opt/esp/idf/export.sh && idf.py build"

# Flash to device
docker run --rm \
  --device=/dev/ttyUSB0 \
  -v $(pwd):/project \
  -w /project \
  esp-idf-link \
  bash -c "source /opt/esp/idf/export.sh && idf.py -p /dev/ttyUSB0 flash"
```

## Project Structure

```
.
├── main/                    # Main application source code
├── components/              # ESP-IDF components
│   ├── link-esp/           # Ableton Link integration
│   └── ...
├── Dockerfile              # Docker build configuration
├── docker-compose.yml      # Docker Compose configuration
├── CMakeLists.txt          # Main project CMake config
├── setup.sh                # Docker image build script
├── build.sh                # Build script
└── data/                   # SPIFFS filesystem data
```

## Key Files

### Main Application Files
- `main/main.cpp` - Main entry point
- `main/io_helpers.cpp` - Hardware I/O utilities (ADC, touch, buzzer)
- `main/midi_file.cpp` - MIDI file loading
- `main/network_midi.cpp` - HTTP server for MIDI uploads
- `main/wifi_config.cpp` - WiFi provisioning
- `main/effect_*.cpp` - Audio effects (arp, filter, sidechain)
- `main/synth_*.cpp` - Synthesizer implementations

### Docker Configuration
- `Dockerfile` - Builds image using Espressif official ESP-IDF image
- `docker-compose.yml` - Defines build, flash, and monitor services

## Advantages of Docker Approach

1. **No Local Dependencies**: Everything runs in the container
2. **Consistent Environment**: Same build environment on all machines (Linux, Mac, WSL2)
3. **No Permission Issues**: Avoids Windows filesystem permission problems on WSL2
4. **Pre-built Binaries**: Official Espressif image includes all WiFi binaries
5. **Easy Cleanup**: Just delete the image, no leftover files
6. **Cross-platform**: Works identically on Windows, Mac, and Linux

## Known Issues & Fixes Applied

### 1. esp_netif_next API Compatibility (FIXED)
**Issue**: ESP-IDF 6.1 renamed `esp_netif_next()` to `esp_netif_next_unsafe()`
**File**: `components/link-esp/link/include/ableton/platforms/esp32/ScanIpIfAddrs.hpp`
**Fix**: Updated API calls to use `esp_netif_next_unsafe()`

### 2. Legacy ADC Driver Removal (FIXED)
**Issue**: Old `driver/adc.h` was removed in ESP-IDF 6.1
**File**: `main/io_helpers.cpp`
**Fix**: Removed legacy include, added `hall_sensor_read()` stub function

### 3. Control Flow Goto Issue (FIXED)
**Issue**: goto statement crossed variable initialization boundary
**File**: `main/main.cpp`
**Fix**: Wrapped provisioning_mode block in braces to create new scope

### 4. Buffer Overflow Warning (FIXED)
**Issue**: snprintf buffer size could be exceeded with long filenames
**File**: `main/network_midi.cpp`
**Fix**: Increased filepath buffer size from 256 to 512 bytes

## Building Manually

If you prefer not to use the provided scripts:

```bash
# Build Docker image
docker build -t esp-idf-link .

# Build project
docker run --rm -v $(pwd):/project -w /project esp-idf-link bash -c \
  "source /opt/esp/idf/export.sh && idf.py build"

# Flash to device (replace /dev/ttyUSB0 with your port)
docker run --rm --device=/dev/ttyUSB0 -v $(pwd):/project -w /project esp-idf-link bash -c \
  "source /opt/esp/idf/export.sh && idf.py -p /dev/ttyUSB0 flash"

# Monitor
docker run --rm -it --device=/dev/ttyUSB0 -v $(pwd):/project -w /project esp-idf-link bash -c \
  "source /opt/esp/idf/export.sh && idf.py -p /dev/ttyUSB0 monitor"
```

## Docker Image Details

The Docker image is built from `espressif/idf:v6.1-full` which includes:
- ESP-IDF v6.1 framework
- All required toolchains (xtensa-esp-elf, etc.)
- All ESP-IDF components
- Pre-built WiFi binaries for all ESP32 variants
- Python environment with all tools
- Git for submodule operations

## Customization

### Build Directory
Default: `build/`

To use a different directory:
```bash
BUILD_DIR=/custom/path bash build.sh
```

### Docker Image Name
Default: `esp-idf-link:latest`

To change, edit the `IMAGE_NAME` and `IMAGE_TAG` variables in `setup.sh` and `build.sh`.

## Troubleshooting

### Docker Not Found
```bash
# Install Docker from https://www.docker.com/
```

### Docker Daemon Not Running
```bash
# Start Docker (depends on your system)
# Linux: sudo systemctl start docker
# Mac: Open Docker Desktop
# Windows: Open Docker Desktop
```

### Permission Denied on /dev/ttyUSB0
```bash
# Add user to dialout group (Linux only)
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

### Build Fails Inside Container
```bash
# Pull latest ESP-IDF image
docker pull espressif/idf:v6.1-full

# Rebuild the image
docker build --no-cache -t esp-idf-link .
```

### Slow Docker on WSL2
WSL2 Docker performance accessing Windows filesystem is slower. To improve:
1. Keep project source in Linux filesystem (`/home/user/...` not `/mnt/c/...`)
2. Or use Docker Desktop's native WSL2 integration

## Compilation Configuration

The project uses the following optimization flags:
- `-Os` - Size optimization
- `-ffunction-sections` - Function-level sectioning
- `-fdata-sections` - Data-level sectioning
- `-Wl,--gc-sections` - Garbage collect unused sections

C++ Standard: C++17

## Component Dependencies

### Main Dependencies
- `nvs_flash` - Non-volatile storage
- `esp_netif` - Network interface
- `esp_event` - Event loop
- `esp_wifi` - WiFi support
- `driver` - GPIO, ADC, UART, LEDC drivers
- `freertos` - Real-time OS
- `esp_http_server` - HTTP server
- `esp_driver_gptimer` - General purpose timers
- `esp_adc` - ADC conversion

### Third-party
- `link-esp` - Ableton Link (from git submodule)
- `protocol_examples_common` - Common protocol examples

## Testing After Build

The binary can be flashed to an ESP32 using:

```bash
# Using bash script
bash build.sh  # Builds in build/ directory

# Using docker-compose
docker-compose run --rm flash

# Using raw docker
docker run --rm --device=/dev/ttyUSB0 -v $(pwd):/project -w /project esp-idf-link bash -c \
  "source /opt/esp/idf/export.sh && idf.py -p /dev/ttyUSB0 flash"
```

Expected output on serial monitor:
- System initialization messages
- WiFi connection attempts or provisioning mode
- SPIFFS filesystem initialization
- Link sync and MIDI processing ready

## Build Output Locations

- **Application Binary**: `build/esp-idf/main/link-idf-example.bin`
- **Bootloader**: `build/esp-idf/esp32/bootloader/bootloader.bin`
- **Partition Table**: `build/esp-idf/esp32/partitions.bin`
- **Build Logs**: `build/log/`

## Clean Build

```bash
# Remove build artifacts
rm -rf build/

# Rebuild
bash build.sh
```

## Additional Resources

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [Espressif Docker Images](https://hub.docker.com/r/espressif/idf)
- [ESP32 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
- [Ableton Link Documentation](https://github.com/Ableton/link)
- [Docker Documentation](https://docs.docker.com/)
