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

### Build with docker-compose (do not use on WSL2)

`docker-compose.yml` still defines `build`, `shell`, `flash` and `monitor`, and they do work
on a plain Linux host. On this project's WSL2 + Docker Desktop setup they have been observed
to hang indefinitely (>30 min) when only source files changed, so use the equivalent
`docker run` forms below instead. Reach for Compose only when you are debugging Compose
itself.

```bash
# Build the project
bash build.sh

# Interactive shell
docker run --rm -it -v $(pwd):/project -w /project esp-idf-link bash

# Flash to device (requires USB at /dev/ttyUSB0)
docker run --rm --device=/dev/ttyUSB0 -v $(pwd):/project -w /project esp-idf-link \
  bash -c "source /opt/esp/idf/export.sh && idf.py -p /dev/ttyUSB0 flash"

# Monitor serial output
docker run --rm -it --device=/dev/ttyUSB0 -v $(pwd):/project -w /project esp-idf-link \
  bash -c "source /opt/esp/idf/export.sh && idf.py -p /dev/ttyUSB0 monitor"
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
├── docker-compose.yml      # Docker Compose configuration (hangs on WSL2 -- see Troubleshooting)
├── CMakeLists.txt          # Main project CMake config
├── setup.sh                # Docker image build script
├── build.sh                # Build script
└── data/                   # SPIFFS filesystem data
```

## Key Files

### Main Application Files
- `main/main.cpp` - Main entry point
- `main/io_helpers.cpp` - Hardware I/O utilities (ADC, touch, buzzer)
- `main/midi_file.cpp` - MIDI file loading -- not listed in `main/CMakeLists.txt` SRCS, so not compiled today
- `main/network_midi.cpp` - HTTP server for MIDI uploads
- `main/wifi_config.cpp` - WiFi mesh AP/STA host election and Link multicast relay
- `main/effect_*.cpp` - Audio effects (arp, filter, sidechain, handler) -- not listed in `main/CMakeLists.txt` SRCS, so not compiled today
- `main/synth_*.cpp` - Synthesizer implementations

### Docker Configuration
- `Dockerfile` - Builds image using Espressif official ESP-IDF image
- `docker-compose.yml` - Defines build, shell, flash and monitor services -- present, but unusable on this project's WSL2 setup (see Troubleshooting)

## Advantages of Docker Approach

1. **No Local Dependencies**: Everything runs in the container
2. **Consistent Environment**: Same build environment on all machines (Linux, Mac, WSL2)
3. **No Permission Issues**: Avoids Windows filesystem permission problems on WSL2
4. **Pre-built Binaries**: Official Espressif image includes all WiFi binaries
5. **Easy Cleanup**: Just delete the image, no leftover files
6. **Cross-platform**: Works identically on Windows, Mac, and Linux

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

The Docker image is built from `espressif/idf:latest`, an unpinned tag, so the framework version
is whatever that tag resolves to at build time. It includes:
- All required toolchains (xtensa-esp-elf, etc.)
- All ESP-IDF components
- Pre-built WiFi binaries for all ESP32 variants
- Python environment with all tools
- Git

## Customization

### Build Directory
Always `build/` -- `build.sh` hardcodes it and takes no directory argument; `clean` is its only one.

### Docker Image Name
Default: `esp-idf-link:latest`

`setup.sh` hardcodes `IMAGE_NAME`/`IMAGE_TAG`. `build.sh` reads them as
`${IMAGE_NAME:-esp-idf-link}` / `${IMAGE_TAG:-latest}`, so
`IMAGE_NAME=... IMAGE_TAG=... bash build.sh` overrides the tag for one build without
editing either script.

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
docker pull espressif/idf:latest

# Rebuild the image
docker build --no-cache -t esp-idf-link .
```

### Slow Docker on WSL2
WSL2 Docker performance accessing Windows filesystem is slower. To improve:
1. Keep project source in Linux filesystem (`/home/user/...` not `/mnt/c/...`)
2. Or use Docker Desktop's native WSL2 integration

### `docker-compose run --rm build` hangs
On this project's WSL2 + Docker Desktop setup, Compose builds have been observed to hang
indefinitely (>30 min) when only source files changed. Prefer `bash build.sh`, which runs
`docker run` directly; use the Compose `build` service only if you are debugging Compose itself.

## Compilation Configuration

Optimization is deliberately not uniform, and the two files that set it disagree:

- `sdkconfig.defaults` sets `CONFIG_COMPILER_OPTIMIZATION_PERF=y` and
  `CONFIG_COMPILER_OPTIMIZATION_SIZE=n`, so the framework-wide default is `-O2`
  (performance) -- **not** `-Os`.
- `main/CMakeLists.txt` then appends `-Os -ffunction-sections -fdata-sections` to
  `__idf_main` and `-Wl,--gc-sections` to its link step. Those land *after* the
  framework flag on the command line, and with GCC the last `-O` wins, so the
  application's own sources compile `-Os` while every other component builds `-O2`.

| Flag | Meaning | Set by |
|---|---|---|
| `-O2` | Performance -- framework-wide default | `sdkconfig.defaults` -> `CONFIG_COMPILER_OPTIMIZATION_PERF=y` |
| `-Os` | Size -- app sources only, overrides the above | `main/CMakeLists.txt` `target_compile_options(__idf_main PRIVATE -Os ...)` |
| `-ffunction-sections` | Function-level sectioning | `main/CMakeLists.txt` (also an ESP-IDF default) |
| `-fdata-sections` | Data-level sectioning | `main/CMakeLists.txt` (also an ESP-IDF default) |
| `-Wl,--gc-sections` | Garbage collect unused sections | `main/CMakeLists.txt` `target_link_options` |

C++ standard: `main/CMakeLists.txt` sets `CXX_STANDARD 17` on `__idf_main`, but the compile
line recorded in `build/compile_commands.json` carries `-std=gnu++26` and no `-std=gnu++17`,
so ESP-IDF's own default is what actually reaches the compiler. Treat 17 as the declared
intent, not as the built standard.

## Component Dependencies

### Main Dependencies
Declared in `main/CMakeLists.txt` as `REQUIRES`:
- `nvs_flash` - Non-volatile storage
- `esp_netif` - Network interface
- `esp_event` - Event loop
- `esp_wifi` - WiFi support
- `esp_http_server` - HTTP server
- `esp_eth` - Ethernet (listed in `REQUIRES`; no Ethernet code in `main/` today)
- `spiffs` - SPIFFS filesystem mounted at `/spiffs` (uploaded clips, MIDI files)
- `driver` - Legacy umbrella driver component, kept alongside the split `esp_driver_*` below
- `log` - Logging
- `esp_adc` - ADC conversion
- `esp_driver_gptimer` - General purpose timers
- `link-esp` - Ableton Link integration
- `protocol_examples_common` - Common protocol examples
- `freertos` - Real-time OS

Declared as `PRIV_REQUIRES` (private to `main`, not propagated to anything that depends on it):
- `esp_driver_touch_sens` - Touch pads
- `esp_driver_gpio` - GPIO
- `esp_driver_uart` - MIDI UART
- `esp_driver_ledc` - Buzzer PWM

### Local components
- `link-esp` - Ableton Link integration (`components/link-esp`)

### Third-party
- `protocol_examples_common` - Common protocol examples

## Testing After Build

The binary can be flashed to an ESP32 using:

```bash
# Using bash script
bash build.sh  # Builds in build/ directory

# Using raw docker (not docker-compose -- see Troubleshooting)
docker run --rm --device=/dev/ttyUSB0 -v $(pwd):/project -w /project esp-idf-link bash -c \
  "source /opt/esp/idf/export.sh && idf.py -p /dev/ttyUSB0 flash"
```

Expected output on serial monitor:
- System initialization messages
- WiFi mesh host-election or station-join attempts
- SPIFFS filesystem initialization
- Link sync and MIDI processing ready

## Build Output Locations

- **Application Binary**: `build/link-idf-example.bin`
- **Bootloader**: `build/bootloader/bootloader.bin`
- **Partition Table**: `build/partition_table/partition-table.bin`

You do not have to build to get these three. CI builds on every push and, on a green
`main`, commits them back to the branch (and uploads them as the
`ticker-firmware-<sha>` artifact). A clean clone therefore already carries flashable
images and needs no Docker at all -- `node flash-ticker.js COM3` reads the app offset
out of the committed partition-table binary rather than hardcoding it.

The SPIFFS image that holds the MIDI files is a separate image, not one of the three:
`flash_midi_data.sh` builds it from `./data` and writes it at `0x317000`, the `storage`
partition offset.

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
