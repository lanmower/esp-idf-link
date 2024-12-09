# ESP-IDF Link Example

This project demonstrates the integration of Ableton Link protocol with ESP32, providing MIDI synchronization capabilities and audio feedback. It's built using ESP-IDF v4.4.4 and includes features for MIDI communication, timing synchronization, and audio indication through a buzzer.

## Features

- Ableton Link integration for tempo synchronization
- MIDI communication via UART
- USB MIDI support
- Audio feedback through a buzzer with different frequencies for beat positions
- WiFi connectivity for Link protocol
- LED status indication

## Hardware Requirements

- ESP32 development board
- Buzzer (connected to GPIO 4)
- LED (connected to GPIO 2)
- MIDI interface components (optional)

## Pin Configuration

### UART Pins
- USB UART (UART0):
  - TX: GPIO 1
  - RX: GPIO 3
- MIDI UART (UART2):
  - TX: GPIO 17
  - RX: GPIO 16

### Other Pins
- LED: GPIO 2
- Buzzer: GPIO 4 (PWM controlled)

## Audio Feedback

The buzzer provides different tones for different beat positions:
- 16th beat: C7 note (2093 Hz)
- 8th beat: G6 note (1568 Hz)
- Quarter beat: E6 note (1319 Hz)
- Normal beat: C6 note (1047 Hz)

## Dependencies

- ESP-IDF v4.4.4
- Ableton Link library
- FreeRTOS
- ESP32 standard drivers (GPIO, UART, LEDC, Timer)

## Building and Flashing

1. Set up ESP-IDF development environment
2. Configure WiFi settings using `idf.py menuconfig`
3. Build the project:
   ```
   idf.py build
   ```
4. Flash to your ESP32:
   ```
   idf.py flash
   ```
5. Monitor the output:
   ```
   idf.py monitor
   ```

## Configuration

The project includes several configurable parameters:
- Link tick period: 100ms
- PWM configuration for the buzzer
- UART baud rates and buffer sizes
- WiFi credentials (via menuconfig)

## License

[Add appropriate license information]

## Contributing

[Add contribution guidelines if applicable]
