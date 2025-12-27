# ESP32 Link Device - Ableton Integration Guide

## Complete Workflow

This guide explains how to export MIDI clips from Ableton Live and play them on your ESP32 Link device with perfect sync via Ableton Link.

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│              Ableton Live + Control Surface                 │
│  (Exports MIDI clips, controls sync via Ableton Link)      │
└────────────────────────────┬────────────────────────────────┘
                             │
                    Ableton Link (WiFi)
                             │
              ┌──────────────┴──────────────┐
              │                             │
    ┌─────────▼──────────┐      ┌──────────▼────────┐
    │   ESP32 Link       │      │   External Synth  │
    │   (This Device)    │      │   (Mininova, etc) │
    │                    │      │                   │
    │ - MIDI Playback    │      │   - Receives MIDI │
    │ - Effects (Arp,    │      │   - Synced via    │
    │   Filter, etc)     │      │     Link          │
    │ - Hall Sensor      │      │                   │
    │   Modwheel Offset  │      │                   │
    └────────────────────┘      └───────────────────┘
```

## Installation Steps

### 1. Install Ableton Control Surface

The control surface is located at:
```
~/Documents/Ableton/User Library/Remote Scripts/LinkDeviceController/
```

**On first use:**
```bash
cd ~/Documents/Ableton/User Library/Remote Scripts/LinkDeviceController
python3 setup.py
```

**Then in Ableton Live:**
1. Go to **Live > Preferences > MIDI/Sync > Control Surfaces**
2. Click **"Add Control Surface"**
3. Select **"LinkDeviceController"** from the dropdown
4. (Optional) Assign a MIDI input for hardware control

### 2. Set Up ESP32 Link Device

Ensure your ESP32 is:
- Connected to the same WiFi network as your computer
- Running the latest firmware with Ableton Link support
- Configured with the modwheel/Hall effect sensor integration

## Exporting MIDI Clips

### Quick Export (GUI)

1. In Ableton Live, select a MIDI clip
2. Right-click the clip and select "Export to LinkDevice"
3. The MIDI file is automatically exported to:
   ```
   ~/Documents/Ableton/User Library/Remote Scripts/Clips/Exported/
   ```

### Command-Line Batch Export

Use the clip manager to export multiple clips:

```bash
# List all exported clips
python3 clip_manager.py list

# List with detailed info
python3 clip_manager.py list --verbose

# Get info about a specific clip
python3 clip_manager.py info "MyClip_20240115_120530.mid"

# Export clip list as JSON (for device import)
python3 clip_manager.py export

# Delete unwanted clips
python3 clip_manager.py delete "OldClip.mid"
```

## Network MIDI Transfer (Wireless Clip Sync)

### Auto-Discovery & Auto-Send

The control surface automatically discovers ESP32 Link devices on your network and sends exported clips wirelessly:

1. **Auto-Discovery**: On first run, the control surface searches for ESP32 devices (mDNS: `esp32.local`)
2. **Auto-Send**: After exporting a clip, it's automatically sent to the discovered device
3. **Configuration**: Device IP is saved in `device_config.json` for future use

### Command-Line Device Discovery

Find available devices on your network:

```bash
python3 clip_manager.py discover
```

Output:
```
Found 1 device(s):

1. ESP32-Link (mDNS) at esp32.local:8080
```

### Send Single Clip to Device (CLI)

**Automatic discovery:**
```bash
python3 clip_manager.py send "MyClip_20240115_120530.mid"
```

**Specify device IP explicitly:**
```bash
python3 clip_manager.py send "MyClip_20240115_120530.mid" --device-ip 192.168.1.100 --device-port 8080
```

### Batch Export & Replace All Clips

Export all MIDI clips from Ableton organized by synth channel and replace all device clips:

**From Ableton Control Surface:**
1. Export all clips via control surface batch function
2. Clips are organized by synth (Mininova, MicroKorg)
3. All clips automatically sent to device

**From Command Line (Replace Mode):**
```bash
python3 clip_manager.py send-all --clear
```

This will:
1. Clear all existing MIDI files from device
2. Send all exported clips from Ableton
3. Organize them by synth prefix (MININOVA_, MICROKORG_)

**Append Mode (Keep Existing):**
```bash
python3 clip_manager.py send-all
```

**Send Only Specific Synth Clips:**
```bash
# Send only Mininova clips
python3 clip_manager.py send-all --filter MININOVA

# Send only MicroKorg clips
python3 clip_manager.py send-all --filter MICROKORG
```

### Device Configuration

Edit `LinkDeviceController/device_config.json` to customize:

```json
{
  "device_ip": "esp32.local",
  "device_port": 8080,
  "auto_send": true
}
```

- `device_ip`: ESP32 hostname or IP address
- `device_port`: HTTP server port (default 8080)
- `auto_send`: Automatically send clips after export (true/false)

## Transferring Clips to Device

### Option 1: Wireless Network Transfer (Recommended)
1. Export the clip from Ableton
2. Clips are automatically sent to device via HTTP if auto_send is enabled
3. Clip appears in device's SPIFFS storage `/spiffs/loops/`

### Option 2: Manual Network Transfer
1. Use `clip_manager.py send` command (see above)
2. Specify device IP if auto-discovery fails

### Option 3: Manual USB File Transfer
1. Export the clip from Ableton
2. Copy the `.mid` file to device's SPIFFS:
   ```bash
   cp ~/Documents/Ableton/User\ Library/Remote\ Scripts/Clips/Exported/MyClip.mid \
      /path/to/esp32/spiffs/loops/
   ```

### Option 4: Via Ableton Link (Direct Sync)
1. Keep Ableton Live running with Link enabled
2. Press play on the clip in Ableton
3. The ESP32 automatically syncs to the tempo and beat
4. Use the device's arpeggiator to trigger MIDI playback

## MIDI Sync Details

### Timing Accuracy

The ESP32 device uses **simplified Ableton Link synchronization**:

- **Quantum Boundary**: 16 beats (1 bar at 4/4 time)
- **MIDI Timing Clocks**: 24 per quarter note
- **Song Position Pointer**: Sent every 4 beats for drift correction
- **Metronome**: Emphasizes beat hierarchy (16-beat, 8-beat, 4-beat, 1-beat)

### Quantum Boundary Detection

```c
// Simplified detection: checks if quantum number changed
if (current_quantum_number != last_quantum_number) {
    // Quantum boundary crossed - perfect time to sync
    send_midi_start_stop();  // Realign all devices
    play_metronome_click();   // User feedback
}
```

### MIDI Clock Delivery

- **Main Timer**: 2000 Hz (500 µs period)
- **MIDI Timer**: 4000 Hz (250 µs period) for note playback
- **Clock Jitter**: Minimized through direct timer callback
- **Message Format**:
  ```
  MIDI Start (0xFA)          - Play state change
  MIDI Stop (0xFC)           - Pause state change
  MIDI Clock (0xF8)          - 24 clocks per quarter
  Song Position Pointer (0xF2) - Sync position
  ```

## Using Exported Clips on Device

### File Format
Each exported clip contains:
- **MIDI File** (.mid): Standard MIDI note data
- **Metadata** (.json): Tempo, time signature, clip name, timestamp

### Example File Structure
```
MyClip_20240115_120530.mid
MyClip_20240115_120530_metadata.json
```

### Metadata Example
```json
{
  "name": "MyClip",
  "length": 16.0,
  "tempo": 120,
  "time_signature": [4, 4],
  "created": "2024-01-15T12:05:30"
}
```

## Modwheel with Hall Effect Sensor

The device implements a unique modwheel feature:

- **Raw Modwheel**: Pot 1 controls standard CC 1
- **Hall Sensor Offset**: ESP32's built-in Hall effect sensor adds a dynamic offset
- **Combined Output**: `modwheel_value = raw_pot1 + (hall_sensor - center_offset)`

### Using Hall Modwheel
1. Move Pot 1 to set base modwheel value
2. Bring a magnet near the ESP32 to modulate the offset
3. The modwheel value changes dynamically based on magnet proximity
4. Perfect for hands-free modulation during performance

## WiFi Configuration

### Setting WiFi Credentials

Credentials are stored in ESP32's NVS (Non-Volatile Storage):

**From Device UI:**
1. If device can't connect to WiFi, it auto-starts a hotspot
2. Connect to the device's hotspot
3. Use the device's menu to set WiFi SSID and password

**Via Command Line (during development):**
```cpp
wifi_config_set_credentials("YourSSID", "YourPassword");
```

The device remembers credentials across power cycles.

## Troubleshooting

### Clips Not Syncing
- Verify Ableton Link is enabled in both Ableton and the device
- Check that both devices are on the same network
- Ensure the tempo is stable (avoid sudden changes)
- Review the device's MIDI clock output

### Timing Drifts
- This is expected after long sessions
- Quantum boundaries force realignment automatically
- If drift persists, check WiFi stability between devices

### WiFi Connection Issues
- Verify correct SSID and password
- Check that the device's WiFi antenna is unobstructed
- Restart the device and Ableton Link
- Check for interference from other 2.4GHz devices

### Export Fails in Ableton
- Ensure you have a MIDI clip selected (not audio)
- Verify the export directory has write permissions
- Check Ableton's console for detailed error messages
- Try exporting from a different track

## Performance Tips

### Optimal Setup
1. **WiFi**: Place router near both computer and device
2. **Tempo**: Use stable 120 BPM for testing
3. **Clips**: Keep MIDI clips under 32 bars for quick loading
4. **Effects**: Disable unused effects to free up CPU

### Minimizing Latency
1. Use wired network connection if possible
2. Reduce WiFi interference (change channel on router)
3. Disable other wireless devices nearby
4. Keep Ableton Link quantum size at 16 beats

## Advanced Usage

### Custom MIDI Processing
Modify the device's arpeggiator or MIDI processing:
1. Edit MIDI files before exporting
2. Use Ableton's MIDI effects to shape clips
3. Save multiple versions with different effects

### Batch Operations
Use clip_manager.py to organize workflow:

```bash
# Create a project-specific clip collection
python3 clip_manager.py export --output "MyProject_clips.json"

# Then distribute clips to collaborators
```

## Files Reference

### Control Surface
- `LinkDeviceController/__init__.py` - Main control surface class
- `clip_manager.py` - CLI utility for clip management
- `setup.py` - Installation verification script
- `CLIP_EXPORT_README.md` - Feature documentation

### Exported Clips
- `Clips/Exported/*.mid` - MIDI files (standard format)
- `Clips/Exported/*_metadata.json` - Clip metadata

## API Reference

### LinkDeviceController Methods
```python
# Export currently selected clip
device.export_selected_clip() -> bool

# List all exported clips
device.list_exported_clips() -> list[str]

# Get path to last exported clip
device.get_last_exported_clip_path() -> str
```

### Clip Manager Commands
```bash
clip_manager.py list [--verbose] [--sort {date,name,size}]
clip_manager.py info <filename>
clip_manager.py delete <filename> [--force]
clip_manager.py export [--output <filename>]
```

## Contributing & Future Enhancements

Planned features:
- Real-time clip export during playback
- Batch export entire projects
- Direct wireless clip transfer
- Clip preview in control surface
- Version control for clip iterations
- Integration with Max for Live

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review Ableton's console logs (Cmd+Shift+O on Mac)
3. Check device's serial output with:
   ```bash
   idf.py monitor -p /dev/ttyUSB0  # or your serial port
   ```

## License

LinkDeviceController is provided as-is for use with the ESP32 Link device.

---

**Last Updated:** 2024-01-15
**Firmware Version:** 1.0+
**Ableton Version:** 12.0+
