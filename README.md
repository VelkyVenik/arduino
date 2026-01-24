# Arduino MIDI Controller Projects

This repository contains Arduino development projects, with a focus on creating MIDI controllers with various sensor integrations.

## Projects

### 1. MagneticSensor - USB MIDI Controller with 8x8 LED Matrix

An advanced Arduino Leonardo/Micro-based USB MIDI controller featuring a TLV493D 3-axis magnetic sensor, 8x8 LED matrix display, and dual toggle buttons.

#### Features

**Magnetic Sensor Input**
- USB MIDI device with native USB support (MIDIUSB library)
- TLV493D 3-axis magnetic sensor for multi-directional control
- 5-direction support: LEFT, RIGHT, BACK, FORWARD, DOWN
- Multiple simultaneous directions can be active (up to 5 directions at once)
- 5 magnitude levels (1-5) for expressive control

**MIDI Output**
- 5 analog inputs → MIDI CC 1-5 (one CC per direction)
- Magnitude levels mapped to CC values: 25, 51, 76, 102, 127
- 2 digital inputs (buttons) → MIDI Notes 60, 62 (C4, D4)
- MIDI burst mode for MainStage compatibility (all CC values sent together)
- Channel 1 by default

**LED Matrix Display (MAX7219)**
- 8x8 LED matrix for real-time visual feedback
- Direction indicators with animated arrows (LEFT, RIGHT, FORWARD, BACK, DOWN)
- Idle smiley face pattern
- Magnitude indicator bar showing 1-5 dots on bottom row
- Button state indicators in top corners
- Adjustable brightness (8 levels)
- Dynamic intensity based on direction state

**Button Functionality**
- **2 Toggle Buttons**: BTN1 (Note 60), BTN2 (Note 62)
- Button states displayed as indicators on LED matrix
- **Calibration Trigger**: Hold both buttons for 5 seconds to start full calibration sequence

**Calibration System**
- **Baseline Calibration**: Automatic on startup with spinning wave animation
- **Direction Calibration**: Triggered on startup or via long-press of both buttons
- Visual feedback showing which direction to move (flashing arrow on LED matrix)
- Checkmark animation on successful calibration
- Cross pattern on calibration failure
- **EEPROM Storage**: Calibration data saved automatically
- Can recalibrate directions by pressing both buttons for 5 seconds

**Serial Commands** (9600 baud)
- `B` - Cycle brightness (0, 3, 6, 9, 12, 15 levels)
- `C` - Full calibration (baseline + all directions)
- `D` - Toggle LED display ON/OFF
- `E` - Show stored EEPROM calibration data
- `L` - Toggle logging to serial ON/OFF
- `M` - Toggle MIDI output ON/OFF
- `R` - Reboot device
- `H` - Show help message

#### Hardware Configuration

**Board**: Arduino Leonardo or Arduino Micro (ATmega32U4)

**Pin Connections**
- TLV493D Sensor (I2C):
  - SDA → Pin 2
  - SCL → Pin 3
- MAX7219 LED Matrix:
  - DIN → Pin 7
  - CS → Pin 8
  - CLK → Pin 9
- Buttons:
  - BTN1 → Pin 4 (connect to GND)
  - BTN2 → Pin 5 (connect to GND)
- Serial: 9600 baud

**Required Libraries**
- MIDIUSB (for USB MIDI support)
- LedControl (for MAX7219 LED matrix)
- TLx493D_inc.hpp (Infineon TLV493D sensor library)

#### Usage Instructions

**Initial Setup**
1. Upload the sketch to your Arduino Leonardo/Micro
2. Device automatically performs baseline calibration on startup
3. Serial monitor shows calibration progress
4. Direction calibration begins automatically if no EEPROM data exists
5. Once calibration completes, device shows idle pattern (smiley) on LED matrix

**Recalibration**
- Hold both buttons (BTN1 and BTN2) for 5 seconds while LED matrix shows progress animation
- Device automatically recalibrates baseline and all directions
- LED matrix shows completion status

**MIDI Usage**
- Connect to computer as a USB MIDI device
- Sensor movements send CC values on channels 1-5
- Button presses send Note On/Off messages
- All 5 directions can be active simultaneously (e.g., pressing left and down at once sends both CC messages)
- MIDI burst mode sends all CC values together, compatible with Logic Pro, MainStage, and other DAWs

**Serial Monitoring**
```bash
# Default port
./monitor.sh

# Or specify port
./monitor.sh /dev/cu.usbmodemMIDI1

# Or use arduino-cli directly
arduino-cli monitor -p /dev/cu.usbmodemMIDI1 -c baudrate=9600
```

**Control LED Brightness**
- Use serial command `B` to cycle through brightness levels
- Visual feedback on LED matrix shows current setting

#### Compilation & Upload

**Compile**
```bash
arduino-cli compile --fqbn arduino:avr:leonardo MagneticSensor/
```

**Upload**
```bash
arduino-cli upload -p /dev/cu.usbmodemMIDI1 --fqbn arduino:avr:leonardo MagneticSensor/
```

#### Development

**Sensor Reading**: Samples magnetic field 5 times per reading cycle with 50ms delays
**MIDI Update Rate**: ~40-50ms between sensor reads
**Response**: Immediate visual feedback on LED matrix and MIDI output

---

### 2. DetectChip - Arduino Microcontroller Detector

Simple Arduino sketch for identifying AVR microcontroller specifications (chip type, clock speed, SRAM, flash memory).

**Usage**
```bash
# Compile for Arduino Uno
arduino-cli compile --fqbn arduino:avr:uno DetectChip/

# Upload to Arduino Uno
arduino-cli upload -p /dev/cu.usbserial-1120 --fqbn arduino:avr:uno DetectChip/
```

---

### 3. blink - Interactive LED Animation System

5 LED animation patterns with button-controlled mode cycling.

**Usage**
```bash
# Compile for Arduino Uno
arduino-cli compile --fqbn arduino:avr:uno blink/

# Upload to Arduino Uno
arduino-cli upload -p /dev/cu.usbserial-1120 --fqbn arduino:avr:uno blink/
```

---

### 4. digital_display - 7-Segment Display Multi-Mode

7-segment display with stopwatch, thermometer, and countdown timer modes.

**Usage**
```bash
# Compile for Arduino Uno
arduino-cli compile --fqbn arduino:avr:uno digital_display/

# Upload to Arduino Uno
arduino-cli upload -p /dev/cu.usbserial-1120 --fqbn arduino:avr:uno digital_display/
```

---

## Core Version

- Arduino AVR Core: 1.8.6

## Board Management Commands

```bash
# List connected boards
arduino-cli board list

# List installed cores
arduino-cli core list

# Search for libraries
arduino-cli lib search TLx493D

# Install a library
arduino-cli lib install "TLx493D"
```

## License

Arduino projects repository.
