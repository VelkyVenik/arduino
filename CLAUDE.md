# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Arduino development workspace containing multiple Arduino sketches. Each subdirectory represents a separate Arduino project with its own `.ino` file.

## Project Structure

- `DetectChip/` - Arduino chip detection utility that identifies the AVR microcontroller, clock speed, SRAM, and flash memory
- `MagneticSensor/` - USB MIDI controller with TLV493D magnetic sensor, 8x8 LED matrix display, and buttons
- `blink/` - Interactive LED animation system with 5 LEDs displaying button-controlled patterns
- `digital_display/` - 7-segment display with stopwatch, thermometer, and countdown timer modes

Each Arduino project follows the standard Arduino IDE structure where the sketch file (`.ino`) must be in a directory with the same name.

## MagneticSensor Features

- **USB MIDI device** with native USB support (MIDIUSB library)
- **5 analog inputs** (directions): LEFT, RIGHT, BACK, FORWARD, DOWN → MIDI CC 1-5
- **2 digital inputs** (buttons): BTN1, BTN2 → MIDI Notes 60, 62 (C4, D4) with toggle ON/OFF states
- **5 magnitude levels** (1-5) mapped to CC values 25, 51, 76, 102, 127
- **8x8 LED matrix display** with direction arrows and magnitude-based intensity (brightness 3-15)
- **Baseline calibration** on startup (hold still)
- **Direction calibration** with EEPROM storage
- **Serial commands**: C=calibrate, D=display, E=EEPROM, L=log, M=midi, R=reboot, H=help
- **Runtime toggles**: Log, MIDI output, and LED display can be enabled/disabled via serial

## DetectChip Features

- **Chip detection** for AVR microcontrollers: ATmega328P/328, ATmega168P/168, ATmega88P/88
- **Clock speed** reporting (F_CPU in MHz)
- **SRAM size** reporting (RAMEND + 1 bytes)
- **Flash memory** reporting (FLASHEND + 1 bytes)
- **One-time execution** - runs detection once at startup, then idle
- **2-second startup delay** before output begins
- **Works on any AVR-based Arduino** (Uno, Nano, Pro Mini, Mega, Leonardo)

## blink Features

- **5 LED animation patterns** with smooth sequential playback
- **Button-controlled mode cycling** - press to advance to next pattern
- **Pattern library**:
  1. Left-to-right scanning with back-and-forth motion (20 frames)
  2. All LEDs on/off sequence (8 frames)
  3. Single LED ping-pong effect (10 frames)
  4. Two-LED windshield wiper (10 frames)
  5. Scrolling 3-LED window (4 frames)
- **Quick flash feedback** when button held during mode transition
- **100ms frame delay** for smooth animation playback
- **Pattern interruption** - pressing button during animation immediately advances to next pattern
- **No external libraries required** - uses only Arduino core functions

## digital_display Features

- **3 operational modes** with persistent state:
  1. **Stopwatch** - tracks time in centiseconds (XX.X format), start/stop control
  2. **Thermometer** - real-time temperature from OneWire sensor (DS18S20/DS18B20/DS1822)
  3. **Countdown** - persistent timer saved to EEPROM (DDDHH format: days×100 + hours)
- **7-segment LED display** - 3 digits with decimal point via shift register
- **Boot animation** on startup with predefined sequences
- **Button controls**:
  - Start button: Short press = toggle stopwatch, Long press (>2s) = reset
  - Mode button: Cycles through 3 modes with 300ms debounce
- **Dynamic refresh rate**: 100ms when stopwatch running, 500ms otherwise
- **EEPROM persistence** - countdown survives power cycles
- **Temperature sensor validation** - automatic chip type identification and CRC checking
- **Libraries**: EEPROM, OneWire

## Development Commands

### Compiling Sketches

```bash
# Compile for Arduino Uno (ATmega328P) - DetectChip, blink, digital_display
arduino-cli compile --fqbn arduino:avr:uno DetectChip/
arduino-cli compile --fqbn arduino:avr:uno blink/
arduino-cli compile --fqbn arduino:avr:uno digital_display/

# Compile for Arduino Leonardo/Micro (ATmega32U4) - MagneticSensor
arduino-cli compile --fqbn arduino:avr:leonardo MagneticSensor/
```

### Uploading to Board

```bash
# Upload to Arduino Uno (for DetectChip, blink, digital_display)
arduino-cli upload -p /dev/cu.usbserial-1120 --fqbn arduino:avr:uno DetectChip/
arduino-cli upload -p /dev/cu.usbserial-1120 --fqbn arduino:avr:uno blink/
arduino-cli upload -p /dev/cu.usbserial-1120 --fqbn arduino:avr:uno digital_display/

# Upload to Arduino Leonardo/Micro (MIDI device - MagneticSensor)
arduino-cli upload -p /dev/cu.usbmodemMIDI1 --fqbn arduino:avr:leonardo MagneticSensor/
```

### Board Management

```bash
# List connected boards
arduino-cli board list

# List installed cores
arduino-cli core list
```

### Library Installation

```bash
# MagneticSensor required libraries
arduino-cli lib install "TLx493D"
arduino-cli lib install "MIDIUSB"
arduino-cli lib install "LedControl"

# digital_display required libraries
arduino-cli lib install "OneWire"

# Other projects (DetectChip, blink) use only built-in Arduino libraries

# Search for a library
arduino-cli lib search <library-name>
```

### Serial Monitor

```bash
# Use helper script (default port or specify)
./monitor.sh
./monitor.sh /dev/cu.usbmodemMIDI1

# Or directly with arduino-cli
arduino-cli monitor -p /dev/cu.usbmodemMIDI1 -c baudrate=9600
```

## Hardware Configuration

### MagneticSensor (Leonardo/Micro)
- TLV493D sensor on I2C: SDA (pin 2), SCL (pin 3)
- LED Matrix (MAX7219): DIN (pin 7), CS (pin 8), CLK (pin 9)
- Button 1: Pin 4 → GND (toggle)
- Button 2: Pin 5 → GND (toggle)
- Serial: 9600 baud
- Default port: /dev/cu.usbmodemMIDI1

### DetectChip (Uno/Nano/Any AVR)
- No external hardware required
- Serial: 9600 baud (2-second startup delay before output)
- Supported chips: ATmega328P/328, ATmega168P/168, ATmega88P/88
- Default port: /dev/cu.usbserial-1120

### blink (Uno/Nano/Any AVR)
- LED 1-5: Pins 2-6 (through 220-330Ω resistors to GND)
- Button: Pin 7 → GND (INPUT_PULLUP)
- Serial: 9600 baud
- 5 animation patterns, button cycles through modes

### digital_display (Uno)
- 7-segment display: Clock (pin 8), Data (pin 9)
- OneWire temp sensor: Pin 10 (requires 4.7K pull-up)
- Start button: Pin 3 → GND (INPUT_PULLUP)
- Mode button: Pin 2 → GND (INPUT_PULLUP)
- Serial: 9600 baud
- 3 modes: Stopwatch, Thermometer, Countdown timer

## Arduino Core

The project uses `arduino:avr` core version 1.8.6 for AVR-based Arduino boards.
