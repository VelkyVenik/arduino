# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Arduino development workspace containing multiple Arduino sketches. Each subdirectory represents a separate Arduino project with its own `.ino` file.

## Project Structure

- `DetectChip/` - Arduino chip detection utility that identifies the AVR microcontroller, clock speed, SRAM, and flash memory
- `MagneticSensor/` - USB MIDI controller with TLV493D magnetic sensor and buttons

Each Arduino project follows the standard Arduino IDE structure where the sketch file (`.ino`) must be in a directory with the same name.

## MagneticSensor Features

- **USB MIDI device** with native USB support (MIDIUSB library)
- **5 analog inputs** (directions): LEFT, RIGHT, BACK, FORWARD, DOWN → MIDI CC 1-5
- **2 digital inputs** (buttons): BTN1, BTN2 → MIDI Notes 60, 62 (C4, D4)
- **5 magnitude levels** (1-5) mapped to CC values 25, 51, 76, 102, 127
- **Baseline calibration** on startup (hold still)
- **Direction calibration** with EEPROM storage
- **Serial commands**: C=calibrate, E=EEPROM, L=log, M=midi, R=reboot, H=help

## Development Commands

### Compiling Sketches

```bash
# Compile for Arduino Uno (ATmega328P)
arduino-cli compile --fqbn arduino:avr:uno DetectChip/

# Compile for Arduino Leonardo/Micro (ATmega32U4)
arduino-cli compile --fqbn arduino:avr:leonardo MagneticSensor/
```

### Uploading to Board

```bash
# Upload to Arduino Uno
arduino-cli upload -p /dev/cu.usbserial-1120 --fqbn arduino:avr:uno DetectChip/

# Upload to Arduino Leonardo/Micro (MIDI device)
arduino-cli upload -p /dev/cu.usbmodemMIDI1 --fqbn arduino:avr:leonardo MagneticSensor/
```

### Board Management

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
- Button 1: Pin 4 → GND
- Button 2: Pin 5 → GND
- Serial: 9600 baud
- Default port: /dev/cu.usbmodemMIDI1

## Arduino Core

The project uses `arduino:avr` core version 1.8.6 for AVR-based Arduino boards.
