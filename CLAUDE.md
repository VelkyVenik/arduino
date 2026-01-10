# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Arduino development workspace containing multiple Arduino sketches. Each subdirectory represents a separate Arduino project with its own `.ino` file.

## Project Structure

- `DetectChip/` - Arduino chip detection utility that identifies the AVR microcontroller, clock speed, SRAM, and flash memory
- `MagneticSensor/` - 3-axis magnetic field monitor with TLV493D sensor for directional input detection

Each Arduino project follows the standard Arduino IDE structure where the sketch file (`.ino`) must be in a directory with the same name.

## MagneticSensor Features

- **Baseline calibration** on startup (hold still)
- **Direction calibration** for LEFT, RIGHT, BACK, FORWARD, DOWN with EEPROM storage
- **5 magnitude levels** (1-5) at 20% intervals of calibrated max
- **Button support** on pins 4 and 5
- **Serial commands**: C=calibrate, E=show EEPROM, L=toggle log, R=reboot, H=help
- **Event output**: `EVENT: LEFT 3`, `BTN1`, `BTN2`

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

# Upload to Arduino Leonardo/Micro
arduino-cli upload -p /dev/cu.usbmodem11201 --fqbn arduino:avr:leonardo MagneticSensor/
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
./monitor.sh /dev/cu.usbmodem11201

# Or directly with arduino-cli
arduino-cli monitor -p /dev/cu.usbmodem11201 -c baudrate=9600
```

## Hardware Configuration

### MagneticSensor (Leonardo/Micro)
- TLV493D sensor on I2C: SDA (pin 2), SCL (pin 3)
- Button 1: Pin 4 → GND
- Button 2: Pin 5 → GND
- Serial: 9600 baud
- Default port: /dev/cu.usbmodem11201

## Arduino Core

The project uses `arduino:avr` core version 1.8.6 for AVR-based Arduino boards.
