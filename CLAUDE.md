# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Arduino development workspace containing multiple Arduino sketches. Each subdirectory represents a separate Arduino project with its own `.ino` file.

## Project Structure

- `DetectChip/` - Arduino chip detection utility that identifies the AVR microcontroller, clock speed, SRAM, and flash memory
- `MagneticSensor/` - TLV493D magnetic field sensor integration using the Infineon TLx493D library over I2C (SDA=A4, SCL=A5)

Each Arduino project follows the standard Arduino IDE structure where the sketch file (`.ino`) must be in a directory with the same name.

## Development Commands

### Compiling Sketches

```bash
# Compile a specific sketch for Arduino Uno (ATmega328P)
arduino-cli compile --fqbn arduino:avr:uno DetectChip/

# Compile for Arduino Nano
arduino-cli compile --fqbn arduino:avr:nano MagneticSensor/
```

### Uploading to Board

```bash
# Upload to Arduino Uno
arduino-cli upload -p /dev/cu.usbserial-1120 --fqbn arduino:avr:uno DetectChip/

# Upload to Arduino Nano (with old bootloader - ATmega328P Old Bootloader)
arduino-cli upload -p /dev/cu.usbserial-1120 --fqbn arduino:avr:nano:cpu=atmega328old MagneticSensor/

# Note: Older Arduino Nano boards require cpu=atmega328old parameter
# If upload fails with sync errors, try the old bootloader setting
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
# Monitor serial output (9600 baud is used in these sketches)
# Use device from command line or default /dev/cu.usbmodem11201
arduino-cli monitor -p /dev/cu.usbmodem11201 -c baudrate=9600
```

## Hardware Configuration

- The MagneticSensor project expects TLV493D sensor on I2C: SDA=A4 (pin A4), SCL=A5 (pin A5)
- Both sketches use Serial at 9600 baud for output
- Default board port: /dev/cu.usbmodem11201

## Arduino Core

The project uses `arduino:avr` core version 1.8.6 for AVR-based Arduino boards (Uno, Nano, etc.).
