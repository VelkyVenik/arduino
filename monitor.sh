#!/bin/bash
DEVICE="${1:-/dev/cu.usbmodemMIDI1}"
arduino-cli monitor -p "$DEVICE" -c baudrate=9600
