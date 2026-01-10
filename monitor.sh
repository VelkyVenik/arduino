#!/bin/bash
DEVICE="${1:-/dev/cu.usbmodem11201}"
arduino-cli monitor -p "$DEVICE" -c baudrate=9600
