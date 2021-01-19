#!/bin/sh

# Run this script to copy the SerialPackets library into your user's Arduino libraries directory.

ARDUINO_LIBRARY_PATH=~/Arduino/libraries/SerialPackets

echo Copying SerialPackets library to \"$ARDUINO_LIBRARY_PATH\"...
rm -rf "$ARDUINO_LIBRARY_PATH"
cp -r ./src/arduino/SerialPackets "$ARDUINO_LIBRARY_PATH"
echo Done.
