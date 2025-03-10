#!/bin/bash

# Get the device file for the Teensy Serial interface
# tycmd list -Ojson -v : List all devices in JSON format with verbose output, this includes the device file
# grep -e "\-Teensy\"" : Filter out only the Teensy devices, this excludes devices like Teensy@1
# grep -o "Serial\", \"[^]]*" : Extract the serial device path section from the JSON output
# grep -o '/[^"]*' : Extract the device file path from the serial device path section
teensy_serial_path=$(tycmd list -Ojson -v | grep -e "\-Teensy\"" | grep -o "Serial\", \"[^]]*" | grep -o '/[^"]*')

# Check if the device file exists
if [ -e "$teensy_serial_path" ]; then
    echo "$teensy_serial_path"
    exit 0
fi

# If the device file does not exist, print an error message and exit with an error code (redirected to stderr)
echo "Teensy device file not found" 1>&2

exit 1