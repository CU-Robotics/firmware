#!/bin/bash

# verify that the custom monitor is compiled
gcc ./tools/monitor.c -g -Wall -Wextra -Wpedantic -Werror -Wshadow -o ./tools/custom_monitor

# handle sigint in a strange way to not break tycmd monitor
trap 'exit 0' INT;

# The correct serial device has the following name
# usb-Teensyduino_USB_Custom_*-if00
# where * is a number
# The important part is the "if00" at the end, its interface number 00

# Get the tty path of the correct serial device
# The tty path is only used by tycmd monitor
tty_path=$(./tools/get_tty_path.sh *-if00)

# Try to monitor using the custom monitor, if it fails, use tycmd monitor, if that fails, exit
# The custom monitor has its own logic to find the correct serial device
./tools/custom_monitor || tycmd monitor --reconnect --board="-Teensy@$tty_path" || (echo "Failed to monitor." && exit 1)

exit 0