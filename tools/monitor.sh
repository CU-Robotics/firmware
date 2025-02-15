#!/bin/bash

# verify that the backup monitor is compiled
gcc ./tools/monitor.c -Wall -Wextra -Wpedantic -Werror -Wshadow -o ./tools/custom_monitor

# handle sigint in a strange way to not break tycmd monitor
trap 'exit 0' INT;

# The correct serial device has the following name
# usb-Teensyduino_USB_Custom_*-if00
# where * is a number
# The important part is the "if00" at the end, its interface number 00

tty_path=$(./tools/get_tty_path.sh)

# If the tty path is not empty, we can start the monitor
if [ -n "$tty_path" ]; then
	echo "Monitoring Teensy on $tty_path"

	# Start the monitor, if it fails, try the backup monitor
    # If both monitors fail, the teensy is likely in an invalid state and not listening to the serial port. Click the button and reflash to fix.
	./tools/custom_monitor $tty_path || tycmd monitor --reconnect --board="-Teensy@$tty_path" || echo "Failed to monitor."

	exit 0
else
	# failed to find the correct serial device
	echo "Failed to monitor."

	exit 1
fi