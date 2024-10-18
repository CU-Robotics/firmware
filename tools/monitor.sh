#!/bin/bash

# handle sigint in a strange way to not break tycmd monitor
trap 'exit 0' INT;

# The correct serial device has the following name
# usb-Teensyduino_USB_Custom_*-if00
# where * is a number
# The important part is the "if00" at the end, its interface number 00

tty_path=$(./tools/get_tty_path.sh *-if00)

# If the tty path is not empty, we can start the monitor
if [ -n "$tty_path" ]; then
	echo "Monitoring Teensy on $tty_path"

	# Start the monitor, if this fails its likely because monitor is acting up and a terminal restart is needed
	tycmd monitor --board="-Teensy@$tty_path" || echo "Failed to monitor: Restart your terminal and try again"

	exit 0
else
	# failed to find the correct serial device
	echo "Failed to monitor."

	exit 1
fi