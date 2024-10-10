#!/bin/bash

# The correct serial device has the following name
# usb-Teensyduino_USB_Custom_*-if00
# where * is a number
# The important part is the "if00" at the end, its interface number 00

# Teensy wont show up as a serial device if it is in bootloader, so take it out of boot loader if needed
# grep will return 0 on finding something, 1 on not finding something
reset_needed=0
if [ "$(tycmd list | grep HalfKay)" ]; then
	tycmd reset -q
	reset_needed=1
fi 

tty_path=$(./tools/get_tty_path.sh *-if00)

# If the tty path is not empty, we can start the monitor
if [ ! -n "$tty_path" ]; then
	# failed to find the correct serial device
	echo "Failed to monitor."

	exit 1
fi

# print the command to be ran to monitor
echo "tycmd monitor --board=\"-Teensy@$tty_path\""

# return teensy into bootloader if it needed to be taken out before
if [ $reset_needed == 1 ]; then
	tycmd reset -b -q
fi

exit 0