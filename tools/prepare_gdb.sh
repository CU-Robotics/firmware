#!/bin/bash

# This finds the correct ttyACM device and puts it into gdb_commands.txt

# The correct serial device has the following name
# usb-Teensyduino_USB_Custom_*-if02
# Where * is a number unique to that serial device, we dont care about that though
# The important part is the "if02" at the end, its interface number 02
# SerialUSB1 is what interface 02 references

tty_path=$(./tools/get_tty_path.sh *-if02)

# Now we put it into gdb_commands.txt
if [ -n "$tty_path" ]; then
	echo "target remote $tty_path" > ./tools/gdb_commands.txt
	
	exit 0
else
	echo "Failed to prepare gdb"

	exit 1
fi