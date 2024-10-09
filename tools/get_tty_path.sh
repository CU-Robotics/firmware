#!/bin/bash

# This finds the correct ttyACM device. First argument passed in should be the device ID.
# Example usage: tty_path=$(./tools/get_tty_path.sh *-f00)

# Device id, passed in as first argument with $1.
tty_id=$1

# path to the /dev/ serial device that the Teensy is connected to
tty_path=""

# if no first argument provided
if [ ! -n "$tty_id" ]; then
    # 1>&2 redirects stdout (1) into stderr (2)
    echo "\"$1\" is not a valid tty ID" 1>&2
    exit 1
fi

# Find the correct serial device
# The important part is the ID/interface number at the end, which is compared here
for i in /dev/serial/by-id/*; do
    # compare with first argument passed to this command
	if [[ "$i" == $tty_id ]]; then
		tty_path=$i
		break
	fi
done

# found the id, now we need the actual path to the specific ttyACM device
if [ -n "$tty_path" ]; then
	tty_path=$(readlink -f $tty_path)
fi

# if no tty path found
if [ ! -n "$tty_path" ]; then
    # 1>&2 redirects stdout (1) into stderr (2)
    echo "Failed to find the correct serial device for Teensy with tty_id '$tty_id' " 1>&2
    echo "This probably means the Teensy is not connected or the Teensy is in bootloader mode (red LED on/blinking)" 1>&2

    exit 1
fi

echo $tty_path # outputs to stdout by default

exit 0