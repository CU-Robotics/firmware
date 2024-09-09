# This finds the correct ttyACM device and puts it into gdb_commands.txt

# The correct serial device has the following name
# usb-Teensyduino_USB_Custom_*-if02
# Where * is a number unique to that serial device, we dont care about that though
# The important part is the "if02" at the end, its interface number 02
# SerialUSB1 is what interface 02 references

tty_path=""

# Find the correct serial device
for i in /dev/serial/by-id/*; do
	if [[ "$i" == *-if02 ]]; then
		tty_path=$i
		break
	fi
done

# found the id, now we need the actual path to the specific ttyACM device
if [ -n "$tty_path" ]; then
	tty_path=$(readlink -f $tty_path)
fi

# Now we put it into gdb_commands.txt
if [ -n "$tty_path" ]; then
	echo "target remote $tty_path" > ./tools/gdb_commands.txt
	
	exit 0
else
	echo "Failed to find the correct serial device for Teensy"
	echo "This probably means the Teensy is not connected or the Teensy is in bootloader mode (red LED on/blinking)"

	exit 1
fi