# path to the /dev/ serial device that the Teensy is connected to
tty_path=""

# Completely disable ctrl-C since it breaks the monitor
trap 'exit 0' INT;

# The correct serial device has the following name
# usb-Teensyduino_USB_Custom_*-if00
# where * is a number
# The important part is the "if00" at the end, its interface number 00

# Find the correct serial device
for i in /dev/serial/by-id/*; do
	if [[ "$i" == *-if00 ]]; then
		tty_path=$i
		break
	fi
done

# found the id, now we need the actual path to the specific ttyACM device
if [ -n "$tty_path" ]; then
	tty_path=$(readlink -f $tty_path)
fi

# If the tty path is not empty, we can start the monitor
if [ -n "$tty_path" ]; then
	echo "Monitoring Teensy on $tty_path"
	echo "Do not ctrl-C to stop the monitor, use ctrl-D instead"

	# Start the monitor, if this fails its likely because monitor is acting up and a terminal restart is needed
	tycmd monitor --board="-Teensy@$tty_path" || echo "Failed to monitor: Restart your terminal and try again"

	exit 0
else
	# failed to find the correct serial device
	echo "Failed to monitor: Could not find the correct serial device for Teensy"
	echo "This probably means the Teensy is not connected or the Teensy is in bootloader mode (red LED on/blinking)"

	exit 1
fi