tty_path=$(./tools/get_tty_path.sh *-if00)
udevadm info --query=property --name=$tty_path | grep "ID_PATH=" | sed -E 's/.*usb[^-]*-([0-9]+):?([0-9]+).*/usb-\1-\2/' | awk -F'-' '{printf("usb-%d-%s\n", $2 + 1, $3)}'
