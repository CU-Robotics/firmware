#!/bin/bash

# verify that the monitor is compiled
gcc ./tools/monitor.c -Wall -Wextra -Wpedantic -Werror -Wshadow -o ./tools/custom_monitor

# run the monitor
./tools/custom_monitor