# Created by Jackson Stepka (Github: Pandabear1125)

# Use uname to detect current OS
UNAME := $(shell uname -s)

# Teensy core library files
TEENSY_DIR = teensy4
TEENSY_INCLUDE = -I$(TEENSY_DIR)
# name of the output lib file
TEENSY_LIB_NAME = libteensy4.a
# lib file name stripped of initial 'lib' and '.a'
TEENSY_LIB = teensy4

# Used external libraries
LIBRARY_DIR = libraries
# this is where you add your new library to, add it's path with a '-I' attached to the front
LIBRARY_INCLUDE = -Ilibraries/Adafruit_BusIO -Ilibraries/Adafruit_ICM20X -Ilibraries/Adafruit_LIS3MDL -Ilibraries/Adafruit_LSM6DS -Ilibraries/Adafruit_Sensor -Ilibraries/FlexCAN_T4 -Ilibraries/FreqMeasureMulti -Ilibraries/SPI -Ilibraries/unity -Ilibraries/Wire -Ilibraries/VL53L4CD
# name of the output lib file
LIBRARY_LIB_NAME = liblibs.a
# lib file name stripped of initial 'lib' and '.a'
LIBRARY_LIB = libs

# Project files
PROJECT_DIR = .
PROJECT_SOURCE = src/*.cpp src/comms/*.cpp src/controls/*.cpp src/filters/*.cpp src/sensors/*.cpp src/utils/*.cpp
PROJECT_INCLUDE = src
# application filename will end up as PROJECT_NAME.hex once built
PROJECT_NAME = firmware

# Teensy41 compiler flags
TEENSY4_FLAGS = -DF_CPU=600000000 -DUSB_RAWHID -DLAYOUT_US_ENGLISH -D__IMXRT1062__ -DTEENSYDUINO=157 -DARDUINO_TEENSY41

# CPU flags to tailor the code for the Teensy processor
CPU_FLAGS = -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb

# Base compiler flags for both C++ and C
COMPILE_FLAGS = -Wall -g -O2 $(CPU_FLAGS) $(TEENSY4_FLAGS) -I$(TEENSY_INCLUDE) -ffunction-sections -fdata-sections
# C++ specific flags for compiling
CPP_FLAGS = -std=gnu++14 -felide-constructors -fno-exceptions -fpermissive -fno-rtti -Wno-error=narrowing
# c++ moment
CPP_FLAGS += -Wno-trigraphs -Wno-comment

# Required linker config for teensy related things
LINKING_FLAGS = -Wl,--gc-sections,--relax $(CPU_FLAGS) -Tteensy4/imxrt1062_t41.ld

# Required base libs for teensy
BASE_LIBS = -larm_cortexM7lfsp_math -lm -lstdc++

# Detects OS and adds respective paths
# Darwin is Mac, Linux is Linux
ifeq ($(UNAME),Darwin)
 ARDUINO_PATH = $(abspath $(HOME)/Library/Arduino15)
 $(info We've detected you are using a Mac! Consult God if this breaks.)
endif
ifeq ($(UNAME),Linux)
 ARDUINO_PATH = $(abspath $(HOME)/.arduino15)
 $(info We've detected you're on Linux! Nerd.)
endif

# Complete compilers
COMPILER_CPP := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/5.4.1/arm/bin/arm-none-eabi-g++
COMPILER_C := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/5.4.1/arm/bin/arm-none-eabi-gcc
OBJCOPY := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/5.4.1/arm/bin/arm-none-eabi-objcopy

# targets are phony to force it to rebuild every time
.PHONY: build upload monitor kill clean_objs clean_bin clean
.DEFAULT_GOAL = build

# builds source, links with libraries, and constructs the .hex to be uploaded
build: clean
	@echo [Building Source]
	@$(COMPILER_CPP) $(COMPILE_FLAGS) $(CPP_FLAGS) $(PROJECT_SOURCE) $(LIBRARY_LIB_NAME) $(TEENSY_LIB_NAME) $(LIBRARY_INCLUDE) $(TEENSY_INCLUDE) $(LINKING_FLAGS) -o $(PROJECT_NAME).elf
	@echo [Constructing $(PROJECT_NAME).hex]
	@$(OBJCOPY) -O ihex -R .eeprom $(PROJECT_NAME).elf $(PROJECT_NAME).hex
	@chmod +x $(PROJECT_NAME).hex
	@echo [Cleaning Up]
	@rm -f $(PROJECT_NAME).elf 

# builds hex, uploades it, and starts monitoring output
upload: build
	@echo [Uploading] - If this fails, press the button on the teensy and re-run make upload
	@tycmd upload $(PROJECT_NAME).hex
	@tycmd monitor --timeout-eof=-1 -R

# monitors currently running firmware on robot
monitor:
	@echo [Monitoring]
	@tycmd monitor --timeout-eof=-1 -R

# resets teensy and switches it into boot-loader mode, effectively stopping any execution
# this only works if power is consistent, will restart loaded firmware if turned off and on again
kill:
	@echo [Attempting to Kill Teensy]
	@tycmd reset -b

# restarts teensy
restart:
	@echo [Attempting to Restart Firmware]
	@tycmd reset

# deletes all object files
clean_objs:
	@rm -f *.o 

# deletes all hex files
clean_bin:
	@rm -f *.hex 
	@rm -f *.elf 

# overall clean target
clean: clean_objs clean_bin

