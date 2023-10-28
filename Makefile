# Teensyduino Core Library
# http://www.pjrc.com/teensy/
# Copyright (c) 2019 PJRC.COM, LLC.
#
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# 1. The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# 2. If the Software is incorporated into a build system that allows
# selection among a list of target devices, then similar target
# devices manufactured by PJRC.COM must be included in the list of
# target devices and selectable in the same manner.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
OS=$(shell uname -s)

#We try to detect the OS we are running on, and adjust commands as needed
ifeq ($(OS),Windows_NT)
	ifeq ($(shell uname -s),) # not in a bash-like shell
		CLEANUP = del /F /Q
		MKDIR = mkdir
	else # in a bash-like shell, like msys
		CLEANUP = rm -f
		MKDIR = mkdir -p
	endif
	
	LOGSTRING = echo -e
	TARGET_EXTENSION=.exe
	EXECUTABLE = chmod +x
else
	CLEANUP = rm -f
	CLEANDIR = rm -rf 
	MKDIR = mkdir -p
	LOGSTRING = echo
	TARGET_EXTENSION=.out
	EXECUTABLE = chmod +x
	ifeq ($(OS),Darwin)
		ARDUINO_PATH = $(abspath $(HOME)/Library/Arduino15)
	endif
	ifeq ($(OS),Linux)
		ARDUINO_PATH = $(abspath $(HOME)/.arduino15)
	endif
endif

# Use these lines for Teensy 4.1
MCU=IMXRT1062
F_CPU=600000000
USB_MODE=USB_RAWHID
MCU_LD=teensy4/imxrt1062_t41.ld
MCU_DEF=ARDUINO_TEENSY41

# The name of your project (used to name the compiled .hex file)
TARGET=src/main

# configurable options
OPTIONS=-DF_CPU=$(F_CPU) -D$(USB_MODE) -DLAYOUT_US_ENGLISH -DUSING_MAKEFILE

# options needed by many Arduino libraries to configure for Teensy model
OPTIONS+=-D__$(MCU)__ -DTEENSYDUINO=157 -D$(MCU_DEF)

# for Cortex M7 with single & double precision FPU
CPU_OPTIONS=-mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb

# use this for a smaller, no-float printf
#SPECS = --specs=nano.specs

# Other Makefiles and project templates for Teensy
#
# https://forum.pjrc.com/threads/57251?p=213332&viewfull=1#post213332
# https://github.com/apmorton/teensy-template
# https://github.com/xxxajk/Arduino_Makefile_master
# https://github.com/JonHylands/uCee


#************************************************************************
# Location of Teensyduino utilities, Toolchain, and Arduino Libraries.
# To use this makefile without Arduino, copy the resources from these
# locations and edit the pathnames.  The rest of Arduino is not needed.
#************************************************************************

# path of teensy core
CORE_DIR=teensy4

# path location for Arduino libraries (currently not used)
LIBRARY_DIR=libraries

# directory to build in
BUILD_PATH=$(abspath $(CURDIR)/build)

# path location for Teensy Loader, teensy_post_compile and teensy_reboot (on Linux)
TOOLS_PATH=$(abspath $(ARDUINO_PATH)/packages/teensy/tools/teensy-tools/1.57.2)

# path location for the arm-none-eabi compiler
COMPILER_PATH=$(abspath $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/5.4.1/arm/bin)

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

# CPP_FLAGS = compiler options for C and C++
CPP_FLAGS=-Wall -g -O2 $(CPU_OPTIONS) -MMD $(OPTIONS) -Isrc -Iexamples -I$(CORE_DIR) -ffunction-sections -fdata-sections

# compiler options for C++ only
CXX_FLAGS=-std=gnu++14 -felide-constructors -fno-exceptions -fpermissive -fno-rtti -Wno-error=narrowing

# compiler options for C only
C_FLAGS =

# linker options
LD_FLAGS = -Os -Wl,--gc-sections,--relax $(SPECS) $(CPU_OPTIONS) -T$(MCU_LD)

# additional libraries to link
LIBS = -larm_cortexM7lfsp_math -lm -lstdc++

# names for the compiler programs
CC = $(COMPILER_PATH)/arm-none-eabi-gcc
CXX = $(COMPILER_PATH)/arm-none-eabi-g++
SIZE = $(COMPILER_PATH)/arm-none-eabi-size
OBJCOPY = $(COMPILER_PATH)/arm-none-eabi-objcopy
OBJDUMP = $(COMPILER_PATH)/arm-none-eabi-objdump

# automatically create lists of the sources and objects
TC_FILES := $(wildcard $(CORE_DIR)/*.c) $(wildcard $(CORE_DIR)/*/*.c)
TCPP_FILES := $(wildcard $(CORE_DIR)/*.cpp) $(wildcard $(CORE_DIR)/*/*.cpp)
LC_FILES := $(wildcard $(LIBRARY_DIR)/*/*.c)
LCPP_FILES := $(wildcard $(LIBRARY_DIR)/*/*.cpp)
C_FILES_TEST := $(wildcard test/*/*.c)
CPP_FILES_TEST := $(wildcard test/*/*.cpp)
C_FILES_MAIN := $(wildcard src/*.c)
CPP_FILES_MAIN := $(wildcard src/*.cpp)
C_FILES_SOURCE := $(wildcard src/*/*.c)
CPP_FILES_SOURCE := $(wildcard src/*/*.cpp)

# include paths for libraries
L_INC := $(foreach lib, $(filter %/, $(wildcard $(LIBRARY_DIR)/*/)), -I$(lib))

SOURCES := $(C_FILES_SOURCE:.c=.o) $(CPP_FILES_SOURCE:.cpp=.o) $(TC_FILES:.c=.o) $(TCPP_FILES:.cpp=.o) $(LC_FILES:.c=.o) $(LCPP_FILES:.cpp=.o)
OBJS := $(foreach obj, $(SOURCES), $(BUILD_PATH)/$(obj))

BINS := $(C_FILES_TEST:.c=.o) $(CPP_FILES_TEST:.cpp=.o) $(C_FILES_MAIN:.c=.o) $(CPP_FILES_MAIN:.cpp=.o)
BIN_OBJS := $(foreach obj, $(BINS), $(BUILD_PATH)/$(obj))
TARGETS := $(BIN_OBJS:.o=.hex)

all: build

obj: $(OBJS) $(BIN_OBJS) $(MCU_LD)

build: obj $(TARGETS)

main: obj $(BUILD_PATH)/$(TARGET).hex
	@$(LOGSTRING) "[MAKE]\tDone with\t$(word 2,$^)"

post_compile: obj $(BUILD_PATH)/$(TARGET).hex
	@$(LOGSTRING) "[MAKE]\tUploading\t$(basename $(word 2,$^)).hex"
	@-$(abspath $(TOOLS_PATH))/teensy_post_compile -file=$(notdir $(TARGET)) -path=$(BUILD_PATH)/$(dir $(TARGET)) -tools="$(TOOLS_PATH)"

reboot:
	@-$(abspath $(TOOLS_PATH))/teensy_reboot

upload: clean_hex post_compile reboot
	@$(LOGSTRING) "[MAKE]\tDone"

$(BUILD_PATH)/%.o: %.c
	@$(LOGSTRING) "[CC] \t$<"
	@$(MKDIR) "$(dir $@)"
	@$(CC) $(CPP_FLAGS) $(C_FLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILD_PATH)/%.o: %.cpp
	@$(LOGSTRING) "[CXX]\t$<"
	@$(MKDIR) "$(dir $@)"
	@$(CXX) $(CPP_FLAGS) $(CXX_FLAGS) $(L_INC) -o "$@" -c "$<"

# $(BUILD_PATH)/%.o: %.ino
# 	@echo -e "[CXX]\t$<"
# 	@mkdir -p "$(dir $@)"
# 	@$(CXX) $(CPP_FLAGS) $(CXX_FLAGS) $(L_INC) -o "$@" -x c++ -include Arduino.h -c "$<"

%.elf: %.o
	@$(LOGSTRING) "[LD]\t $<"
	@$(CC) $(LD_FLAGS) -o "$@" $< $(OBJS) $(LIBS)
	@$(OBJDUMP) -d -S -C $@ > $(basename $<).lst
	@$(OBJDUMP) -t -C $@ > $(basename $<).sym

%.hex: %.elf 
	@$(LOGSTRING) "[HEX]\t$@"
	@$(OBJCOPY) -O ihex -R .eeprom "$<" "$@"
	@$(EXECUTABLE) $@
	
	@$(SIZE) "$<"

# compiler generated dependency info
-include $(OBJS:.o=.d) 

clean:
	@$(LOGSTRING) "[MAKE]\tCleaning build/"
	@$(CLEANDIR) "$(BUILD_PATH)/$(dir $(TARGET))"

clean_hex:
	@$(LOGSTRING) "[MAKE]\tCleaning hex"
	@$(CLEANUP) "$(BUILD_PATH)/$(TARGET).hex"