# Made by Jack Miller 2024 (Github: guywithhat99)
# Thanks to Jackson Stepka (Github: Pandabear1125); a lot of code is based off his original makefile 

# Detect the current operating system using uname
UNAME := $(shell uname -s)

# The name of the target executable
TARGET_EXEC := firmware

# Directory where build outputs will be placed
BUILD_DIR := ./build

# Tools directory
TOOLS_DIR := ./tools

# Source directories
TEENSY_SRC_DIRS := ./teensy4
LIBRARY_SRC_DIRS := ./libraries
SRC_SRC_DIRS := ./src

# Find all C, C++, and assembly source files in the specified directories
# Note: Single quotes are used to prevent the shell from expanding '*'
TEENSY_SRC := $(shell find $(TEENSY_SRC_DIRS) -name '*.cpp' -or -name '*.c')
LIBRARY_SRC := $(shell find $(LIBRARY_SRC_DIRS) -name '*.cpp' -or -name '*.c')
SRC_SRC := $(shell find $(SRC_SRC_DIRS) -name '*.cpp' -or -name '*.c')

# Generate object file paths by prepending BUILD_DIR and appending .o to source files
# Example: ./your_dir/hello.cpp turns into ./build/./your_dir/hello.cpp.o
TEENSY_OBJS := $(TEENSY_SRC:%=$(BUILD_DIR)/%.o)
LIBRARY_OBJS := $(LIBRARY_SRC:%=$(BUILD_DIR)/%.o)
SRC_OBJS :=  $(SRC_SRC:%=$(BUILD_DIR)/%.o)

# Generate dependency file paths by replacing .o with .d in object file paths
# Example: ./build/hello.cpp.o turns into ./build/hello.cpp.d
TEENSY_DEPS := $(TEENSY_OBJS:.o=.d)
LIBRARY_DEPS := $(LIBRARY_OBJS:.o=.d)
SRC_DEPS := $(SRC_OBJS:.o=.d)

# Find all include directories for GCC to locate header files
TEENSY_INC_DIRS := $(shell find $(TEENSY_SRC_DIRS) -type d)
LIBRARY_INC_DIRS := $(shell find $(LIBRARY_SRC_DIRS) -maxdepth 2 -type d)
SRC_INC_DIRS := $(shell find $(SRC_SRC_DIRS) -type d)

# Generate compiler include flags from include directories
TEENSY_INC_FLAGS := $(addprefix -I,$(TEENSY_INC_DIRS))
LIBRARY_INC_FLAGS := $(addprefix -I,$(LIBRARY_INC_DIRS))
SRC_INC_FLAGS := $(addprefix -I,$(SRC_INC_DIRS))
INCLUDE_FLAGS := $(TEENSY_INC_FLAGS) $(LIBRARY_INC_FLAGS) $(SRC_INC_FLAGS)

# Compiler flags specific to Teensy 4.1
TEENSY4_FLAGS = -DF_CPU=600000000 -DUSB_CUSTOM -DLAYOUT_US_ENGLISH -D__IMXRT1062__ -DTEENSYDUINO=159 -DARDUINO_TEENSY41 -DARDUINO=10813

# CPU flags to optimize code for the Teensy processor
CPU_CFLAGS = -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb

DEFINES := $(TEENSY4_FLAGS)

# Preprocessor flags for both C and C++ files
CPPFLAGS := $(INCLUDE_FLAGS) $(DEFINES) -MMD -MP -ffunction-sections -fdata-sections -O2

# Compiler flags for C files
CFLAGS := $(CPU_CFLAGS)

# Compiler flags for C++ files
CXXFLAGS := $(CPU_CFLAGS) -std=gnu++17 \
            -felide-constructors -fno-exceptions -fpermissive -fno-rtti \
            -Wno-error=narrowing -Wno-trigraphs -Wno-comment -Wall -Werror

# Linker flags, including Teensy-specific linker script
# --gc-sections: Remove unused sections to reduce binary size
# --relax: Allow linker to relax some constraints to sometimes generate smaller code
# -Tteensy4/imxrt1062_t41.ld: Use the Teensy 4.1 linker script
# --print-memory-usage: Print memory usage after linking
# -Map=... and --cref: Generate a cross-reference map file
LINKING_FLAGS = -Wl,--gc-sections,--relax,-Tteensy4/imxrt1062_t41.ld,--print-memory-usage,-Map=$(BUILD_DIR)/$(TARGET_EXEC).map,--cref

# Set the Arduino path based on the detected operating system
ifeq ($(UNAME),Darwin)
 ARDUINO_PATH = $(abspath $(HOME)/Library/Arduino15)
 $(info We've detected you are using a Mac! Consult God if this breaks.)
endif
ifeq ($(UNAME),Linux)
 ARDUINO_PATH = $(abspath $(HOME)/.arduino15)
 $(info We've detected you're on Linux! Nerd.)
endif

# Base arm-none-eabi and Teensyduino tool paths
COMPILER_TOOLS_PATH := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/11.3.1/arm/bin
TEENSYDUINO_TOOLS_PATH := $(ARDUINO_PATH)/packages/teensy/tools/teensy-tools/1.59.0

# arm-none-eabi tools
COMPILER_CPP	:= $(COMPILER_TOOLS_PATH)/arm-none-eabi-g++
COMPILER_C		:= $(COMPILER_TOOLS_PATH)/arm-none-eabi-gcc
AR				:= $(COMPILER_TOOLS_PATH)/arm-none-eabi-ar
GDB				:= $(COMPILER_TOOLS_PATH)/arm-none-eabi-gdb
OBJCOPY			:= $(COMPILER_TOOLS_PATH)/arm-none-eabi-objcopy
OBJDUMP			:= $(COMPILER_TOOLS_PATH)/arm-none-eabi-objdump
READELF			:= $(COMPILER_TOOLS_PATH)/arm-none-eabi-readelf
ADDR2LINE		:= $(COMPILER_TOOLS_PATH)/arm-none-eabi-addr2line
SIZE			:= $(COMPILER_TOOLS_PATH)/arm-none-eabi-size

# Teensyduino tools
TEENSY_SIZE		:= $(TEENSYDUINO_TOOLS_PATH)/teensy_size

# Path to the Git scraper tool source file
GIT_SCRAPER = $(TOOLS_DIR)/git_scraper.cpp

# Utilize all available CPU cores for parallel build
MAKEFLAGS += -j$(nproc)

# Phony target to force a build every time
.PHONY: build

# Main build target; depends on the target executable and git scraper
build: $(BUILD_DIR)/$(TARGET_EXEC)

# Final linking step to create the executable.
# This rule links all the object files to produce the final ELF executable.
# It depends on all object files and the 'git_scraper' target to ensure
# that Git information is up-to-date before linking.
$(BUILD_DIR)/$(TARGET_EXEC): git_scraper $(SRC_OBJS) $(LIBRARY_OBJS) $(TEENSY_OBJS) 
    # Invoke the C++ compiler as the linker.
    # - $(COMPILER_CPP): The compiler executable.
    # - $(CPPFLAGS): Preprocessor and common compiler flags.
    # - $(CXXFLAGS): C++ compiler flags.
    # - $(LIBRARY_OBJS), $(TEENSY_OBJS), $(SRC_OBJS): Object files to link.
    # - $(LINKING_FLAGS): Linker flags, including the linker script.
    # - '-o $(BUILD_DIR)/$(TARGET_EXEC).elf': Output the ELF executable to the build directory.
	@$(COMPILER_CPP) $(CPPFLAGS) $(CXXFLAGS) $(LIBRARY_OBJS) $(TEENSY_OBJS) $(SRC_OBJS) $(LINKING_FLAGS) -o $(BUILD_DIR)/$(TARGET_EXEC).elf

    # Copy the ELF executable to the root directory.
	@cp $(BUILD_DIR)/$(TARGET_EXEC).elf .

    # Inform the user that the HEX file is being constructed.
	@echo [Constructing $(TARGET_EXEC).hex]

    # Convert the ELF executable to an Intel HEX format, which is required for uploading to the Teensy board.
    # - '-O ihex': Specifies the output format as Intel HEX.
    # - '-R .eeprom': Removes the .eeprom section from the output, as it's not needed for flashing.
    # - '$(TARGET_EXEC).elf': The input ELF executable.
    # - '$(TARGET_EXEC).hex': The output HEX file.
	@$(OBJCOPY) -O ihex -R .eeprom $(TARGET_EXEC).elf $(TARGET_EXEC).hex

    # Ensure the HEX file has execute permissions.
	@chmod +x $(TARGET_EXEC).hex

    # Disassemble the ELF executable to a .dump file
	@$(OBJDUMP) -dstz $(BUILD_DIR)/$(TARGET_EXEC).elf > $(BUILD_DIR)/$(TARGET_EXEC).dump


# Build step for compiling C source files
$(BUILD_DIR)/%.c.o: %.c
	@mkdir -p $(dir $@)
	@echo [Building $<]
	@$(COMPILER_C) $(CPPFLAGS) $(CFLAGS) -c $< -o $@
    # Disassemble the object file to a .dump file
	@$(OBJDUMP) -dstz $@ > $@.dump

# Build step for compiling C++ source files
$(BUILD_DIR)/%.cpp.o: %.cpp
	@mkdir -p $(dir $@)
	@echo [Building $<]
	@$(COMPILER_CPP) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@
    # Disassemble the object file to a .dump file
	@$(OBJDUMP) -dstz $@ > $@.dump


# Phony target to prevent conflicts with files named 'clean' and force a rebuild every time
.PHONY: clean

# Clean the build directory and remove generated executables
clean:
	rm -rf $(BUILD_DIR)
	rm -rf $(TARGET_EXEC).elf $(TARGET_EXEC).hex $(TARGET_EXEC).map $(TARGET_EXEC).dump

# Clean only the source object files
clean_src:
	rm -r $(BUILD_DIR)/src

# Clean only the library object files
clean_libs:
	rm -r $(BUILD_DIR)/libraries

# Clean only the Teensy object files
clean_teensy4:
	rm -r $(BUILD_DIR)/teensy4

# Include the dependency files to manage header file dependencies
# The '-' suppresses errors if the files are missing (which they will be on the first run)
-include $(TEENSY_DEPS)
-include $(LIBRARY_DEPS)
-include $(SRC_DEPS)

# Build, run, and clean up the git scraper tool to store current Git info in a header file
git_scraper:
	@g++ -std=gnu++17 $(GIT_SCRAPER) -o $(TOOLS_DIR)/git_scraper
	@$(TOOLS_DIR)/git_scraper
	@rm $(TOOLS_DIR)/git_scraper


# Upload the firmware to the Teensy device
upload: build
	@echo [Uploading] - If this fails, press the button on the teensy and re-run 'make upload'
	@tycmd upload $(TARGET_EXEC).hex
    # Teensy serial isn't immediately available after upload, so we wait a bit
    # The Teensy waits for 20 + 280 + 20 ms after power up/boot
	@sleep 0.4s
	@bash $(TOOLS_DIR)/monitor.sh


# Install required tools for building and uploading firmware
install:
	@$(TOOLS_DIR)/install_tytools.sh
	@$(TOOLS_DIR)/install_arduino.sh


# starts GDB and attaches to the firmware running on a connected Teensy
# calls a script to prepare the GDB environment, this finds the exact port Teensy is connected to
gdb:
	@echo [Starting GDB]
	@bash $(TOOLS_DIR)/prepare_gdb.sh
	@$(GDB) -x $(TOOLS_DIR)/gdb_commands.txt --args $(TARGET_EXEC).elf


# monitors currently running firmware on robot
monitor:
	@echo [Monitoring]
	@bash $(TOOLS_DIR)/monitor.sh


# resets teensy and switches it into boot-loader mode, effectively stopping any execution
# this only works if power is consistent, will restart loaded firmware if turned off and on again
kill:
	@echo [Attempting to Kill Teensy]
	@tycmd reset -b


# restarts teensy
restart:
	@echo [Attempting to Restart Firmware]
	@tycmd reset


help: 
	@echo "Basic usage: make [target]"
	@echo "Targets:"
	@echo "  install:      installs all required dependencies"
	@echo "  build:        compiles the source code and links with libraries"
	@echo "  upload:       builds the source and uploads it to the Teensy"
	@echo "  gdb:          starts GDB and attaches to the firmware running on a connected Teensy"
	@echo "  monitor:      monitors any actively running firmware and displays serial output"
	@echo "  kill:         stops any running firmware"
	@echo "  restart:      restarts any running firmware"
