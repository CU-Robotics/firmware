# Made by Jack Miller 2024 (Github: guywithhat99)
# Thanks to Jackson Stepka (Github: Pandabear1125); a lot of code is based off his original makefile

# Detect the current operating system using uname
UNAME := $(shell uname -s)

TARGET := firmware
TARGET_ELF := $(TARGET).elf
TARGET_HEX := $(TARGET).hex
TARGET_MAP := $(TARGET).map
TARGET_DUMP := $(TARGET).dump

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
# -isystem on Teensy and Library files to suppress warnings
TEENSY_INC_FLAGS := $(addprefix -isystem,$(TEENSY_INC_DIRS))
LIBRARY_INC_FLAGS := $(addprefix -isystem,$(LIBRARY_INC_DIRS))
SRC_INC_FLAGS := $(addprefix -I,$(SRC_INC_DIRS))
INCLUDE_FLAGS := $(TEENSY_INC_FLAGS) $(LIBRARY_INC_FLAGS) $(SRC_INC_FLAGS)

# Compiler flags specific to Teensy 4.1
TEENSY4_FLAGS = -DF_CPU=600000000 -DUSB_CUSTOM -DLAYOUT_US_ENGLISH -D__IMXRT1062__ -DTEENSYDUINO=159 -DARDUINO_TEENSY41 -DARDUINO=10813 -DFIRMWARE

# CPU flags to optimize code for the Teensy processor
CPU_CFLAGS = -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb

DEFINES := $(TEENSY4_FLAGS)

# Preprocessor flags for both C and C++ files
# -MMD: Generate dependency files for each source file
# -MP: Add a phony target for each dependency to avoid errors if the dependency is missing
# -ffunction-sections: Place each function in its own section to allow the linker to remove unused functions
# -fdata-sections: Place each variable in its own section to allow the linker to remove unused variables
# -O2: Optimize the code for speed
# --specs=nano.specs: Use newlib nano instead of full newlib to reduce binary size
# -g3: Generate debug information for GDB. Level 3 includes the most information possible
CPPFLAGS := $(INCLUDE_FLAGS) $(DEFINES) -MMD -MP -ffunction-sections -fdata-sections -O2 --specs=nano.specs -g3

# Compiler flags for C files
CFLAGS := $(CPU_CFLAGS)

# Compiler flags for C++ files
CXXFLAGS := $(CPU_CFLAGS) -std=gnu++23 \
			-felide-constructors -fno-exceptions -fpermissive \
			-Wno-error=narrowing -Wno-trigraphs -Wno-comment -Wall -Werror \
			-Wno-volatile

# Linker flags, including Teensy-specific linker script
# --gc-sections: Remove unused sections to reduce binary size
# --relax: Allow linker to relax some constraints to sometimes generate smaller code
# -Tteensy4/imxrt1062_t41.ld: Use the Teensy 4.1 linker script
# --print-memory-usage: Print memory usage after linking
# -Map=... and --cref: Generate a cross-reference map file
LINKING_FLAGS = -Wl,--gc-sections,--relax,-Tteensy4/imxrt1062_t41.ld,--print-memory-usage,-Map=$(BUILD_DIR)/$(TARGET_MAP),--cref

# Base arm-none-eabi and Teensyduino tool paths
COMPILER_TOOLS_PATH = $(TOOLS_DIR)/compiler/arm-gnu-toolchain/bin
TARGET_TRIPLE ?= arm-none-eabi

# arm-none-eabi tools
COMPILER_CPP	= $(COMPILER_TOOLS_PATH)/arm-none-eabi-g++
COMPILER_C		= $(COMPILER_TOOLS_PATH)/arm-none-eabi-gcc
AR				= $(COMPILER_TOOLS_PATH)/arm-none-eabi-ar
GDB				= $(COMPILER_TOOLS_PATH)/arm-none-eabi-gdb
OBJCOPY			= $(COMPILER_TOOLS_PATH)/arm-none-eabi-objcopy
OBJDUMP			= $(COMPILER_TOOLS_PATH)/arm-none-eabi-objdump
READELF			= $(COMPILER_TOOLS_PATH)/arm-none-eabi-readelf
ADDR2LINE		= $(COMPILER_TOOLS_PATH)/arm-none-eabi-addr2line
SIZE			= $(COMPILER_TOOLS_PATH)/arm-none-eabi-size

# Path to the Git scraper tool source file
GIT_SCRAPER = $(TOOLS_DIR)/git_scraper.cpp

# Utilize all available CPU cores for parallel build
MAKEFLAGS += -j$(nproc)

.PHONY: build docs clean upload install gdb monitor kill restart help clangd git_scraper

build:	clangd $(BUILD_DIR)/$(TARGET_ELF)
$(BUILD_DIR)/$(TARGET_ELF): git_scraper $(SRC_OBJS) $(LIBRARY_OBJS) $(TEENSY_OBJS)
	@$(COMPILER_CPP) $(CPPFLAGS) $(CXXFLAGS) $(LIBRARY_OBJS) $(TEENSY_OBJS) $(SRC_OBJS) $(LINKING_FLAGS) -o $(BUILD_DIR)/$(TARGET_ELF)
	@echo [Constructing $(TARGET_HEX)]
	@$(OBJCOPY) -O ihex -R .eeprom $(BUILD_DIR)/$(TARGET_ELF) $(BUILD_DIR)/$(TARGET_HEX)
	@chmod +x $(BUILD_DIR)/$(TARGET_HEX)
	@$(OBJDUMP) -dstz $(BUILD_DIR)/$(TARGET_ELF) > $(BUILD_DIR)/$(TARGET_DUMP)

# Ensure git_scraper finishes before compiling any object files
$(SRC_OBJS) $(LIBRARY_OBJS) $(TEENSY_OBJS): | git_scraper
# Build step for compiling C source files
$(BUILD_DIR)/%.c.o: %.c
	@mkdir -p $(dir $@)
	@echo [Building $<]
	@$(COMPILER_C) $(CPPFLAGS) $(CFLAGS) -c $< -o $@
	@$(OBJDUMP) -dstz $@ > $@.dump

# Build step for compiling C++ source files
$(BUILD_DIR)/%.cpp.o: %.cpp
	@mkdir -p $(dir $@)
	@echo [Building $<]
	@$(COMPILER_CPP) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@
	@$(OBJDUMP) -dstz $@ > $@.dump



DOC_SCRIPT = $(TOOLS_DIR)/build_docs.sh

docs: build
	@chmod +x $(DOC_SCRIPT)
	@$(DOC_SCRIPT)


# Clean the build directory and remove generated executables
clean:
	rm -rf $(BUILD_DIR)
	rm -f compile_commands.json

# Include the dependency files to manage header file dependencies
# The '-' suppresses errors if the files are missing (which they will be on the first run)
-include $(TEENSY_DEPS)
-include $(LIBRARY_DEPS)
-include $(SRC_DEPS)

git_scraper:
	@g++ -std=gnu++17 $(GIT_SCRAPER) -o $(TOOLS_DIR)/git_scraper
	@$(TOOLS_DIR)/git_scraper
	@rm $(TOOLS_DIR)/git_scraper


# Upload the firmware to the Teensy device
upload: build
	@echo [Uploading] - If this fails, press the button on the teensy and re-run 'make upload'
	@tycmd upload $(BUILD_DIR)/$(TARGET_HEX)
	@sleep 0.4s
	@bash $(TOOLS_DIR)/monitor.sh


# Install required tools for building and uploading firmware
install:
	@bash $(TOOLS_DIR)/install_tytools.sh
	@bash $(TOOLS_DIR)/install_compiler.sh


# starts GDB and attaches to the firmware running on a connected Teensy
# calls a script to prepare the GDB environment, this finds the exact port Teensy is connected to
gdb:
	@echo [Starting GDB]
	@bash $(TOOLS_DIR)/prepare_gdb.sh
	@$(GDB) -x $(TOOLS_DIR)/gdb_commands.txt --args $(BUILD_DIR)/$(TARGET_ELF)


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
	@echo "  install:       installs all required dependencies"
	@echo "  build:         compiles the source code and links with libraries"
	@echo "  upload:        builds the source and uploads it to the Teensy"
	@echo "  gdb:           starts GDB and attaches to the firmware running on a connected Teensy"
	@echo "  monitor:       monitors any actively running firmware and displays serial output"
	@echo "  kill:          stops any running firmware"
	@echo "  restart:       restarts any running firmware"
	@echo "  clean:         removes all build artifacts and generated files"
	@echo "  clangd:        generates compile_commands.json for clangd"
	@echo "  docs:          generates documentation of our src/ code using Doxygen"


# Generate compile_commands.json from the Makefile's flags and source lists.
COMPILE_DB = compile_commands.json
COMPILE_DB_SCRIPT = $(TOOLS_DIR)/generate_compile_commands.sh
COMPILE_DB_DIR_DEPS = $(SRC_INC_DIRS) $(LIBRARY_INC_DIRS) $(TEENSY_INC_DIRS)

clangd: $(COMPILE_DB)

$(COMPILE_DB): Makefile $(COMPILE_DB_SCRIPT) $(COMPILE_DB_DIR_DEPS)
	@CURDIR="$(CURDIR)" \
	BUILD_DIR="$(BUILD_DIR)" \
	COMPILER_CPP="$(COMPILER_CPP)" \
	COMPILER_C="$(COMPILER_C)" \
	TARGET_TRIPLE="$(TARGET_TRIPLE)" \
	CPPFLAGS='$(CPPFLAGS)' \
	CXXFLAGS='$(CXXFLAGS)' \
	CFLAGS='$(CFLAGS)' \
	SRC_FILES='$(SRC_SRC) $(LIBRARY_SRC) $(TEENSY_SRC)' \
	"$(COMPILE_DB_SCRIPT)"
