# Optional features: PROFILER, COMMS_DEBUG, REF_SYSTEM_DEBUG, CAN_MANAGER_DEBUG
# Example: make clean build FEATURE_DEFINES="-DPROFILER -DCOMMS_DEBUG"
FEATURE_DEFINES ?=

BUILD_DIR := build
TOOLS_DIR := tools

TARGET := firmware
TARGET_ELF := $(BUILD_DIR)/$(TARGET).elf
TARGET_HEX := $(BUILD_DIR)/$(TARGET).hex
TARGET_MAP := $(BUILD_DIR)/$(TARGET).map
TARGET_DUMP := $(BUILD_DIR)$(TARGET).dump

TEENSY_SRC_DIRS := teensy4
LIBRARY_SRC_DIRS := libraries
SRC_SRC_DIRS := src

TEENSY_SRC := $(shell find $(TEENSY_SRC_DIRS) -name '*.cpp' -or -name '*.c')
LIBRARY_SRC := $(shell find $(LIBRARY_SRC_DIRS) -name '*.cpp' -or -name '*.c')
SRC_SRC := $(shell find $(SRC_SRC_DIRS) -name '*.cpp' -or -name '*.c')

TEENSY_OBJS := $(TEENSY_SRC:%=$(BUILD_DIR)/%.o)
LIBRARY_OBJS := $(LIBRARY_SRC:%=$(BUILD_DIR)/%.o)
SRC_OBJS :=  $(SRC_SRC:%=$(BUILD_DIR)/%.o)

TEENSY_DEPS := $(TEENSY_OBJS:.o=.d)
LIBRARY_DEPS := $(LIBRARY_OBJS:.o=.d)
SRC_DEPS := $(SRC_OBJS:.o=.d)

TEENSY_INC_DIRS := $(shell find $(TEENSY_SRC_DIRS) -type d)
LIBRARY_INC_DIRS := $(LIBRARY_SRC_DIRS) \
	$(wildcard $(LIBRARY_SRC_DIRS)/*/) \
	$(wildcard $(LIBRARY_SRC_DIRS)/*/src/) \
	$(wildcard $(LIBRARY_SRC_DIRS)/*/utility/)
SRC_INC_DIRS := $(SRC_SRC_DIRS)

# -isystem on Teensy and Library files to suppress warnings
TEENSY_INC_FLAGS := $(addprefix -isystem,$(TEENSY_INC_DIRS))
LIBRARY_INC_FLAGS := $(addprefix -isystem,$(LIBRARY_INC_DIRS))
SRC_INC_FLAGS := $(addprefix -I,$(SRC_INC_DIRS))
INCLUDE_FLAGS := $(TEENSY_INC_FLAGS) $(LIBRARY_INC_FLAGS) $(SRC_INC_FLAGS)

# CPU and ABI flags required during compilation and linking
ARCH_FLAGS := -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb

BOARD_DEFINES := -DF_CPU=600000000 -D__IMXRT1062__ -DTEENSYDUINO=159 -DARDUINO_TEENSY41 -DARDUINO=10813
USB_DEFINES := -DUSB_CUSTOM -DLAYOUT_US_ENGLISH
PROJECT_DEFINES := $(BOARD_DEFINES) $(USB_DEFINES) -DFIRMWARE $(FEATURE_DEFINES)

# -MMD: Generate dependency files for each source file
# -MP: Add a phony target for each dependency to avoid errors if the dependency is missing
DEPFLAGS := -MMD -MP

# Place each function and variable in its own section so the linker can remove unused code and data
SECTION_FLAGS := -ffunction-sections -fdata-sections
COMMON_COMPILE_FLAGS := $(ARCH_FLAGS) $(SECTION_FLAGS) -O2 -g3 --specs=nano.specs
CXX_LANGUAGE_FLAGS := -std=gnu++23 -felide-constructors -fno-exceptions -fpermissive
CXX_WARNING_FLAGS := -Wno-error=narrowing -Wno-trigraphs -Wno-comment -Wall -Werror -Wno-volatile

PROJECT_CPPFLAGS := $(INCLUDE_FLAGS) $(PROJECT_DEFINES)
PROJECT_CFLAGS := $(COMMON_COMPILE_FLAGS)
PROJECT_CXXFLAGS := $(COMMON_COMPILE_FLAGS) $(CXX_LANGUAGE_FLAGS) $(CXX_WARNING_FLAGS)

# --gc-sections: Remove unused sections to reduce binary size
# --relax: Allow linker to relax some constraints to sometimes generate smaller code
# -Tteensy4/imxrt1062_t41.ld: Use the Teensy 4.1 linker script
# -Map=... and --cref: Generate a cross-reference map file
PROJECT_LDFLAGS := $(ARCH_FLAGS) --specs=nano.specs -Wl,--gc-sections,--relax,-Tteensy4/imxrt1062_t41.ld,--print-memory-usage,-Map=$(TARGET_MAP),--cref

COMPILER_TOOLS_PATH = $(TOOLS_DIR)/compiler/arm-gnu-toolchain/bin
TARGET_TRIPLE ?= arm-none-eabi

COMPILER_CPP	= $(COMPILER_TOOLS_PATH)/arm-none-eabi-g++
COMPILER_C		= $(COMPILER_TOOLS_PATH)/arm-none-eabi-gcc
AR				= $(COMPILER_TOOLS_PATH)/arm-none-eabi-ar
GDB				= $(COMPILER_TOOLS_PATH)/arm-none-eabi-gdb
OBJCOPY			= $(COMPILER_TOOLS_PATH)/arm-none-eabi-objcopy
OBJDUMP			= $(COMPILER_TOOLS_PATH)/arm-none-eabi-objdump
READELF			= $(COMPILER_TOOLS_PATH)/arm-none-eabi-readelf
ADDR2LINE		= $(COMPILER_TOOLS_PATH)/arm-none-eabi-addr2line
SIZE			= $(COMPILER_TOOLS_PATH)/arm-none-eabi-size

GIT_SCRAPER = $(TOOLS_DIR)/git_scraper.cpp


.PHONY: build docs clean upload install gdb monitor kill restart help clangd git_scraper


build: clangd $(TARGET_ELF)

$(TARGET_ELF): git_scraper $(SRC_OBJS) $(LIBRARY_OBJS) $(TEENSY_OBJS)
	@printf "LINK     %s\n" "$@"
	@$(COMPILER_CPP) $(PROJECT_LDFLAGS) $(LDFLAGS) $(LIBRARY_OBJS) $(TEENSY_OBJS) $(SRC_OBJS) $(LDLIBS) -o $@

	@printf "OBJCOPY  %s\n" "$(TARGET_HEX)"
	@$(OBJCOPY) -O ihex -R .eeprom $@ $(TARGET_HEX)
	@chmod +x $(TARGET_HEX)

	@printf "OBJDUMP  %s\n" "$(TARGET_DUMP)"
	@$(OBJDUMP) -dstz $@ > $(TARGET_DUMP)


# Ensure git_scraper finishes before compiling any object files
$(SRC_OBJS) $(LIBRARY_OBJS) $(TEENSY_OBJS): | git_scraper


$(BUILD_DIR)/%.c.o: %.c
	@mkdir -p $(dir $@)
	@printf "CC       %s\n" "$<"
	@$(COMPILER_C) $(PROJECT_CPPFLAGS) $(CPPFLAGS) $(PROJECT_CFLAGS) $(CFLAGS) $(DEPFLAGS) -c $< -o $@
	@$(OBJDUMP) -dstz $@ > $@.dump


$(BUILD_DIR)/%.cpp.o: %.cpp
	@mkdir -p $(dir $@)
	@printf "CXX      %s\n" "$<"
	@$(COMPILER_CPP) $(PROJECT_CPPFLAGS) $(CPPFLAGS) $(PROJECT_CXXFLAGS) $(CXXFLAGS) $(DEPFLAGS) -c $< -o $@
	@$(OBJDUMP) -dstz $@ > $@.dump

DOC_SCRIPT = $(TOOLS_DIR)/build_docs.sh


docs: build
	@chmod +x $(DOC_SCRIPT)
	@$(DOC_SCRIPT)


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


# Upload firmware to Teensy and start monitoring
upload: build
	@echo [Uploading] - If this fails, press the button on the teensy and re-run 'make upload'
	@tycmd upload $(TARGET_HEX)
	@sleep 0.4s
	@bash $(TOOLS_DIR)/monitor.sh


# Install requirements for building and uploading firmware
install:
	@bash $(TOOLS_DIR)/install_tytools.sh
	@bash $(TOOLS_DIR)/install_compiler.sh


# starts GDB and attaches to the firmware running on a connected Teensy
# calls a script to prepare the GDB environment, this finds the exact port Teensy is connected to
gdb:
	@echo [Starting GDB]
	@bash $(TOOLS_DIR)/prepare_gdb.sh
	@$(GDB) -x $(TOOLS_DIR)/gdb_commands.txt --args $(TARGET_ELF)


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
	CPPFLAGS='$(PROJECT_CPPFLAGS) $(CPPFLAGS)' \
	CXXFLAGS='$(PROJECT_CXXFLAGS) $(CXXFLAGS)' \
	CFLAGS='$(PROJECT_CFLAGS) $(CFLAGS)' \
	SRC_FILES='$(SRC_SRC) $(LIBRARY_SRC) $(TEENSY_SRC)' \
	"$(COMPILE_DB_SCRIPT)"
	@printf "CLANGD   compile_commands.json generated\n"
