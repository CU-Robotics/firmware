# Made by Jack Miller 2024 (Github: guywithhat99)
# Thanks to Jackson Stepka (Github: Pandabear1125) a lot of code is based off his original makefile 


# Use uname to detect current OS
UNAME := $(shell uname -s)

TARGET_EXEC := firmware

BUILD_DIR := ./build

TEENSY_SRC_DIRS := ./teensy4
LIBRARY_SRC_DIRS := ./libraries
SRC_SRC_DIRS := ./src

# Find all the C and C++ files we want to compile
# Note the single quotes around the * expressions. The shell will incorrectly expand these otherwise, but we want to send the * directly to the find command.
TEENSY_SRC := $(shell find $(TEENSY_SRC_DIRS) -name '*.cpp' -or -name '*.c' -or -name '*.s')
LIBRARY_SRC := $(shell find $(LIBRARY_SRC_DIRS) -name '*.cpp' -or -name '*.c' -or -name '*.s')
SRC_SRC := $(shell find $(SRC_SRC_DIRS) -name '*.cpp' -or -name '*.c' -or -name '*.s')


# Prepends BUILD_DIR and appends .o to every src file
# As an example, ./your_dir/hello.cpp turns into ./build/./your_dir/hello.cpp.o
TEENSY_OBJS := $(TEENSY_SRC:%=$(BUILD_DIR)/%.o)
LIBRARY_OBJS := $(LIBRARY_SRC:%=$(BUILD_DIR)/%.o)
SRC_OBJS :=  $(SRC_SRC:%=$(BUILD_DIR)/%.o)


# String substitution (suffix version without %).
# As an example, ./build/hello.cpp.o turns into ./build/hello.cpp.d
TEENSY_DEPS := $(TEENSY_OBJS:.o=.d)
LIBRARY_DEPS := $(LIBRARY_OBJS:.o=.d)
SRC_DEPS := $(SRC_OBJS:.o=.d)


# Every folder will need to be passed to GCC so that it can find header files
TEENSY_INC_DIRS := $(shell find $(TEENSY_SRC_DIRS) -type d )
LIBRARY_INC_DIRS := $(shell find $(LIBRARY_SRC_DIRS) -maxdepth 2 -type d )  
SRC_INC_DIRS := $(shell find $(SRC_SRC_DIRS) -type d) 

# Include directories
TEENSY_INC_FLAGS := $(addprefix -I,$(TEENSY_INC_DIRS))
LIBRARY_INC_FLAGS := $(addprefix -I,$(LIBRARY_INC_DIRS))
SRC_INC_FLAGS := $(addprefix -I,$(SRC_INC_DIRS))
INCLUDE_FLAGS := $(TEENSY_INC_FLAGS) $(LIBRARY_INC_FLAGS) $(SRC_INC_FLAGS)

# Teensy41 compiler flags
TEENSY4_FLAGS = -DF_CPU=600000000 -DUSB_CUSTOM -DLAYOUT_US_ENGLISH -D__IMXRT1062__ -DTEENSYDUINO=159 -DARDUINO_TEENSY41 -DARDUINO=10813

# CPU flags to tailor the code for the Teensy processor
CPU_CFLAGS = -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb


# Defines
DEFINES := $(TEENSY4_FLAGS)

# Preprocessor flags
CPPFLAGS := $(INCLUDE_FLAGS) $(DEFINES) -MMD -MP -ffunction-sections -fdata-sections -O2

# Compiler flags
CFLAGS := $(CPU_CFLAGS)
CXXFLAGS := $(CPU_CFLAGS) -std=gnu++17 \
            -felide-constructors -fno-exceptions -fpermissive -fno-rtti \
            -Wno-error=narrowing -Wno-trigraphs -Wno-comment

# Required linker config for teensy related things
LINKING_FLAGS =-Wl,--gc-sections,--relax,-Tteensy4/imxrt1062_t41.ld


ifeq ($(UNAME),Darwin)
 ARDUINO_PATH = $(abspath $(HOME)/Library/Arduino15)
 $(info We've detected you are using a Mac! Consult God if this breaks.)
endif
ifeq ($(UNAME),Linux)
 ARDUINO_PATH = $(abspath $(HOME)/.arduino15)
 $(info We've detected you're on Linux! Nerd.)
endif

# Complete compilers
COMPILER_CPP:= $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/*/arm/bin/arm-none-eabi-g++
COMPILER_C := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/*/arm/bin/arm-none-eabi-gcc
OBJCOPY := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/*/arm/bin/arm-none-eabi-objcopy
GDB := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/*/arm/bin/arm-none-eabi-gdb
SIZE := $(ARDUINO_PATH)/packages/teensy/tools/teensy-tools/1.59.0/teensy_size
ARM_SIZE := $(ARDUINO_PATH)/packages/teensy/tools/teensy-compile/*/arm/bin/arm-none-eabi-size

GIT_SCRAPER = ./tools/git_scraper.cpp

MAKEFLAGS += -j$(nproc)

.PHONY: all

all: $(BUILD_DIR)/$(TARGET_EXEC) git_scraper

#This line will output a list of memory sections with sizes
#	$(ARM_SIZE) -A $(BUILD_DIR)/$(TARGET_EXEC).elf

# The final build step.
$(BUILD_DIR)/$(TARGET_EXEC): $(SRC_OBJS) $(LIBRARY_OBJS) $(TEENSY_OBJS)  git_scraper
	 $(COMPILER_CPP) $(CPPFLAGS) $(CXXFLAGS) $(LIBRARY_OBJS) $(TEENSY_OBJS) $(SRC_OBJS) $(LINKING_FLAGS) -o $(BUILD_DIR)/$(TARGET_EXEC).elf


$(SRC_OBJS): $(LIBRARY_OBJS)

$(LIBRARY_OBJS): $(TEENSY_OBJS)

# Build step for C source
$(BUILD_DIR)/%.c.o: %.c
	mkdir -p $(dir $@)
	$(COMPILER_C) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# Build step for C++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	mkdir -p $(dir $@)
	$(COMPILER_CPP) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	rm -r $(BUILD_DIR)

# Include the .d makefiles. The - at the front suppresses the errors of missing
# Makefiles. Initially, all the .d files will be missing, and we don't want those
# errors to show up.
-include $(TEENSY_DEPS)
-include $(LIBRARY_DEPS)
-include $(SRC_DEPS)


# compiles, runs, and cleans up the git_scraper tool which stores the current git info in a header file
git_scraper:
	@g++ $(GIT_SCRAPER) -o ./tools/git_scraper
	@./tools/git_scraper
	@rm ./tools/git_scraper