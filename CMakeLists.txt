cmake_minimum_required(VERSION 3.16)

project(firmware C CXX ASM)

# Always write compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Collect sources
file(GLOB_RECURSE SRC_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_SOURCE_DIR}/src/*.c
  ${CMAKE_SOURCE_DIR}/src/*.cpp
)
file(GLOB_RECURSE LIB_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_SOURCE_DIR}/libraries/*.c
  ${CMAKE_SOURCE_DIR}/libraries/*.cpp
)
file(GLOB_RECURSE TEENSY_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_SOURCE_DIR}/teensy4/*.c
  ${CMAKE_SOURCE_DIR}/teensy4/*.cpp
)

add_executable(firmware
  ${SRC_SOURCES}
  ${LIB_SOURCES}
  ${TEENSY_SOURCES}
)

# C++ standard and extensions
set_target_properties(firmware PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS ON # gnu++23
)

# Include directories (mark external as SYSTEM to quiet warnings)
target_include_directories(firmware
  PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

# Add top-level external include dirs as SYSTEM
target_include_directories(firmware SYSTEM
  PRIVATE
    ${CMAKE_SOURCE_DIR}/teensy4
    ${CMAKE_SOURCE_DIR}/libraries
)

# Add library nested include dirs (depth 2) as SYSTEM to match Makefile behavior
file(GLOB LIB_DIRS_1 LIST_DIRECTORIES true ${CMAKE_SOURCE_DIR}/libraries/*)
file(GLOB LIB_DIRS_2 LIST_DIRECTORIES true ${CMAKE_SOURCE_DIR}/libraries/*/*)
set(ALL_LIB_INC_DIRS)
foreach(dir ${LIB_DIRS_1} ${LIB_DIRS_2})
  if(IS_DIRECTORY ${dir})
    list(APPEND ALL_LIB_INC_DIRS ${dir})
  endif()
endforeach()
if(ALL_LIB_INC_DIRS)
  target_include_directories(firmware SYSTEM PRIVATE ${ALL_LIB_INC_DIRS})
endif()

# Preprocessor definitions (from Makefile TEENSY4_FLAGS)
target_compile_definitions(firmware PRIVATE
  F_CPU=600000000
  USB_CUSTOM
  LAYOUT_US_ENGLISH
  __IMXRT1062__
  TEENSYDUINO=159
  ARDUINO_TEENSY41
  ARDUINO=10813
  FIRMWARE
)

# CPU flags (from Makefile CPU_CFLAGS)
set(CPU_FLAGS -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb)

target_compile_options(firmware PRIVATE
  ${CPU_FLAGS}
  # Common compiler flags (from CPPFLAGS/CXXFLAGS)
  -MMD -MP -ffunction-sections -fdata-sections -O2 -g3
  # C++ specific
  $<$<COMPILE_LANGUAGE:CXX>:-felide-constructors>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
  $<$<COMPILE_LANGUAGE:CXX>:-fpermissive>
  $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=narrowing>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-trigraphs>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-comment>
  $<$<COMPILE_LANGUAGE:CXX>:-Wall>
  $<$<COMPILE_LANGUAGE:CXX>:-Werror>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-volatile>
)

# Linker flags (from Makefile LINKING_FLAGS) and nano.specs
target_link_options(firmware PRIVATE
  ${CPU_FLAGS}
  --specs=nano.specs
  -Wl,--gc-sections
  -Wl,--relax
  -Wl,-T${CMAKE_SOURCE_DIR}/teensy4/imxrt1062_t41.ld
  -Wl,--print-memory-usage
  -Wl,-Map,${CMAKE_BINARY_DIR}/firmware.map
  -Wl,--cref
)

# Optional: generate .hex after linking (objcopy path comes from toolchain file)
if(DEFINED CMAKE_OBJCOPY)
  add_custom_command(TARGET firmware POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex -R .eeprom $<TARGET_FILE:firmware> ${CMAKE_BINARY_DIR}/firmware.hex
    BYPRODUCTS ${CMAKE_BINARY_DIR}/firmware.hex
    COMMENT "Generating firmware.hex with objcopy"
  )
endif()


