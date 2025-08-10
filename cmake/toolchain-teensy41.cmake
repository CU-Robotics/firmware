# Teensy 4.1 (IMXRT1062) cross-compilation toolchain for CMake

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m7)

# Avoid try-run/linking during compiler checks for bare-metal
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Allow overriding toolchain root via env or cache
set(TEENSY_GCC_ROOT "${CMAKE_SOURCE_DIR}/tools/compiler/arm-gnu-toolchain/bin" CACHE PATH "Path to arm-none-eabi toolchain bin")
if(DEFINED ENV{TEENSY_GCC_ROOT})
  set(TEENSY_GCC_ROOT "$ENV{TEENSY_GCC_ROOT}")
endif()

find_program(ARM_CC NAMES arm-none-eabi-gcc HINTS ${TEENSY_GCC_ROOT})
find_program(ARM_CXX NAMES arm-none-eabi-g++ HINTS ${TEENSY_GCC_ROOT})
find_program(ARM_AR NAMES arm-none-eabi-ar HINTS ${TEENSY_GCC_ROOT})
find_program(ARM_RANLIB NAMES arm-none-eabi-ranlib HINTS ${TEENSY_GCC_ROOT})
find_program(ARM_OBJCOPY NAMES arm-none-eabi-objcopy HINTS ${TEENSY_GCC_ROOT})
find_program(ARM_OBJDUMP NAMES arm-none-eabi-objdump HINTS ${TEENSY_GCC_ROOT})
find_program(ARM_SIZE NAMES arm-none-eabi-size HINTS ${TEENSY_GCC_ROOT})

if(ARM_CC)
  set(CMAKE_C_COMPILER ${ARM_CC} CACHE FILEPATH "" FORCE)
  set(CMAKE_ASM_COMPILER ${ARM_CC} CACHE FILEPATH "" FORCE)
endif()
if(ARM_CXX)
  set(CMAKE_CXX_COMPILER ${ARM_CXX} CACHE FILEPATH "" FORCE)
endif()
if(ARM_AR)
  set(CMAKE_AR ${ARM_AR} CACHE FILEPATH "" FORCE)
endif()
if(ARM_RANLIB)
  set(CMAKE_RANLIB ${ARM_RANLIB} CACHE FILEPATH "" FORCE)
endif()
if(ARM_OBJCOPY)
  set(CMAKE_OBJCOPY ${ARM_OBJCOPY} CACHE FILEPATH "" FORCE)
endif()
if(ARM_OBJDUMP)
  set(CMAKE_OBJDUMP ${ARM_OBJDUMP} CACHE FILEPATH "" FORCE)
endif()
if(ARM_SIZE)
  set(CMAKE_SIZE ${ARM_SIZE} CACHE FILEPATH "" FORCE)
endif()

# Use newlib-nano by default
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nano.specs")

# Do not search host paths
set(CMAKE_FIND_ROOT_PATH "")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE NEVER)


