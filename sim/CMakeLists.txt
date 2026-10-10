cmake_minimum_required(VERSION 3.16)
project(firmware_turret_sim LANGUAGES CXX)

# No UNIT_TEST: compile actual production controllers and estimators, replacing
# only hardware I/O headers via the explicit host include boundary.
add_library(firmware_turret_sim STATIC
    bridge.cpp
    ../src/controls/controller.cpp
    ../src/controls/estimator.cpp
    ../src/controls/state.cpp
    ../src/controls/robot_state_array.cpp
    ../src/controls/reference_governor.cpp
    ../src/filters/pid_filter.cpp
    ../src/utils/vector_math.cpp
    ../src/utils/wrapping.cpp)
target_compile_features(firmware_turret_sim PUBLIC cxx_std_23)
target_include_directories(firmware_turret_sim PRIVATE host ../src)
set_target_properties(firmware_turret_sim PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(firmware_turret_sim PRIVATE -Wall -Wextra -Wno-unused-parameter)

add_executable(firmware_bridge_smoke smoke.cpp)
target_link_libraries(firmware_bridge_smoke PRIVATE firmware_turret_sim)
