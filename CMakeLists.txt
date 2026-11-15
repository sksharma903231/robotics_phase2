cmake_minimum_required(VERSION 3.16)
project(RoboticsPhase2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Executables
add_executable(lidar_node src/lidar_node.cpp)
add_executable(motor_controller src/motor_controller.cpp)

# Testing
enable_testing()
add_executable(test_rover_logic tests/test_rover_logic.cpp)
add_test(NAME test_rover_logic COMMAND test_rover_logic)