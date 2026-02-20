cmake_minimum_required(VERSION 3.14)
project(RoboticArm VERSION 0.1.0 LANGUAGES C CXX)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable testing
enable_testing()

# Directories
add_subdirectory(hardware)
# add_subdirectory(application_layer)
# add_subdirectory(kinematics)
add_subdirectory(math)
