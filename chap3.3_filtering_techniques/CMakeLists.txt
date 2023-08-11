# Require minimum CMake version
cmake_minimum_required(VERSION 3.0) 

message(STATUS "Current source dir: ${CMAKE_CURRENT_SOURCE_DIR}")

# Set project name 
project(OpenCV_Project)

# Find OpenCV packages
find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/include)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Add Definitions
add_definitions(${PCL_DEFINITIONS})

# Add executable for displayImage and meanFilter programs
add_executable(mainFilter mainFilter.cpp src/color.cpp src/filter.cpp)

# Link display executable to OpenCV libraries
target_link_libraries (mainFilter ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

# Set build type to debug
set(CMAKE_BUILD_TYPE Debug)
