# Require minimum CMake version
cmake_minimum_required(VERSION 3.0) 

# Set project name 
project(OpenCV_Project)

# Find OpenCV packages
find_package(OpenCV REQUIRED)

# Add executable for displayImage and meanFilter programs
add_executable(displayImage displayImage.cpp) 
add_executable(meanFilter meanFilter.cpp)

# Link display executable to OpenCV libraries
target_link_libraries (displayImage ${OpenCV_LIBS})
target_link_libraries (meanFilter ${OpenCV_LIBS})

# Set build type to debug
set(CMAKE_BUILD_TYPE Debug)
