# Require minimum CMake version
cmake_minimum_required(VERSION 3.0) 

# Set project name 
project(OpenCV_Project)

# Find OpenCV packages
find_package(OpenCV REQUIRED)

# Add executable for displayImage and meanFilter programs
add_executable(segment segment.cpp) 

# Link display executable to OpenCV libraries
target_link_libraries (segment ${OpenCV_LIBS})

# Set build type to debug
set(CMAKE_BUILD_TYPE Debug)

# Matplotlib
# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
# target_include_directories(segment PRIVATE ${MATPLOTLIB_CPP_INCLUDE_DIRS})

# Cmake according to the following steps
# mkdir build
# cd build
# cmake ..
# make