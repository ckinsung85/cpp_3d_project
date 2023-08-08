# Require minimum CMake version
cmake_minimum_required(VERSION 3.0) 

# Set project name 
project(OpenCV_Project)

# Find OpenCV packages
find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)

# Include directories
include_directories(${PCL_INCLUDE_DIRS})

# Link directories
link_directories(${PCL_LIBRARY_DIRS})

# Add Definitions
add_definitions(${PCL_DEFINITIONS})

# Add executable for displayImage and meanFilter programs
add_executable(mainRead mainRead.cpp) 
add_executable(mainGenPointCloud mainGenPointCloud.cpp) 

# Link display executable to OpenCV libraries
target_link_libraries (mainRead ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries (mainGenPointCloud ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

# Set build type to debug
set(CMAKE_BUILD_TYPE Debug)
