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
add_executable(mainRead1CloudStack mainRead1CloudStack.cpp)
add_executable(mainRead2CloudPointer mainRead2CloudPointer.cpp)
add_executable(mainRead3CloudPointerColorMap mainRead3CloudPointerColorMap.cpp)
add_executable(mainRead4CloudPointerXYZRGBNormal mainRead4CloudPointerXYZRGBNormal.cpp)

# Link display executable to OpenCV libraries
target_link_libraries (mainRead1CloudStack ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries (mainRead2CloudPointer ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries (mainRead3CloudPointerColorMap ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
target_link_libraries (mainRead4CloudPointerXYZRGBNormal ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

# Set build type to debug
set(CMAKE_BUILD_TYPE Debug)
