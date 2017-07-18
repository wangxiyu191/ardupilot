cmake_minimum_required(VERSION 2.8.4)
project(Ardupilot)
set(CMAKE_CXX_STANDARD 11)
# we just grab all the cpp and h files. not ideal, but works well enough
# means a "reload CMake project" when you add new files
file(GLOB_RECURSE SOURCE_FILES APMrover2/*.cpp APMrover2/*.h)
file(GLOB_RECURSE SOURCE_FILES ArduCopter/*.cpp ArduCopter/*.h)
file(GLOB_RECURSE SOURCE_FILES ArduPlane/*.cpp ArduPlane/*.h)
file(GLOB_RECURSE SOURCE_FILES libraries/*.cpp libraries/*.h)
include_directories(APMrover2)
include_directories(ArduCopter)
include_directories(ArduPlane)
include_directories(libraries)
# To work with SITL 
add_definitions(-DSKETCHBOOK="./")
#add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_SITL)
add_definitions(-DCONFIG_HAL_BOARD=HAL_BOARD_PX4)
add_executable(fake-target-for-clion ${SOURCE_FILES} libraries/AP_OpticalFlow/AP_OpticalFlow_NanoPi.cpp libraries/AP_OpticalFlow/AP_OpticalFlow_NanoPi.h)