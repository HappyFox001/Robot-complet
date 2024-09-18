cmake_minimum_required(VERSION 3.10)
# C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Project name
project(robot)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${YDLIDAR_SDK_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    /usr/local/include
    ./tof
    /usr/local/include/src
    
)

message("YDLIDAR_SDK_INCLUDE_DIRS: ${YDLIDAR_SDK_INCLUDE_DIRS}")
message("YDLIDAR_SDK_LIBRARIES: ${YDLIDAR_SDK_LIBRARIES}")

# Find ydlidar_sdk and OpenCV packages
find_package(ydlidar_sdk REQUIRED)
find_package(OpenCV REQUIRED)

# 查找AprilTag库  
find_package(PkgConfig)  
pkg_check_modules(APRILTAG REQUIRED apriltag)  
include_directories(${APRILTAG_INCLUDE_DIRS}) 

# Add source files
set(SOURCE_FILES
    main.cpp
    main.hpp
    radar.cpp
    radar.hpp
    apriltagDetect.cpp
    apriltagDetect.hpp
    tof/nlink_linktrack_nodeframe0.c
    tof/nlink_linktrack_nodeframe1.c
    tof/nlink_tofsense_frame0.c
    tof/nlink_tofsensem_frame0.c
    tof/nlink_utils.c
)

# Add executable
add_executable(robot ${SOURCE_FILES})

# Link libraries
target_link_libraries(
    robot
    ${OpenCV_LIBS}
    ${YDLIDAR_SDK_LIBRARIES}
    ${APRILTAG_LIBRARIES}
    wiringPi
    pthread
)
