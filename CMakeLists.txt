# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(myProject)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the path to libftdi and libusb if they are not in the default search path
list(APPEND CMAKE_PREFIX_PATH "/path/to/libftdi;/path/to/libusb")

# Find libftdi and libusb libraries
find_library(FTDI_LIB libftdi.so)
find_library(USB_LIB libusb-1.0.so)

file(GLOB SOURCES_FILES "src/*.cc")
add_executable(myProject pute.cpp ${SOURCES_FILES})

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src)
include_directories(${EIGEN3_INCLUDE_DIR} ${INCLUDE_DIR})

set_property(TARGET myProject PROPERTY CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fstack-protector-all")

# Link with libftdi and libusb libraries
target_link_libraries(myProject ${DEPENDENCIES} ${realsense2_LIBRARY} ${OpenCV_LIBS} ${FTDI_LIB} ${USB_LIB})

set_target_properties(myProject PROPERTIES FOLDER "REALSENSE-PROJECT")
