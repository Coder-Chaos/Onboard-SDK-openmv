# *  @Copyright (c) 2016-2017 DJI
# *
# * Permission is hereby granted, free of charge, to any person obtaining a copy
# * of this software and associated documentation files (the "Software"), to deal
# * in the Software without restriction, including without limitation the rights
# * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# * copies of the Software, and to permit persons to whom the Software is
# * furnished to do so, subject to the following conditions:
# *
# * The above copyright notice and this permission notice shall be included in
# * all copies or substantial portions of the Software.
# *
# * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# * SOFTWARE.
# *
# *

cmake_minimum_required(VERSION 2.8)
project(djiosdk-flightcontrol-demo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a -marm -mfpu=neon -std=c++11 -pthread -g -O3")

# Try to see if user has OpenCV installed
# if yes, default callback will display the image
find_package( OpenCV QUIET )
if (OpenCV_FOUND)
  message( "\n${PROJECT_NAME}...")
  message( STATUS "Found OpenCV installed in the system, will use it to display image in AdvancedSensing APIs")
  message( STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
  message( STATUS " - Libraries: ${OpenCV_LIBRARIES}")
  add_definitions(-DOPEN_CV_INSTALLED)
else()
  message( STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
endif ()

#set(OpenCV_DIR /usr/include/opencv)
#find_package( OpenCV QUIET COMPONENTS core highgui improc imgcodecs videoio ) # locate OpenCV in system

include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers


include_directories(${ONBOARDSDK_SOURCE}/api/inc)
include_directories(${ONBOARDSDK_SOURCE}/utility/inc)
include_directories(${ONBOARDSDK_SOURCE}/hal/inc)
include_directories(${ONBOARDSDK_SOURCE}/protocol/inc)
include_directories(${ONBOARDSDK_SOURCE}/platform/linux/inc)
include_directories(${MODULES_HEADER_DIR})
include_directories(${FLIGHT_MODULES_HEADER_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../core/inc)
FILE(GLOB SOURCE_FILES *.hpp *.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_environment.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../common/dji_linux_helpers.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../../core/src/flight_sample.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} djiosdk-core)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

