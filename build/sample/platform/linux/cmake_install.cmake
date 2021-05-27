# Install script for directory: /home/pi/Downloads/Onboard-SDK-openmv/sample/platform/linux

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/pi/Downloads/Onboard-SDK-openmv/build/sample/platform/linux/camera-gimbal/cmake_install.cmake")
  include("/home/pi/Downloads/Onboard-SDK-openmv/build/sample/platform/linux/flight-control/cmake_install.cmake")
  include("/home/pi/Downloads/Onboard-SDK-openmv/build/sample/platform/linux/flight-control-demo/cmake_install.cmake")
  include("/home/pi/Downloads/Onboard-SDK-openmv/build/sample/platform/linux/mfio/cmake_install.cmake")
  include("/home/pi/Downloads/Onboard-SDK-openmv/build/sample/platform/linux/missions/cmake_install.cmake")
  include("/home/pi/Downloads/Onboard-SDK-openmv/build/sample/platform/linux/mobile/cmake_install.cmake")
  include("/home/pi/Downloads/Onboard-SDK-openmv/build/sample/platform/linux/telemetry/cmake_install.cmake")
  include("/home/pi/Downloads/Onboard-SDK-openmv/build/sample/platform/linux/logging/cmake_install.cmake")
  include("/home/pi/Downloads/Onboard-SDK-openmv/build/sample/platform/linux/time-sync/cmake_install.cmake")
  include("/home/pi/Downloads/Onboard-SDK-openmv/build/sample/platform/linux/payload-3rd-party/cmake_install.cmake")
  include("/home/pi/Downloads/Onboard-SDK-openmv/build/sample/platform/linux/payloads/cmake_install.cmake")

endif()

