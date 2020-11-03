cmake_minimum_required(VERSION 2.6.3)
project(control_modes_interface)
add_definitions(-std=c++11)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(OROCOS-RTT REQUIRED rtt-marshalling ${RTT_HINTS})
find_package(orocos_kdl REQUIRED)

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

include_directories(include/${PROJECT_NAME})

# Rest of the configuration is in src/
add_subdirectory( src )

#  orocos_typegen_headers(include/control_modes_interface/control_modes_interface-types.hpp) # ...you may add multiple header files

orocos_generate_package()
