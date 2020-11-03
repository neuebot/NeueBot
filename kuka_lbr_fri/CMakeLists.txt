cmake_minimum_required(VERSION 2.6.3)
project(kuka_lbr_fri)
add_definitions(-std=c++11 -fPIC)
set(CMAKE_BUILD_TYPE Debug)
#Enable Corba
add_definitions(-DENABLE_MQUEUE=ON)
add_definitions(-DENABLE_CORBA=ON)
add_definitions(-DCORBA_IMPLEMENTATION=OMNIORB)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(OROCOS-RTT REQUIRED rtt-marshalling rtt-transport-mqueue rtt-transport-corba ${RTT_HINTS})
find_package(orocos_kdl REQUIRED)
find_package(Boost 1.54 REQUIRED COMPONENTS thread serialization)

include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

orocos_use_package(ocl-logging)

add_subdirectory( fri )
include_directories( include/${PROJECT_NAME} )
include_directories( ${KukaFRIClient_INCLUDE_DIRS} )

add_subdirectory( src )

#orocos_typegen_headers( include/${PROJECT_NAME}/kuka_lbr_fri-types.hpp )

orocos_generate_package()
