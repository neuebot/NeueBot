cmake_minimum_required(VERSION 2.6.3)
project(robot_state)
add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)
#Enable Corba
add_definitions(-DENABLE_MQUEUE=ON)
#add_definitions(-DENABLE_CORBA=ON)
#add_definitions(-DCORBA_IMPLEMENTATION=OMNIORB)

find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(OROCOS-RTT REQUIRED rtt-marshalling rtt-transport-mqueue ${RTT_HINTS})# rtt-transport-corba ${RTT_HINTS})
find_package(orocos_kdl REQUIRED)

include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

add_subdirectory( forward_kinematics )

include_directories( include/${PROJECT_NAME} ${ForwardKinematics_INCLUDE_DIRECTORIES} )
link_directories( ${ForwardKinematics_LINK_DIRECTORIES} )

add_subdirectory( src )

#orocos_typegen_headers(include/${PROJECT_NAME}/robot_state_types.hpp)

orocos_generate_package()
