cmake_minimum_required(VERSION 2.6.3)
project(cartesian_trajectory_controller)
add_definitions(-std=c++11)
#Enable Corba
#add_definitions(-DENABLE_MQUEUE=ON)
#add_definitions(-DENABLE_CORBA=ON)
#add_definitions(-DCORBA_IMPLEMENTATION=OMNIORB)

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(OROCOS-RTT REQUIRED rtt-marshalling ${RTT_HINTS})
find_package(orocos_kdl REQUIRED)

include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

add_subdirectory( trajectories )

include_directories( include/${PROJECT_NAME} ${CartesianTrajectory_INCLUDE_DIRECTORIES} )
link_directories( ${CartesianTrajectory_LINK_DIRECTORIES} )

add_subdirectory( src )

orocos_typegen_headers(include/${PROJECT_NAME}/cartesian_trajectory_controller_types.hpp)

orocos_generate_package()
