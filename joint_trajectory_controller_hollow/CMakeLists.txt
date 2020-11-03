cmake_minimum_required(VERSION 2.6.3)
project(joint_trajectory_controller_hollow)
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

find_package(OROCOS-RTT REQUIRED rtt-marshalling ${RTT_HINTS})#rtt-transport-mqueue rtt-transport-corba ${RTT_HINTS})
find_package(orocos_kdl REQUIRED)

include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

add_subdirectory( point_to_point )
add_subdirectory( motion_through_points )

include_directories( include/${PROJECT_NAME} ${PointToPoint_INCLUDE_DIRECTORIES} ${MotionThroughPoints_INCLUDE_DIRECTORIES} )
link_directories( ${PointToPoint_LINK_DIRECTORIES} ${MotionThroughPoints_LINK_DIRECTORIES} )

add_subdirectory( src )

#orocos_typegen_headers(include/${PROJECT_NAME}/joint_trajectory_controller_hollow_types.hpp)

orocos_generate_package()
