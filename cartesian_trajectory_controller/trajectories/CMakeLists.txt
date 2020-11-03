cmake_minimum_required(VERSION 2.8.11)
project(CartesianTrajectory)

#Create Plotter macros
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CartesianTrajectory_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "" FORCE)
set(CartesianTrajectory_LINK_DIRECTORIES ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "" FORCE)

find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

orocos_library(CartesianTrajectory SHARED 
	cartesian_trajectory_generator.hpp
	cartesian_trajectory_generator.cpp)

target_link_libraries(CartesianTrajectory ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
