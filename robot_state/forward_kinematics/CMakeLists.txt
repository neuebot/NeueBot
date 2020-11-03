cmake_minimum_required(VERSION 2.8.11)
project(ForwardKinematics)

#Create Plotter macros
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(ForwardKinematics_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "" FORCE)
set(ForwardKinematics_LINK_DIRECTORIES ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "" FORCE)

find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(orocos_kdl REQUIRED)

orocos_library(ForwardKinematics SHARED forward_kinematics.hpp forward_kinematics.cpp global_configuration.hpp)

target_link_libraries(ForwardKinematics ${orocos_kdl_LIBRARIES})
