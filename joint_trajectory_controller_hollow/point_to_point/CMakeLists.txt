cmake_minimum_required(VERSION 2.8.11)
project(PointToPoint)

#Create Plotter macros
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(PointToPoint_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "" FORCE)
set(PointToPoint_LINK_DIRECTORIES ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "" FORCE)

find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(ReflexxesTypeII REQUIRED HINTS ~/neuebot_ws/deps/ReflexxesTypeII)

orocos_library(PointToPoint SHARED point_to_point_trajectory_generator.hpp point_to_point_trajectory_generator.cpp)

target_include_directories(PointToPoint PUBLIC ${ReflexxesTypeII_INCLUDE_DIR})
#target_link_libraries(PointToPoint ${USE_OROCOS_LIBRARIES})
IF(CMAKE_BUILD_TYPE MATCHES Release)
	message("Linked to ReflexxesTypeII Release Lib")
	target_link_libraries(PointToPoint ${USE_OROCOS_LIBRARIES} ${ReflexxesTypeII_LIBRARY_RELEASE})
ELSE()
	message("Linked to ReflexxesTypeII Debug Lib")
	target_link_libraries(PointToPoint ${USE_OROCOS_LIBRARIES} ${ReflexxesTypeII_LIBRARY_DEBUG})
ENDIF()
