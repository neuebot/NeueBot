cmake_minimum_required(VERSION 2.8.11)
project(kdl_marshaller)
add_definitions(-std=c++11)

find_package(orocos_kdl REQUIRED)
find_package(Boost 1.54 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src ${orocos_kdl_INCLUDE_DIRS})

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

