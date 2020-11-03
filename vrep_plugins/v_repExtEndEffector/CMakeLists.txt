cmake_minimum_required(VERSION 2.6.3)
project (v_repExtEndEffector)
# COMPILATION FLAGS
add_definitions(-std=c++11)
add_definitions(-DRTT_COMPONENT)
# ENABLE CORBA
add_definitions(-DENABLE_CORBA=ON)
add_definitions(-DCORBA_IMPLEMENTATION=OMNIORB)

# OROCOS PACKAGES
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(OROCOS-RTT REQUIRED rtt-transport-corba ${RTT_HINTS})
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

#INCLUDE DIRECTORIES AND SOURCE DIRECTORIES
include_directories(vrep/include)
include_directories(include)
add_subdirectory(include)

add_subdirectory(vrep/common)
add_subdirectory(src)

#project source, include and resource files
set( HEADER_FILES
        ${vrep_plugin_hdr}
        ${end_effector_hdr}
        v_repExtEndEffector.h
)

set( SOURCE_FILES
        ${vrep_plugin_src}
        ${end_effector_src}
        v_repExtEndEffector.cpp
)

set(EndEffectorPlugin_SRCS
    ${HEADER_FILES}
    ${SOURCE_FILES}
)

add_library(v_repExtEndEffector SHARED ${EndEffectorPlugin_SRCS})

#link to which libs
target_link_libraries(v_repExtEndEffector ${OROCOS-RTT_LIBRARIES} ${OROCOS-RTT_CORBA_LIBRARIES})

