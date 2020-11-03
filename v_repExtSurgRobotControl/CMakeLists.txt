cmake_minimum_required(VERSION 2.6.3)
project (v_repExtSurgRobotControl)
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

find_package(OROCOS-RTT REQUIRED rtt-marshalling rtt-transport-corba ${RTT_HINTS})
find_package(orocos_kdl REQUIRED)

include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

#INCLUDE DIRECTORIES AND SOURCE DIRECTORIES
include_directories(vrep/include)
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR} ${OROCOS-RTT_INCLUDE_DIRS})

add_subdirectory(include)

add_subdirectory(vrep/common)
add_subdirectory(src)

#project source, include and resource files
set( HEADER_FILES
        ${vrep_plugin_hdr}
        ${surg_robot_control_hdr}
        v_repExtSurgRobotControl.h
)

set( SOURCE_FILES
        ${vrep_plugin_src}
        ${surg_robot_control_src}
        v_repExtSurgRobotControl.cpp
)

set(SurgRobotControlPlugin_SRCS
    ${HEADER_FILES}
    ${SOURCE_FILES}
)

add_library(${PROJECT_NAME} SHARED ${SurgRobotControlPlugin_SRCS})

#link to which libs
target_link_libraries(${PROJECT_NAME}
${OROCOS-RTT_LIBRARIES} 
${OROCOS-RTT_CORBA_LIBRARIES} 
${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} 
${Boost_LIBRARIES})

