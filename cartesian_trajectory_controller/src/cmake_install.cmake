# Install script for directory: /home/neuebot/neuebot_ws/src/cartesian_trajectory_controller/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/cartesian_trajectory_controller/libcartesian_trajectory_controller-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/cartesian_trajectory_controller/libcartesian_trajectory_controller-gnulinux.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/cartesian_trajectory_controller/libcartesian_trajectory_controller-gnulinux.so"
         RPATH "/opt/ros/kinetic/lib/orocos/gnulinux/ocl:/opt/ros/kinetic/lib/orocos/gnulinux/ocl/plugins:/opt/ros/kinetic/lib/orocos/gnulinux/ocl/types:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/orocos/gnulinux:/opt/ros/kinetic/lib/orocos/gnulinux/plugins:/opt/ros/kinetic/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/cartesian_trajectory_controller:/usr/local/lib/orocos/gnulinux/cartesian_trajectory_controller/types:/usr/local/lib/orocos/gnulinux/cartesian_trajectory_controller/plugins:/usr/local/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/cartesian_trajectory_controller" TYPE SHARED_LIBRARY FILES "/home/neuebot/neuebot_ws/src/cartesian_trajectory_controller/src/orocos/gnulinux/cartesian_trajectory_controller/libcartesian_trajectory_controller-gnulinux.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/cartesian_trajectory_controller/libcartesian_trajectory_controller-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/cartesian_trajectory_controller/libcartesian_trajectory_controller-gnulinux.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/cartesian_trajectory_controller/libcartesian_trajectory_controller-gnulinux.so"
         OLD_RPATH "/home/neuebot/neuebot_ws/src/cartesian_trajectory_controller/trajectories:/opt/ros/kinetic/lib/orocos/gnulinux/ocl:/opt/ros/kinetic/lib/orocos/gnulinux/ocl/plugins:/opt/ros/kinetic/lib/orocos/gnulinux/ocl/types:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/orocos/gnulinux/plugins::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/opt/ros/kinetic/lib/orocos/gnulinux/ocl:/opt/ros/kinetic/lib/orocos/gnulinux/ocl/plugins:/opt/ros/kinetic/lib/orocos/gnulinux/ocl/types:/opt/ros/kinetic/lib:/opt/ros/kinetic/lib/orocos/gnulinux:/opt/ros/kinetic/lib/orocos/gnulinux/plugins:/opt/ros/kinetic/lib/orocos/gnulinux/types:/usr/local/lib/orocos/gnulinux/cartesian_trajectory_controller:/usr/local/lib/orocos/gnulinux/cartesian_trajectory_controller/types:/usr/local/lib/orocos/gnulinux/cartesian_trajectory_controller/plugins:/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/cartesian_trajectory_controller/libcartesian_trajectory_controller-gnulinux.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/orocos/cartesian_trajectory_controller" TYPE FILE FILES "/home/neuebot/neuebot_ws/src/cartesian_trajectory_controller/src/cartesian_trajectory_controller-component.hpp")
endif()

