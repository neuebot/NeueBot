orocos_component(cartesian_trajectory_controller 
    cartesian_trajectory_controller-component.hpp
    cartesian_trajectory_controller-component.cpp
    cartesian_trajectory_controller-errorcode.hpp) # ...you may add multiple source files

target_link_libraries(cartesian_trajectory_controller CartesianTrajectory ${orocos_kdl_LIBRARIES} ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY})

orocos_install_headers(cartesian_trajectory_controller-component.hpp) # ...you may add multiple header files

