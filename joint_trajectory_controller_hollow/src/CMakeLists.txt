orocos_component(joint_trajectory_controller_hollow
    joint_trajectory_controller_hollow-component.hpp
    joint_trajectory_controller_hollow-component.cpp
    joint_trajectory_controller_hollow-errorcode.hpp)

target_link_libraries(joint_trajectory_controller_hollow PointToPoint MotionThroughPoints ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY})

orocos_install_headers(joint_trajectory_controller_hollow-component.hpp)

