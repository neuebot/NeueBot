orocos_component(${PROJECT_NAME}
    joint_trajectory_controller-component.hpp
    joint_trajectory_controller-component.cpp
    joint_trajectory_controller-errorcode.hpp)

target_link_libraries(${PROJECT_NAME}
    PointToPoint
    MotionThroughPoints
    ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY})

orocos_install_headers(joint_trajectory_controller-component.hpp)

