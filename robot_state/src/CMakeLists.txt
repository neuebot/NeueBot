orocos_component(robot_state robot_state-component.hpp robot_state-component.cpp)

target_link_libraries(robot_state 
ForwardKinematics 
${orocos_kdl_LIBRARIES} 
${OROCOS-RTT_MQUEUE_LIBRARIES} 
${OROCOS-RTT_RTT-MARSHALLING_LIBRARY}
${OROCOS-RTT_CORBA_LIBRARIES})

orocos_install_headers( robot_state-component.hpp )


