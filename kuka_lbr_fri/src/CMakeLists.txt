
orocos_component(kuka_lbr_fri 
kuka_lbr_fri-component.hpp 
kuka_lbr_fri-component.cpp 
kuka_lbr_fri-errorcode.hpp
circular_vector_buffer.hpp
persistent_vector_queue.hpp
fri_joint_overlay_client.hpp
fri_joint_overlay_client.cpp
fri_driver.hpp
fri_driver.cpp)

target_link_libraries( ${PROJECT_NAME} 
    KukaFRIClient
    ${orocos_kdl_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OROCOS-RTT_MQUEUE_LIBRARIES}
    ${OROCOS-RTT_CORBA_LIBRARIES}
    ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY}
)

orocos_install_headers(kuka_lbr_fri-component.hpp)
