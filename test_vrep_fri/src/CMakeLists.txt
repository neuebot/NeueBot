orocos_component(${PROJECT_NAME}
	test_vrep_fri-component.hpp 
	test_vrep_fri-component.cpp
	test_vrep_fri-errorcode.hpp
)

target_link_libraries(${PROJECT_NAME}
    ${orocos_kdl_LIBRARIES}
    ${OROCOS-RTT_MQUEUE_LIBRARIES}
    ${OROCOS-RTT_CORBA_LIBRARIES}
    ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY}
)

orocos_install_headers(test_vrep_fri-component.hpp)

