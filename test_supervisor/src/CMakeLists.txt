orocos_component(${PROJECT_NAME} test_supervisor-component.hpp test_supervisor-component.cpp test_supervisor-errors.hpp)

target_link_libraries(${PROJECT_NAME}
    ${orocos_kdl_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OROCOS-RTT_MQUEUE_LIBRARIES}
    ${OROCOS-RTT_CORBA_LIBRARIES}
    ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY}
)

orocos_install_headers(test_supervisor-component.hpp)
