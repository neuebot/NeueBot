orocos_component(registration
    registration-component.hpp
    registration-component.cpp
    registration-errorcode.hpp)

target_link_libraries(registration
    kdl_marshaller
    ${orocos_kdl_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OROCOS-RTT_MQUEUE_LIBRARIES}
    ${OROCOS-RTT_CORBA_LIBRARIES}
    ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY}
)

orocos_install_headers(registration-component.hpp)
