orocos_component(procedure_coordinator 
procedure_coordinator-component.hpp 
procedure_coordinator-component.cpp
procedure_coordinator-errorcode.hpp)

target_link_libraries(procedure_coordinator
    ${orocos_kdl_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OROCOS-RTT_MQUEUE_LIBRARIES}
    ${OROCOS-RTT_CORBA_LIBRARIES}
    ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY}
)

orocos_install_headers(procedure_coordinator-component.hpp)

