
orocos_component(${PROJECT_NAME}
    control_modes_interface-component.hpp
    control_modes_interface-component.cpp
    control_modes_interface-errorcode.hpp)

target_link_libraries(${PROJECT_NAME}
    ${orocos_kdl_LIBRARIES}
    ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY})

orocos_install_headers(control_modes_interface-component.hpp)


