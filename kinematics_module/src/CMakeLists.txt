orocos_component(kinematics_module
    kinematics_module-component.hpp
    kinematics_module-component.cpp
    kinematics_module-errorcode.hpp)

target_link_libraries(kinematics_module Kinematics ${orocos_kdl_LIBRARIES} ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY} )

orocos_install_headers(	kinematics_module-component.hpp ) # ...you may add multiple header files
