orocos_component(surgery_plan 
		surgery_plan-component.hpp 
		surgery_plan-component.cpp
		surgery_plan-errorcode.hpp)

target_link_libraries(surgery_plan
		kdl_marshaller
    ${orocos_kdl_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OROCOS-RTT_MQUEUE_LIBRARIES}
    ${OROCOS-RTT_CORBA_LIBRARIES}
    ${OROCOS-RTT_RTT-MARSHALLING_LIBRARY}
)

orocos_install_headers(surgery_plan-component.hpp)

