orocos_component(${PROJECT_NAME} 
transformations-component.hpp 
transformations-component.cpp
transformations-errorcode.hpp
)

target_link_libraries(${PROJECT_NAME} 
	${orocos_kdl_LIBRARIES} 
	${OROCOS-RTT_RTT-MARSHALLING_LIBRARY}
)

orocos_install_headers(transformations-component.hpp)

