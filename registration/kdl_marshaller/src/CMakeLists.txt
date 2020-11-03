
add_library(${PROJECT_NAME}
	kdl_marshaller.h
	kdl_marshaller.cpp
	marshaller_calibration.h
	marshaller_calibration.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME}
	${orocos_kdl_LIBRARIES} 
	${Boost_LIBRARIES})
