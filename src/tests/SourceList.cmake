message(STATUS "Adding Tests")

#add_executable(move3dplannerstest ${CMAKE_SOURCE_DIR}/src/tests/main.cpp)
#set_target_properties(move3dplannerstest PROPERTIES LINKER_LANGUAGE CXX)
#target_link_libraries(move3dplannerstest move3d-planners)

add_executable(move3d_test_control ${CMAKE_SOURCE_DIR}/src/tests/test_control_costs.cpp)
set_target_properties(move3d_test_control PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(move3d_test_control move3d-planners ${Boost_LIBRARIES} ${LIBS})
install(TARGETS move3d_test_control
        RUNTIME DESTINATION bin CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib CONFIGURATIONS ${CMAKE_BUILD_TYPE})

add_executable(move3d_test_random_generator ${CMAKE_SOURCE_DIR}/src/tests/test_random_generator.cpp)
set_target_properties(move3d_test_random_generator PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(move3d_test_random_generator move3d-planners ${Boost_LIBRARIES} ${LIBS})
install(TARGETS move3d_test_random_generator
        RUNTIME DESTINATION bin CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib CONFIGURATIONS ${CMAKE_BUILD_TYPE})


add_executable(move3d_test_save_to_file ${CMAKE_SOURCE_DIR}/src/tests/test_save_to_file.cpp)
set_target_properties(move3d_test_save_to_file PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(move3d_test_save_to_file move3d-planners ${Boost_LIBRARIES} ${LIBS})
install(TARGETS move3d_test_save_to_file
        RUNTIME DESTINATION bin CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib CONFIGURATIONS ${CMAKE_BUILD_TYPE})

enable_testing()
#message("CMAKE_BINARY_DIR = " ${CMAKE_BINARY_DIR})
add_test(NAME basic COMMAND move3dplannerstest)
