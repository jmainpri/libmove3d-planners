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

add_executable(move3d_test_features ${CMAKE_SOURCE_DIR}/src/tests/test_features.cpp)
set_target_properties(move3d_test_features PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(move3d_test_features move3d-planners ${Boost_LIBRARIES} ${LIBS})
install(TARGETS move3d_test_features
        RUNTIME DESTINATION bin CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib CONFIGURATIONS ${CMAKE_BUILD_TYPE})

add_executable(move3d_test_goal_set ${CMAKE_SOURCE_DIR}/src/tests/test_goal_set_project.cpp)
set_target_properties(move3d_test_goal_set PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(move3d_test_goal_set move3d-planners ${Boost_LIBRARIES} ${LIBS})
install(TARGETS move3d_test_goal_set
        RUNTIME DESTINATION bin CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib CONFIGURATIONS ${CMAKE_BUILD_TYPE})

add_executable(move3d_test_vector_trajectory ${CMAKE_SOURCE_DIR}/src/tests/test_vector_trajectory.cpp)
set_target_properties(move3d_test_vector_trajectory PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(move3d_test_vector_trajectory move3d-planners ${Boost_LIBRARIES} ${LIBS})
install(TARGETS move3d_test_vector_trajectory
        RUNTIME DESTINATION bin CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib CONFIGURATIONS ${CMAKE_BUILD_TYPE})

add_executable(move3d_test_dtw ${CMAKE_SOURCE_DIR}/src/tests/test_dtw.cpp)
set_target_properties(move3d_test_dtw PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(move3d_test_dtw move3d-planners ${Boost_LIBRARIES} ${LIBS})
install(TARGETS move3d_test_dtw
      RUNTIME DESTINATION bin CONFIGURATIONS ${CMAKE_BUILD_TYPE}
      LIBRARY DESTINATION lib CONFIGURATIONS ${CMAKE_BUILD_TYPE})

add_executable(move3d_test_rotations ${CMAKE_SOURCE_DIR}/src/tests/test_rotations.cpp)
set_target_properties(move3d_test_rotations PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(move3d_test_rotations move3d-planners ${Boost_LIBRARIES} ${LIBS})
install(TARGETS move3d_test_rotations
        RUNTIME DESTINATION bin CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib CONFIGURATIONS ${CMAKE_BUILD_TYPE})

add_executable(move3d_test_fft ${CMAKE_SOURCE_DIR}/src/tests/test_fft.cpp)
set_target_properties(move3d_test_fft PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(move3d_test_fft move3d-planners ${Boost_LIBRARIES} ${LIBS})
install(TARGETS move3d_test_fft
        RUNTIME DESTINATION bin CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib CONFIGURATIONS ${CMAKE_BUILD_TYPE})

enable_testing()
#message("CMAKE_BINARY_DIR = " ${CMAKE_BINARY_DIR})
add_test(NAME basic COMMAND move3dplannerstest)
