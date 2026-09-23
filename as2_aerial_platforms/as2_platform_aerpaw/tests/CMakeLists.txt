# Tests
file(GLOB GTEST_SOURCE "*_gtest.cpp")

if(GTEST_SOURCE)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)

  foreach(TEST_SOURCE ${GTEST_SOURCE})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

    ament_add_gtest(${PROJECT_NAME}_${TEST_NAME} ${TEST_SOURCE})
    ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES})
    target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ament_index_cpp::ament_index_cpp gtest_main ${PROJECT_NAME} nlohmann_json::nlohmann_json)
  endforeach()
endif()

# Stage 18: fleet-config loader test (pure python; unittest, no ROS/pytest runtime).
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
  add_test(NAME fleet_config_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/fleet_config_test.py)
  set_tests_properties(fleet_config_test PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/../launch")
endif()