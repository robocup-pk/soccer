# Collect all test source files
file(GLOB TEST_SOURCES "*.cpp")
# Exclude heavy integration test that requires full Python/numpy stack
list(REMOVE_ITEM TEST_SOURCES ${CMAKE_CURRENT_LIST_DIR}/test_bangbang_trajectory.cpp)

foreach(test_src ${TEST_SOURCES})
  # Get the filename without extension to use as target name
  get_filename_component(test_name ${test_src} NAME_WE)

  add_executable(${test_name} ${test_src})

  target_link_libraries(${test_name}
    ctrl_lib
    gtest
    gtest_main
  )

  # Register with ctest
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()