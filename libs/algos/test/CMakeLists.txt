# Collect all test source files
file(GLOB TEST_SOURCES "*.cpp")

foreach(test_src ${TEST_SOURCES})
  # Get the filename without extension to use as target name
  get_filename_component(test_name ${test_src} NAME_WE)

  add_executable(${test_name} ${test_src})

  target_link_libraries(${test_name}
    gtest
    gtest_main
    state_lib
    cfg_lib
    algos_lib
    vis_lib
  )

  # Register with ctest
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()