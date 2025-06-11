message("aaaaaaaaaaaaaaaaaaa")
# Collect all test source files
file(GLOB TEST_SOURCES "*.cpp")

foreach(test_src ${TEST_SOURCES})
  message(STATUS "Tests: " ${test_src})
  # Get the filename without extension to use as target name
  get_filename_component(test_name ${test_src} NAME_WE)

  add_executable(${test_name} ${test_src})

  target_link_libraries(${test_name}
    vis_lib
    gtest
    gtest_main
    glfw
    glad
  )

  # Register with ctest
  add_test(NAME ${test_name} COMMAND ${test_name})
  message("added test")
endforeach()