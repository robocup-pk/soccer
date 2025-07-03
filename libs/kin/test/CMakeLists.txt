project(kin_tests)

# - to run tests navigate to build/ and type ctest on terminal
# - to run a specific test type ctest -R MySpecificTest, which is ${TEST_NAME} below
# - to see details on what happened: ctest -R TestWaypoint --output-on-failure

# - to build with tests:
# cmake .. -DENABLE_TESTS=ON 

# add your unit tests here
file(GLOB_RECURSE TEST_SOURCES "*.cpp")

foreach (TEST_SOURCE ${TEST_SOURCES})
  # Extract the test name from the source file name
  # (NAME_WE = File name without directory or longest extension)
  get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

  # Create the test executable
  add_executable(${TEST_NAME} ${TEST_SOURCE})

  # Link the test executable with Google Test and your library
  target_link_libraries(${TEST_NAME} PRIVATE gtest gtest_main kin_lib state_lib)

  # Add the test to CMake's test suite
  add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()