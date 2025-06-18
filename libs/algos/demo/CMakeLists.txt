file(GLOB DEMO_SOURCES "Demo*.cpp")

foreach(demo_src ${DEMO_SOURCES})
  # Get the filename without extension and directory
  get_filename_component(demo_name ${demo_src} NAME_WE)

  add_executable(${demo_name} ${demo_src})

  target_link_libraries(${demo_name} PRIVATE algos_lib state_lib vis_lib)
endforeach()