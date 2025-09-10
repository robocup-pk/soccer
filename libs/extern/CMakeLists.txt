file(GLOB libs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)

foreach(lib ${libs})
  if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${lib})
    # Skip libserial on macOS as it uses Linux-specific headers
    if(APPLE AND ${lib} STREQUAL "libserial")
      message(STATUS "Skipping libserial on macOS (Linux-specific library)")
      continue()
    endif()
    add_subdirectory(${lib})
  endif()
endforeach()