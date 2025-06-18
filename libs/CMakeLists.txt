file(GLOB libs RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)

foreach(lib ${libs})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${lib})
        add_subdirectory(${lib})
    endif()
endforeach()