function(add_ros_executable name main)
  add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${main})
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} ${catkin_LIBRARIES})
  target_compile_options(
    ${name}
    PRIVATE -O2
            -Wall
            # -Werror
            -Wextra
            -Wpedantic
            # -Weffc++
            -Wsign-conversion)
endfunction(add_ros_executable)
