cmake_minimum_required(VERSION 3.14)
project(
  amr_term_project
  VERSION 0.1.0
  DESCRIPTION ""
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # for clangd

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

add_executable(create_octomap_of_airlab_cage
               src/create_octomap_of_airlab_cage.cpp)
target_link_libraries(create_octomap_of_airlab_cage ${OCTOMAP_LIBRARIES})
target_compile_options(create_octomap_of_airlab_cage PUBLIC -O2 -std=c++17
                                                            -Wall -Werror)

find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             nav_msgs
             mavros_msgs
             octomap_msgs
             roscpp
             tf2
             tf2_ros
             tf2_eigen)

include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

include_directories(include)

catkin_package()

function(add_ros_executable name main)
  add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${main})
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} ${catkin_LIBRARIES} fmt::fmt-header-only)
  target_compile_options(
    ${name}
    PRIVATE -O0 -g
            # -g -Wall -Werror -Wextra -Wpedantic -Weffc++ -Wsign-conversion
  )
endfunction(add_ros_executable)

# add_executable(${PROJECT_NAME} src/main.cpp) target_sources(src/*.cpp)
# target_compile_options(${PROJECT_NAME} PUBLIC -O2 -std=c++17 -Wall -Werror)
