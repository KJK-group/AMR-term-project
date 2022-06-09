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

find_package(
  catkin REQUIRED
  COMPONENTS std_msgs
             geometry_msgs
             nav_msgs
             mavros_msgs
             message_generation
             visualization_msgs
             roscpp
             tf2
             tf2_ros
             tf2_eigen)

include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include)
add_subdirectory(libs/kdtree)

add_message_files(
  DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/msg
  FILES
  PointNorm.msg
  PointNormStamped.msg
  MissionStateStamped.msg
  ControllerStateStamped.msg
  Error.msg
)
add_service_files(
  FILES
  RrtFindPath.srv
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package()

function(add_ros_executable name main)
  add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${main})
  add_dependencies(${name} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${name} ${catkin_LIBRARIES})
  target_compile_options(
    ${name}
    PRIVATE #-O0
            # -g -Wall -Werror -Wextra -Wpedantic -Weffc++ -Wsign-conversion
  )
endfunction(add_ros_executable)

function(add_ros_dependencies target)
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                   ${catkin_EXPORTED_TARGETS})
endfunction(add_ros_dependencies)


add_ros_executable(map_to_odom_tf_broadcaster src/nodes/map_to_odom_tf_broadcaster.cpp)

# Libraries
add_library(bezier_spline SHARED src/bezier_spline.cpp)
target_sources(bezier_spline PUBLIC src/math.cpp)

add_library(linear_trajectory SHARED src/linear_trajectory.cpp)

add_library(rrt SHARED src/rrt.cpp)
target_link_libraries(rrt ${catkin_LIBRARIES} kdtree-header-only)

add_library(compound_trajectory SHARED src/compound_trajectory.cpp)
target_link_libraries(compound_trajectory bezier_spline linear_trajectory rrt)

add_library(mission SHARED src/mission.cpp)
target_link_libraries(mission compound_trajectory rrt)

# control library
add_library(control SHARED src/pid_controller.cpp)
target_sources(control PUBLIC src/transformlistener.cpp)

# add_ros_executable(mission_manager src/nodes/mission_manager.cpp)
add_executable(control_manager src/nodes/control_manager.cpp)
add_ros_dependencies(control_manager)
target_link_libraries(control_manager ${catkin_LIBRARIES} control)

# executables using libraries
add_executable(rrt_service src/nodes/rrt_service.cpp)
add_ros_dependencies(rrt_service)
# target_compile_options(rrt_service PRIVATE -O0 -g)
target_compile_options(rrt_service PRIVATE -O2 -g)
target_link_libraries(rrt_service ${catkin_LIBRARIES} rrt kdtree-header-only ${OCTOMAP_LIBRARIES})

# add_ros_executable(mission_manager src/mission_manager.cpp)
add_executable(mission_manager src/nodes/mission_manager.cpp)
add_dependencies(mission_manager ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_manager ${catkin_LIBRARIES} mission)
target_compile_options(mission_manager PRIVATE -g)

add_executable(create_octomap_of_airlab_cage
               src/create_octomap_of_airlab_cage.cpp)
target_link_libraries(create_octomap_of_airlab_cage ${OCTOMAP_LIBRARIES})
target_compile_options(create_octomap_of_airlab_cage PUBLIC -O2 -std=c++17
                                                            -Wall -Werror)

add_ros_executable(set_point_node src/nodes/set_point_node.cpp)