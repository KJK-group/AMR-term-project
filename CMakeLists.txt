cmake_minimum_required(VERSION 3.14)
project(
  AMR-term-project
  VERSION 0.1.0
  DESCRIPTION ""
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # for clangd

# include_directories(include)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

add_executable(create_octomap_of_airlab_cage
               src/create_octomap_of_airlab_cage.cpp)
target_link_libraries(create_octomap_of_airlab_cage ${OCTOMAP_LIBRARIES})
target_compile_options(create_octomap_of_airlab_cage PUBLIC -O2 -std=c++17
                                                            -Wall -Werror)
