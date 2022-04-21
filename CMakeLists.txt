cmake_minimum_required(VERSION 3.14)
project(
  AMR-term-project
  VERSION 0.1.0
  DESCRIPTION ""
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # for clangd

include(cmake) # include cmake scripts in ./cmake/
include_directories(include)

add_executable(${PROJECT_NAME} src/main.cpp)
target_sources(src/*.cpp)
target_compile_options(${PROJECT_NAME} PUBLIC -O2 -std=c++17 -Wall -Werror)
