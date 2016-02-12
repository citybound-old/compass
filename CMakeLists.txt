cmake_minimum_required(VERSION 3.3)
project(compass)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp)
add_executable(compass ${SOURCE_FILES})