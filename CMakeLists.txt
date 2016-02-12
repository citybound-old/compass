cmake_minimum_required(VERSION 3.3)
project(compass)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
        deps/range-v3/include
        deps/eigen
        deps/googletest/googletest/include
)

add_subdirectory(deps/googletest)

set(SOURCE_FILES main.cpp)
add_executable(compass ${SOURCE_FILES})

add_executable(compass_tests test.cpp)
target_link_libraries(compass_tests gtest)