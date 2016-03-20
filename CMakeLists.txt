cmake_minimum_required(VERSION 3.3)
project(compass)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
        deps/range-v3/include
        deps/eigen
        deps/googletest/googletest/include
        deps
)

add_subdirectory(deps/googletest)
add_subdirectory(deps/whiteboard)

set(SOURCE_FILES main.cpp)
add_executable(compass ${SOURCE_FILES})

add_executable(compass_tests test.cpp)
target_link_libraries(compass_tests gtest)

set_target_properties(compass_tests PROPERTIES
        COTIRE_PREFIX_HEADER_INCLUDE_PATH "${CMAKE_SOURCE_DIR}/deps")

add_custom_target(graphical_tests
        ${CMAKE_BINARY_DIR}/compass_tests --gtest_color=no | ${CMAKE_BINARY_DIR}/deps/whiteboard/whiteboard
        DEPENDS compass_tests whiteboard
)