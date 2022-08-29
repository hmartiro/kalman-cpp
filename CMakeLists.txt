cmake_minimum_required(VERSION 2.8.4)
project(kalman)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(SOURCE_FILES kalman.cpp)
set(TEST_FILES kalman-test.cpp)

add_library(kalman-cpp ${SOURCE_FILES})
add_executable(kalman-test ${TEST_FILES})
target_link_libraries(kalman-test kalman-cpp)
