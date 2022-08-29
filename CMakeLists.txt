cmake_minimum_required(VERSION 2.8.4)
project(kalman-cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(SOURCE_FILES kalman.cpp)
set(TEST_FILES kalman-test.cpp)

add_library({{PROJECT_NAME}} ${SOURCE_FILES})
add_executable({{PROJECT_NAME}}-test ${TEST_FILES})
target_link_libraries({{PROJECT_NAME}}-test {{PROJECT_NAME}} )

install(TARGETS {{PROJECT_NAME}}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
  DESTINATION include
  PATTERN ".svn" EXCLUDE)

