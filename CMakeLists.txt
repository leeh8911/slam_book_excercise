cmake_minimum_required(VERSION 3.11)

project(SLAM_BOOK_EXCERCISE)

set(PROJECT_NAME SLAM_BOOK_EXCERCISE)
set(TEST_NAME EXCERCISE_TEST)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src)
add_subdirectory(test)

add_dependencies(${TEST_NAME} ${PROJECT_NAME})