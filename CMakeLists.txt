cmake_minimum_required(VERSION 3.11.3)

project(SLAMBOOK)

cmake_policy(SET CMP0079 NEW)

set(PROJECT_NAME SLAMBOOK)
set(TEST_NAME SLAMBOOK_TEST)

# Set the C++ standard we will use
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)

find_program(CLANGTIDY clang-tidy)
if(CLANGTIDY)
message(STATUS "activate clang-tidy")
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
message(STATUS ${CLANGTIDY})
set(CMAKE_CXX_CLANG_TIDY "${CLANGTIDY};-extra-arg=-Wno-unknown-warning-option;--system-headers;-header-filter=src/*.cpp")
else()
message(SEND_ERROR "clang-tidy requested but executable not found")
endif()

add_subdirectory(src)

add_subdirectory(test)