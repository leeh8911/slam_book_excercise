set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
enable_testing()

file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(${TEST_NAME} ${SRC_FILES})

target_link_libraries(
    ${TEST_NAME} PUBLIC
    ${PROJECT_NAME}
    gtest_main
)

target_include_directories(${TEST_NAME} PUBLIC
"${CMAKE_SOURCE_DIR}/includes"
"${CMAKE_SOURCE_DIR}")

include(GoogleTest)
gtest_discover_tests(${TEST_NAME})