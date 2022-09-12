set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories("/usr/include/eigen3/")

file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(${TEST_NAME} ${SRC_FILES})

target_link_libraries(
    ${TEST_NAME} PUBLIC
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES} 
    pthread
    fmt
)

target_include_directories(${TEST_NAME} PUBLIC
"${CMAKE_SOURCE_DIR}/includes"
"${CMAKE_SOURCE_DIR}"
Eigen3::Eigen
fmt::fmt)

include(GoogleTest)
gtest_discover_tests(${TEST_NAME})