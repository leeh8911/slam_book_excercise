file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Pangolin 0.8 REQUIRED)

find_package(Python3 REQUIRED)


add_library(
    ${PLOT_TRAJECTORY} SHARED
    ${SRC_FILES}
)

add_executable(
    ${PLOT_TRAJECTORY}_APP
    ${SRC_FILES}
)

target_link_libraries(${PLOT_TRAJECTORY}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    "/tmp/Pangolin/build/libpango_windowing.so"
    ${UTILS}
)

target_link_libraries(${PLOT_TRAJECTORY}_APP
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    "/tmp/Pangolin/build/libpango_windowing.so"
    ${UTILS}
)

include_directories(
    "/usr/include/eigen3/"
    ${Pangolin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/includes
    ${CMAKE_SOURCE_DIR}/src/utils
)

if ( CMAKE_COMPILER_IS_GNUCC )
    target_compile_options(${PLOT_TRAJECTORY} PRIVATE -Werror -Wall -Wextra -Wuninitialized -pedantic)
endif()
if ( MSVC )
    target_compile_options(${PLOT_TRAJECTORY} PRIVATE /W4)
endif()