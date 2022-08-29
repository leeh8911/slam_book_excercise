file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories("/usr/include/eigen3/")

find_package(Pangolin 0.8 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Python3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
    ${SRC_FILES})

add_executable(${PROJECT_NAME}_APP
        ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} 
Eigen3::Eigen
pango_display 
pango_plot)

target_link_libraries(${PROJECT_NAME}_APP
Eigen3::Eigen
pango_display 
pango_plot)



target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/includes
)

if ( CMAKE_COMPILER_IS_GNUCC )
    target_compile_options(${PROJECT_NAME} PRIVATE -Werror -Wall -Wextra -Wuninitialized -pedantic)
endif()
if ( MSVC )
    target_compile_options(${PROJECT_NAME} PRIVATE /W4)
endif()