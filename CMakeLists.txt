cmake_minimum_required(VERSION 3.5)
project(panorama)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CROSSCOMPILING)
    message(STATUS "Cross-compiling Windows")
    add_definitions(-DWIN32 -D_WINDOWS)
    find_package(Protobuf REQUIRED)
else()
    message(STATUS "Native")
    if(UNIX)
        add_definitions(-DLINUX)
    endif()
endif()

find_package(OpenMP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK REQUIRED gtk+-3.0)
include_directories(${GTK_INCLUDE_DIRS})
link_directories(${GTK_LIBRARY_DIRS})
add_definitions(${GTK_CFLAGS_OTHER})


add_custom_target(copy_res ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/res ${CMAKE_BINARY_DIR}/res
    COMMENT "Copying resources to build directory"
)

#Ofast -march=native
add_definitions(-O2)

include_directories(        ${CMAKE_SOURCE_DIR}/src/math
                            ${CMAKE_SOURCE_DIR}/src/ui
                            ${CMAKE_SOURCE_DIR}/src/system
                            ${CMAKE_SOURCE_DIR}/src/test
                            ${CMAKE_SOURCE_DIR}/src/classes
                            ${CMAKE_SOURCE_DIR}/src/max_flow
                            ${OpenCV_INCLUDE_DIRS})

file(GLOB graph_SOURCES STATIC "${CMAKE_SOURCE_DIR}/src/max_flow/*.cpp")
file(GLOB graph_HEADERS STATIC "${CMAKE_SOURCE_DIR}/src/max_flow/*.h")

add_library(code
    ${CMAKE_SOURCE_DIR}/src/classes/_image.h
    ${CMAKE_SOURCE_DIR}/src/classes/_image.cpp
    ${CMAKE_SOURCE_DIR}/src/classes/_panorama.h
    ${CMAKE_SOURCE_DIR}/src/classes/_panorama.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_toolbar.h
    ${CMAKE_SOURCE_DIR}/src/ui/_toolbar.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_create_menu.h
    ${CMAKE_SOURCE_DIR}/src/ui/_create_menu.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_flowbox.h
    ${CMAKE_SOURCE_DIR}/src/ui/_flowbox.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_main_windows.h
    ${CMAKE_SOURCE_DIR}/src/ui/_main_windows.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_gtk_vars.h
    ${CMAKE_SOURCE_DIR}/src/ui/_gtk_ops.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_gtk_ops.h
    ${CMAKE_SOURCE_DIR}/src/ui/_image_viewer.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_image_viewer.h
    ${CMAKE_SOURCE_DIR}/src/ui/_viewer_toolbar.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_viewer_toolbar.h
    ${CMAKE_SOURCE_DIR}/src/ui/_config.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_config.h
    ${CMAKE_SOURCE_DIR}/src/ui/_dialog.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_dialog.h
    ${CMAKE_SOURCE_DIR}/src/math/_img_manipulation.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_img_manipulation.h
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust_tools.h
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust_tools.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust_main.h
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust_main.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust_fast.h
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust_fast.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_gain_compensation.h
    ${CMAKE_SOURCE_DIR}/src/math/_gain_compensation.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_blending.h
    ${CMAKE_SOURCE_DIR}/src/math/_blending.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_stitch.h
    ${CMAKE_SOURCE_DIR}/src/math/_stitch.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_straightening.h
    ${CMAKE_SOURCE_DIR}/src/math/_straightening.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_graph_cut.h
    ${CMAKE_SOURCE_DIR}/src/math/_graph_cut.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_graph_cut_helper.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_graph_cut_helper.h
    ${CMAKE_SOURCE_DIR}/src/math/_homography.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_homography.h
    ${CMAKE_SOURCE_DIR}/src/math/_distance_cut.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_distance_cut.h
    ${CMAKE_SOURCE_DIR}/src/math/_projection.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_projection.h
    ${CMAKE_SOURCE_DIR}/src/system/_config_parser.cpp
    ${CMAKE_SOURCE_DIR}/src/system/_config_parser.h
    ${CMAKE_SOURCE_DIR}/src/system/_util.h
    ${CMAKE_SOURCE_DIR}/src/system/_util.cpp


    ${CMAKE_SOURCE_DIR}/src/test/_test.h
    ${CMAKE_SOURCE_DIR}/src/test/_test.cpp
)

add_executable(panorama src/main.cpp)
target_link_libraries(code ${GTK_LIBRARIES})

target_link_libraries(code ${OpenCV_LIBS})

target_link_libraries(code ${graph_SOURCES} ${graph_HEADERS})
target_link_libraries(code Eigen3::Eigen)
target_link_libraries(panorama code)
target_link_libraries(code OpenMP::OpenMP_CXX)


