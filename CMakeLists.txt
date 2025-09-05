cmake_minimum_required(VERSION 3.1)
project(panorama)

find_package(PkgConfig REQUIRED)
find_package( OpenCV REQUIRED )
pkg_check_modules(GTK REQUIRED gtk+-3.0)
set (CMAKE_CXX_STANDARD 20)

find_package(OpenMP)
if (NOT OPENMP_FOUND)
	message(WARNING "OpenMP is not found. Will be slow.")
else()
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

configure_file(${CMAKE_SOURCE_DIR}/res/drag.jpg drag.jpg COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/res/Image_Crop_Icon.png Image_Crop_Icon.png COPYONLY)

include_directories(${GTK_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS} )

link_directories(${GTK_LIBRARY_DIRS})
add_definitions(${GTK_CFLAGS_OTHER})

find_package (Eigen3 3.4 REQUIRED NO_MODULE)

add_definitions(-O2)

include_directories(        ${CMAKE_SOURCE_DIR}/src/math
                            ${CMAKE_SOURCE_DIR}/src/ui
                            ${CMAKE_SOURCE_DIR}/src/classes
                            ${CMAKE_SOURCE_DIR}/src/max_flow)

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
    ${CMAKE_SOURCE_DIR}/src/math/_img_manipulation.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_img_manipulation.h
    ${CMAKE_SOURCE_DIR}/src/math/_util.h
    ${CMAKE_SOURCE_DIR}/src/math/_util.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust_tools.h
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust_tools.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust_main.h
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust_main.cpp
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
    ${CMAKE_SOURCE_DIR}/src/math/_config_parser.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_config_parser.h

)

add_executable(panorama src/main.cpp)

target_link_libraries(code ${OpenCV_LIBS}
                           ${GTK_LIBRARIES})

target_link_libraries(code ${graph_SOURCES} ${graph_HEADERS})

target_link_libraries(code Eigen3::Eigen)

target_link_libraries(panorama code)

install(TARGETS panorama RUNTIME DESTINATION bin)




