cmake_minimum_required(VERSION 3.0.2)
project(panorama)

find_package(PkgConfig REQUIRED)
find_package( OpenCV REQUIRED )
pkg_check_modules(GTK REQUIRED gtk+-3.0)


configure_file(${CMAKE_SOURCE_DIR}/res/drag.jpg drag.jpg COPYONLY)

include_directories(${GTK_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS} )

link_directories(${GTK_LIBRARY_DIRS})
add_definitions(${GTK_CFLAGS_OTHER})

find_package (Eigen3 3.4 REQUIRED NO_MODULE)




include_directories(        ${CMAKE_SOURCE_DIR}/src/math
                            ${CMAKE_SOURCE_DIR}/src/ui
                            ${CMAKE_SOURCE_DIR}/src/classes)

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
    ${CMAKE_SOURCE_DIR}/src/math/_img_manipulation.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_img_manipulation.h
    ${CMAKE_SOURCE_DIR}/src/math/_maths.h
    ${CMAKE_SOURCE_DIR}/src/math/_maths.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust.h
    ${CMAKE_SOURCE_DIR}/src/math/_bundle_adjust.cpp
)

add_executable(panorama src/main.cpp)

target_link_libraries(code ${OpenCV_LIBS}
                               ${GTK_LIBRARIES})

target_link_libraries(code Eigen3::Eigen)

target_link_libraries(panorama code)

install(TARGETS panorama RUNTIME DESTINATION bin)




