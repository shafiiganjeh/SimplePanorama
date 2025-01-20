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

include_directories(        ${CMAKE_SOURCE_DIR}/src/math
                            ${CMAKE_SOURCE_DIR}/src/ui)

add_library(code
    ${CMAKE_SOURCE_DIR}/src/ui/_gtk_vars.h
    ${CMAKE_SOURCE_DIR}/src/ui/_build_ui.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_build_ui.h
    ${CMAKE_SOURCE_DIR}/src/ui/_gtk_ops.cpp
    ${CMAKE_SOURCE_DIR}/src/ui/_gtk_ops.h
    ${CMAKE_SOURCE_DIR}/src/math/_img_manipulation.cpp
    ${CMAKE_SOURCE_DIR}/src/math/_img_manipulation.h
)

add_executable(panorama src/main.cpp)

target_link_libraries(code ${OpenCV_LIBS}
                               ${GTK_LIBRARIES})

target_link_libraries(panorama code)

install(TARGETS panorama RUNTIME DESTINATION bin)




