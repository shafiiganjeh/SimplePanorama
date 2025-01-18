cmake_minimum_required(VERSION 3.0.2)
project(panorama)

find_package(PkgConfig REQUIRED)
find_package( OpenCV REQUIRED )
pkg_check_modules(GTK REQUIRED gtk+-3.0)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${GTK_INCLUDE_DIRS})
link_directories(${GTK_LIBRARY_DIRS})
add_definitions(${GTK_CFLAGS_OTHER})


add_executable(panorama main.cpp)
target_link_libraries(panorama ${GTK_LIBRARIES})
target_link_libraries(panorama ${OpenCV_LIBS} )

install(TARGETS panorama RUNTIME DESTINATION bin)




