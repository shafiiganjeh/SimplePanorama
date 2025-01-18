cmake_minimum_required(VERSION 3.0)

project(panorama)

add_executable(panorama main.cpp)

install(TARGETS panorama RUNTIME DESTINATION bin)
