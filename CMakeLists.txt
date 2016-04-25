cmake_minimum_required(VERSION 2.8)

project(projector_tracker)

find_package(OpenCV 3 REQUIRED)

include_directories(include)
add_library(projector_tracker src/ProjectorTracker.cpp)
target_link_libraries(projector_tracker ${OpenCV_LIBS})

add_executable(projector_tracker_app main.cpp)
