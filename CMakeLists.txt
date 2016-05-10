cmake_minimum_required(VERSION 2.8)

project(projector_tracker)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV 3 REQUIRED)

include_directories(include)

add_library(projector_tracker src/ProjectorTracker.cpp)
target_link_libraries(projector_tracker ${OpenCV_LIBS})

add_library(camera_interface src/CameraInterface.cpp src/CameraProjectorInterface.cpp)
target_link_libraries(camera_interface ${OpenCV_LIBS})

add_executable(projector_tracker_app nodes/node.cpp)
target_link_libraries(projector_tracker_app projector_tracker camera_interface)
