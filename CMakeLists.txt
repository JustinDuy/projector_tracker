cmake_minimum_required(VERSION 2.8)

project(projector_tracker)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
if(CMAKE_COMPILER_IS_GNUCC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif(CMAKE_COMPILER_IS_GNUCC)
set (OpenCV_DIR /opt/ros/jade/share/OpenCV-3.1.0-dev)
find_package(OpenCV 3 REQUIRED)
find_package(Qt4 REQUIRED)

include(${QT_USE_FILE})

include_directories(include)

add_library(projector_tracker src/ProjectorTracker.cpp)
target_link_libraries(projector_tracker ${OpenCV_LIBS})


add_library(camera_calibration src/CameraCalibration.cpp)
target_link_libraries(camera_calibration ${OpenCV_LIBS})

add_library(projector_calibration src/ProjectorCalibration.cpp)
target_link_libraries(projector_calibration ${OpenCV_LIBS})

add_library(hw_interface src/CameraInterface.cpp src/ProjectorInterface.cpp src/CameraProjectorInterface.cpp src/ImageLabel.cpp)
target_link_libraries(hw_interface ${OpenCV_LIBS} ${QT_LIBRARIES})

# add_executable(projector_tracker_app nodes/node.cpp)


# target_link_libraries(projector_tracker_app projector_tracker hw_interface)

# TEST
add_executable(test_node test/test.cpp)
target_link_libraries(test_node hw_interface projector_tracker)
add_executable(calib_cam test/calib_cam.cpp)
target_link_libraries(calib_cam hw_interface camera_calibration  )
add_executable(capture test/capture.cpp)
target_link_libraries(capture hw_interface camera_calibration  )
add_executable(calib_proj test/calib_proj.cpp)
target_link_libraries(calib_proj hw_interface projector_calibration  )