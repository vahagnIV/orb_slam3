cmake_minimum_required(VERSION 3.14)
project(orb_slam3)

set(CMAKE_CXX_STANDARD 11)


find_package(OpenCV REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(orbslam main.cpp
        src/frame_base.cpp
        src/rgbd_frame.cpp
        src/orb_feature_extractor.cpp
        src/pinhole_camera.cpp)

add_executable(orb_slam3 main.cpp)

target_link_libraries(orb_slam3 orbslam ${OpenCV_LIBS})