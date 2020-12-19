cmake_minimum_required(VERSION 3.10)
project(orb_slam3)

set(CMAKE_CXX_STANDARD 11)


find_package(OpenCV REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(orbslam main.cpp
        src/frame_base.cpp
        src/rgbd_frame.cpp
        src/orb_feature_extractor.cpp
        src/rgbd_camera.cpp
        src/atlas.cpp
        src/tracker.cpp
        src/map_point.cpp

        include/frame_base.h
        include/rgbd_frame.h
        include/orb_feature_extractor.h
        include/rgbd_camera.h
        include/atlas.h
        include/tracker.h
        include/map_point.h include/orb_vocabulary.h)

add_executable(orb_slam3 main.cpp src/atlas.cpp include/atlas.h src/map.cpp include/map.h src/tracker.cpp include/tracker.h src/map_point.cpp include/map_point.h include/constants.h)

target_link_libraries(orb_slam3 orbslam ${OpenCV_LIBS} dbow2)