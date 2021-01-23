cmake_minimum_required(VERSION 3.10)
project(orb_slam3)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)


#find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_subdirectory(orb_slam3)

add_executable(test main.cpp )

target_link_libraries(test orbslam
        ${OpenCV_LIBS}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY})