cmake_minimum_required(VERSION 3.13)
project(orb_slam3)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)


#find_package(OpenCV REQUIRED)
include_directories(external ${OpenCV_INCLUDE_DIRS})
add_subdirectory(orb_slam3)

add_executable(test_p main.cpp )

target_link_libraries(test_p orbslam
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY})