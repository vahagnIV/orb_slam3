cmake_minimum_required(VERSION 3.13)
project(orb_slam3)

set(CMAKE_CXX_STANDARD 11)
message("${PROJECT_SOURCE_DIR}/cmake/modules")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(g2o REQUIRED)

add_definitions(-DMULTITHREADED)

enable_testing()
include_directories(external ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR})
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(drawer)

add_executable(test_p main.cpp main_utils/message_print_functions.cpp )

target_link_libraries(test_p orbslam
        ${Boost_FILESYSTEM_LIBRARY}
        ${g2o_LIBRARIES}
        ${Boost_SYSTEM_LIBRARY}
        drawer)
