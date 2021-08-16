find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_library(drawer SHARED drawer.cpp)
target_link_libraries(drawer ${Pangolin_LIBRARIES} orbslam)