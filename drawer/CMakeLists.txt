find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLFW3 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

add_library(drawer SHARED
        drawer_impl.cpp
        drawer.cpp
        node.cpp
        graph.cpp
        edge.cpp
        shaders/shader_repository.cpp
        map_point_node.cpp
        key_frame_node.cpp)
target_link_libraries(drawer
        ${OpenGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${GLFW3_LIBRARY}
        orbslam)
target_include_directories(orbslam INTERFACE ${CMAKE_CURRENT_LIST_DIR})