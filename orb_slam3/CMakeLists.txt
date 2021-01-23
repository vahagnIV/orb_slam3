find_package(DBoW2 REQUIRED)
find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)

#include_directories(include ${DBoW2_INCLUDE_DIRS}) #${OpenCV_INCLUDE_DIRS}
include_directories(include g2o ${Eigen3_INCLUDE_DIRECTORIES} ${CHOLMOD_INCLUDE_DIR})

add_library(orbslam SHARED
        src/frame/frame_base.cpp
        src/frame/monocular_frame.cpp
#        src/rgbd_frame.cpp
        src/feature_extraction/orb_feature_extractor.cpp
        src/rgbd_camera.cpp
        src/atlas.cpp
        src/tracker.cpp
        src/map_point.cpp

        include/frame/frame_base.h
        include/frame/monocular_frame.h
        #        include/rgbd_frame.h
        include/feature_extraction/orb_feature_extractor.h
        include/feature_extraction/ifeature_extractor.h
        include/rgbd_camera.h
        include/atlas.h
        include/tracker.h
        include/map_point.h include/orb_vocabulary.h)

target_include_directories(orbslam INTERFACE include)


target_link_libraries(orbslam ${DBoW2_LIBS} g2o::core g2o::solver_cholmod g2o::types_sba g2o::opengl_helper Eigen3::Eigen pthread) #${OpenCV_LIBS}