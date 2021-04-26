find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# Temporaty for the time being we use opencv for ORB feature extraction
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

#include_directories(include ${DBoW2_INCLUDE_DIRS}) #${OpenCV_INCLUDE_DIRS}
include_directories(include g2o ${Eigen3_INCLUDE_DIRECTORIES} ${CHOLMOD_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

add_library(orbslam SHARED
        src/identifiable.cpp
        src/tracker.cpp
        src/local_mapper.cpp
        src/logging.cpp

        src/frame/frame_base.cpp
        src/frame/monocular_frame.cpp
        src/frame/covisibility_graph_node.cpp

        src/camera/monocular_camera.cpp
        src/camera/kannala_brandt_5.cpp
        src/camera/kannala_brandt_8.cpp
        src/camera/fish_eye.cpp

        src/features/orb_feature_extractor.cpp
        src/features/features.cpp
        src/features/bow_container.cpp
        src/features/matching/iterators/projection_search_iterator.cpp
        src/features/matching/iterators/projection_search_pointee.cpp
        src/features/matching/iterators/bow_to_iterator.cpp
        src/features/matching/validators/bow_match_tracking_validator.cpp
        src/features/matching/orientation_validator.cpp
        src/features/matching/validators/bow_match_local_mapping_validator.cpp

        src/map/map_point.cpp
        src/map/map.cpp
        src/map/atlas.cpp

        src/geometry/two_view_reconstructor.cpp
        src/geometry/homography_matrix_estimator.cpp
        src/geometry/essential_matrix_estimator.cpp
        src/geometry/transformation_estimator_base.cpp
        src/geometry/utils.cpp

        src/optimization/edges/se3_project_xyz_pose.cpp
        src/optimization/edges/se3_project_xyz_pose_only.cpp
        src/optimization/bundle_adjustment.cpp
        src/optimization/pose_optimization.cpp

        src/debug/debug_utils.cpp


        include/typedefs.h
        include/constants.h
        include/tracker.h
        include/position_observer.h
        include/local_mapper.h

        include/geometry/two_view_reconstructor.h
        include/geometry/pose.h
        include/geometry/homography_matrix_estimator.h
        include/geometry/transfromation_estimator_base.h
        include/geometry/essential_matrix_estimator.h
        include/geometry/utils.h

        include/camera/monocular_camera.h
        include/camera/idistortion_model.h
        include/camera/kannala_brandt_5.h
        include/camera/kannala_brandt_8.h
        include/camera/fish_eye.h
        include/camera/icamera.h

        include/frame/frame_base.h
        include/frame/monocular_frame.h
        include/frame/frame_type.h
        include/frame/frame_link.h
        include/frame/covisibility_graph_node.h

        include/features/key_point.h
        include/features/orb_feature_extractor.h
        include/features/ifeature_extractor.h
        include/features/features.h
        include/features/matching/imatcher.h
        include/features/key_point.h
        include/features/matching/imatcher.h
        include/features/matching/second_nearest_neighbor_matcher.h
        include/features/match.h
        include/frame/iframe_database.h
        include/features/bow_vocabulary.h
        include/features/bow_container.h
        include/features/matching/iterators/bow_to_iterator.h
        include/features/matching/iterators/bow_to_pointee.h
        include/features/matching/validators/iindex_validator.h
        include/features/matching/validators/bow_match_tracking_validator.h
        include/features/matching/validators/imatch_result_validator.h
        include/features/matching/orientation_validator.h
        include/features/matching/validators/bow_match_local_mapping_validator.h
        include/features/matching/iterators/vector_from_iterator.h
        include/features/matching/iterators/area_to_pointee.h
        include/features/matching/iterators/vector_from_pointee.h
        include/features/matching/iterators/area_to_iterator.h
        include/features/matching/iterators/projection_search_iterator.h
        include/features/matching/iterators/projection_search_pointee.h

        include/map/map_point.h
        include/map/atlas.h
        include/map/map.h

        include/optimization/edges/se3_project_xyz_pose.h
        include/optimization/bundle_adjustment.h
        include/optimization/pose_optimization.h
        include/optimization/edges/se3_project_xyz_pose_only.h

        include/debug/debug_utils.h

        include/orb_vocabulary.h
        include/identifiable.h
        include/logging.h src/features/matching/iterators/triangulate_from_pointee.cpp include/features/matching/iterators/triangulate_from_pointee.h src/features/matching/iterators/triangulate_from_iterator.cpp include/features/matching/iterators/triangulate_from_iterator.h)

target_include_directories(orbslam INTERFACE include)


target_link_libraries(orbslam
        g2o::core
        g2o::solver_cholmod
        g2o::types_sba
        g2o::opengl_helper
        Eigen3::Eigen
        ${OpenCV_LIBS}
        DBoW2
        pthread) #