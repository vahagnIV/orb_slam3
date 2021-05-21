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
include_directories(${CMAKE_CURRENT_LIST_DIR} g2o ${Eigen3_INCLUDE_DIRECTORIES} ${CHOLMOD_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})


add_library(orbslam SHARED
        tracker.cpp
        local_mapper.cpp
        logging.cpp

#        frame/frame_base.cpp
#        frame/monocular_frame_old.cpp
        frame/monocular/base_monocular.cpp
        frame/monocular/monocular_frame.cpp
        frame/monocular/monocular_key_frame.cpp
        frame/covisibility_graph_node.cpp
        frame/observation.cpp
        frame/frame.cpp

        camera/monocular_camera.cpp
        camera/kannala_brandt_5.cpp
        camera/kannala_brandt_8.cpp
        camera/fish_eye.cpp

        features/orb_feature_extractor.cpp
        features/features.cpp
        features/bow_container.cpp
        features/matching/iterators/projection_search_iterator.cpp
        features/matching/iterators/projection_search_pointee.cpp
        features/matching/iterators/bow_to_iterator.cpp
        features/matching/orientation_validator.cpp
        features/matching/validators/triangulation_validator.cpp

        map/map_point.cpp
        map/map.cpp
        map/atlas.cpp

        geometry/two_view_reconstructor.cpp
        geometry/homography_matrix_estimator.cpp
        geometry/essential_matrix_estimator.cpp
        geometry/transformation_estimator_base.cpp
        geometry/utils.cpp

        optimization/edges/se3_project_xyz_pose.cpp
        optimization/edges/se3_project_xyz_pose_only.cpp
        optimization/vertices/map_point_vertex.cpp
        optimization/utils.cpp
        optimization/bundle_adjustment.cpp
        optimization/monocular_optimization.cpp

        debug/debug_utils.cpp

        typedefs.h
        constants.h
        tracker.h
        position_observer.h
        local_mapper.h

        geometry/two_view_reconstructor.h
        geometry/pose.h
        geometry/homography_matrix_estimator.h
        geometry/transfromation_estimator_base.h
        geometry/essential_matrix_estimator.h
        geometry/utils.h

        camera/monocular_camera.h
        camera/idistortion_model.h
        camera/kannala_brandt_5.h
        camera/kannala_brandt_8.h
        camera/fish_eye.h
        camera/icamera.h

#        frame/frame_base.h
#        frame/monocular_frame_old.h
        frame/frame_type.h
        frame/covisibility_graph_node.h
        frame/visible_map_point.h
        frame/observation.h
#        frame/monocular_observation.h

        features/key_point.h
        features/orb_feature_extractor.h
        features/ifeature_extractor.h
        features/features.h
        features/matching/imatcher.h
        features/key_point.h
        features/matching/imatcher.h
        features/matching/second_nearest_neighbor_matcher.hpp
        features/match.h
        frame/iframe_database.h
        features/bow_vocabulary.h
        features/bow_container.h
        features/matching/iterators/bow_to_iterator.h
        features/matching/iterators/bow_to_pointee.h
        features/matching/orientation_validator.h
        features/matching/iterators/vector_from_iterator.h
        features/matching/iterators/area_to_pointee.h
        features/matching/iterators/vector_from_pointee.h
        features/matching/iterators/area_to_iterator.h
        features/matching/iterators/projection_search_iterator.h
        features/matching/iterators/projection_search_pointee.h

        map/map_point.h
        map/atlas.h
        map/map.h

        optimization/edges/se3_project_xyz_pose.h
        optimization/edges/se3_project_xyz_pose_only.h
        optimization/bundle_adjustment.h

        debug/debug_utils.h

        orb_vocabulary.h
        identifiable.h
        logging.h features/matching/iterators/triangulate_from_pointee.cpp features/matching/iterators/triangulate_from_pointee.h features/matching/iterators/triangulate_from_iterator.cpp features/matching/iterators/triangulate_from_iterator.h)

target_include_directories(orbslam INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(orbslam
        g2o::core
        g2o::solver_cholmod
        g2o::types_sba
        g2o::opengl_helper
        Eigen3::Eigen
        ${OpenCV_LIBS}
        DBoW2
        pthread) #