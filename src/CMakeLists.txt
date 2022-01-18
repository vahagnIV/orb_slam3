find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(Boost COMPONENTS system filesystem REQUIRED)

# Temporaty for the time being we use opencv for ORB feature extraction
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")


if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} ")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

#include_directories(include ${DBoW2_INCLUDE_DIRS}) #${OpenCV_INCLUDE_DIRS}
include_directories(${CMAKE_CURRENT_LIST_DIR}
        g2o
        ${Eigen3_INCLUDE_DIRECTORIES}
        ${CHOLMOD_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS})


add_library(orbslam SHARED
        tracker.cpp
        local_mapper.cpp
        loop_merge_detector.cpp
        logging.cpp
        settings.cpp
        profiler.cpp

        frame/key_frame.cpp
        frame/base_frame.cpp
        frame/monocular/base_monocular.cpp
        frame/monocular/monocular_frame.cpp
        frame/monocular/monocular_key_frame.cpp
        frame/covisibility_graph_node.cpp
        frame/observation.cpp
        frame/frame.cpp
        frame/database/DBoW2/dbo_w2_database.cpp
        frame/sensor_constants.cpp

        camera/monocular_camera.cpp
        camera/distortions/barrel5.cpp
        camera/distortions/barrel8.cpp
        camera/distortions/fish_eye.cpp

        features/orb_feature_extractor.cpp
        features/features.cpp
        features/key_point.cpp
        features/matching/iterators/projection_search_iterator.cpp
        features/matching/iterators/projection_search_pointee.cpp
        features/handlers/DBoW2/bow_to_iterator.cpp
        features/handlers/base_feature_handler.cpp
        features/matching/validators/orientation_validator.cpp
        features/handlers/DBoW2/dbo_w2_handler.cpp
        features/bow/dbo_w2_vocabulary.cpp

        messages/message_type.cpp
        messages/base_message.cpp
        messages/message_processor.cpp
        messages/key_frame_created.cpp
        messages/key_frame_deleted.cpp
        messages/tracking_info.cpp
        messages/map_created.cpp
        messages/map_point_created.cpp
        messages/map_point_deleted.cpp
        messages/observation_added.cpp
        messages/observation_deleted.cpp
        messages/key_frame_position_updated.cpp
        messages/key_frame_covisibility_updated.cpp
        messages/map_point_geometry_updated.cpp
        messages/serialization_utils.cpp

        map/map_point.cpp
        map/map.cpp
        map/atlas.cpp

        geometry/two_view_reconstructor.cpp
        geometry/homography_matrix_estimator.cpp
        geometry/essential_matrix_estimator.cpp
        geometry/transformation_estimator_base.cpp
        geometry/utils.cpp
        geometry/pose.cpp
        geometry/sim3_solver.cpp
        geometry/ransac_sim3_solver.cpp
        geometry/random_subset_generator.cpp
        geometry/sim3_transformation.cpp

        factories/camera_factory.cpp
        factories/key_frame_factory.cpp
        factories/feature_handler_factory.cpp
        factories/distortion_model_factory.cpp
        factories/frame_factory.cpp
        factories/feature_extractor_factory.cpp
        factories/keyframe_database_factory.cpp

        serialization/serialization_context.cpp

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
        camera/distortions/idistortion_model.h
        camera/distortions/barrel5.h
        camera/distortions/barrel8.h
        camera/distortions/fish_eye.h
        camera/icamera.h

        frame/frame_type.h
        frame/covisibility_graph_node.h
        frame/map_point_visibility_params.h
        frame/observation.h

        features/key_point.h
        features/orb_feature_extractor.h
        features/ifeature_extractor.h
        features/features.h
        features/matching/imatcher.h
        features/key_point.h
        features/matching/imatcher.h
        features/matching/second_nearest_neighbor_matcher.hpp
        features/match.h
        features/bow/bow_vocabulary.h
        features/handlers/DBoW2/bow_to_iterator.h
        features/handlers/DBoW2/bow_to_pointee.h
        features/matching/validators/orientation_validator.h
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

        identifiable.h
        logging.h)

#target_include_directories(orbslam INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_include_directories(orbslam PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)


target_link_libraries(orbslam PUBLIC Eigen3::Eigen)

target_link_libraries(orbslam PRIVATE
        g2o::core
        g2o::solver_cholmod
        g2o::types_sba
        g2o::types_sim3
        Eigen3::Eigen
        ${OpenCV_LIBS}
        DBoW2
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        pthread) #

install(FILES constants.h
        identifiable.h
        image_utils.h
        local_mapper.h
        logging.h
        loop_merge_detector.h
        observer.h
        observable.h
        position_observer.h
        settings.h
        tracker.h
        tracking_result.h
        typedefs.h
        DESTINATION include/${PROJECT_NAME})


install(FILES
        ../external/concurrentqueue/blockingconcurrentqueue.h
        ../external/concurrentqueue/concurrentqueue.h
        DESTINATION include/concurrentqueue)

install(FILES
        ../external/json/json.hpp
        DESTINATION include/json)

install(FILES
        camera/icamera.h
        camera/camera_type.h
        camera/monocular_camera.h
        camera/rgbd_camera.h
        DESTINATION include/${PROJECT_NAME}/camera)

install(FILES
        camera/distortions/barrel5.h
        camera/distortions/barrel8.h
        camera/distortions/fish_eye.h
        camera/distortions/idistortion_model.h
        camera/distortions/distortion_model_type.h
        DESTINATION include/${PROJECT_NAME}/camera/distortions)

install(FILES
        debug/debug_utils.h
        DESTINATION include/${PROJECT_NAME}/debug)

install(FILES
        factories/camera_factory.h
        factories/distortion_model_factory.h
        factories/feature_extractor_factory.h
        factories/feature_handler_factory.h
        factories/frame_factory.h
        factories/key_frame_factory.h
        factories/keyframe_database_factory.h
        DESTINATION include/${PROJECT_NAME}/factories)

install(FILES
        features/feature_extractor_type.h
        features/features.h
        features/ifeature_extractor.h
        features/key_point.h
        features/match.h
        features/orb_feature_extractor.h
        DESTINATION include/${PROJECT_NAME}/features)

install(FILES
        features/bow/bow_vocabulary.h
        features/bow/dbo_w2_vocabulary.h
        DESTINATION include/${PROJECT_NAME}/features/bow)

install(FILES
        features/handlers/handler_type.h
        features/handlers/base_feature_handler.h
        DESTINATION include/${PROJECT_NAME}/features/handlers)

install(FILES
        features/handlers/DBoW2/bow_to_iterator.h
        features/handlers/DBoW2/bow_to_pointee.h
        features/handlers/DBoW2/dbo_w2_handler.h
        DESTINATION include/${PROJECT_NAME}/features/handlers/DBoW2)

install(FILES
        features/matching/imatcher.h
        features/matching/second_nearest_neighbor_matcher.hpp
        DESTINATION include/${PROJECT_NAME}/features/matching)

install(FILES
        features/matching/iterators/area_to_iterator.h
        features/matching/iterators/area_to_pointee.h
        features/matching/iterators/projection_search_iterator.h
        features/matching/iterators/vector_from_iterator.h
        features/matching/iterators/projection_search_pointee.h
        features/matching/iterators/vector_from_pointee.h
        DESTINATION include/${PROJECT_NAME}/features/matching/iterators)

install(FILES
        features/matching/validators/orientation_validator.h
        features/matching/validators/imatch_validator.h
        DESTINATION include/${PROJECT_NAME}/features/matching/validators)

install(FILES
        frame/base_frame.h
        frame/covisibility_graph_node.h
        frame/frame.h
        frame/frame_type.h
        frame/key_frame.h
        frame/map_point_visibility_params.h
        frame/observation.h
        frame/sensor_constants.h
        DESTINATION include/${PROJECT_NAME}/frame)

install(FILES
        frame/database/ikey_frame_database.h
        frame/database/keyframe_database_type.h
        DESTINATION include/${PROJECT_NAME}/frame/database)

install(FILES
        frame/database/DBoW2/dbo_w2_database.h
        DESTINATION include/${PROJECT_NAME}/frame/database/DBoW2)

install(FILES
        frame/monocular/base_monocular.h
        frame/monocular/monocular_frame.h
        frame/monocular/monocular_key_frame.h
        DESTINATION include/${PROJECT_NAME}/frame/monocular)

install(FILES
        geometry/essential_matrix_estimator.h
        geometry/homography_matrix_estimator.h
        geometry/pose.h
        geometry/random_subset_generator.h
        geometry/ransac_sim3_solver.h
        geometry/rigid_object.h
        geometry/sim3_solver.h
        geometry/sim3_transformation.h
        geometry/transfromation_estimator_base.h
        geometry/two_view_reconstructor.h
        geometry/utils.h
        DESTINATION include/${PROJECT_NAME}/geometry)

install(FILES
        map/atlas.h
        map/map.h
        map/map_point.h
        DESTINATION include/${PROJECT_NAME}/map)

install(FILES
        messages/base_message.h
        messages/key_frame_covisibility_updated.h
        messages/key_frame_created.h
        messages/key_frame_deleted.h
        messages/key_frame_position_updated.h
        messages/map_created.h
        messages/map_point_created.h
        messages/map_point_deleted.h
        messages/map_point_geometry_updated.h
        messages/message_processor.h
        messages/message_type.h
        messages/messages.h
        messages/observation_added.h
        messages/observation_deleted.h
        messages/serialization_utils.h
        messages/tracking_info.h
        DESTINATION include/${PROJECT_NAME}/messages)

install(FILES
        optimization/bundle_adjustment.h
        optimization/monocular_optimization.h
        optimization/utils.h
        DESTINATION include/${PROJECT_NAME}/optimization)

install(FILES
        optimization/edges/se3_project_xyz_pose.h
        optimization/edges/se3_project_xyz_pose_only.h
        optimization/edges/ba_binary_edge.h
        optimization/edges/ba_unary_edge.h
        DESTINATION include/${PROJECT_NAME}/optimization/edges)

install(FILES
        optimization/vertices/map_point_vertex.h
        optimization/vertices/frame_vertex.h
        DESTINATION include/${PROJECT_NAME}/optimization/vertices)

install(FILES
        serialization/serialization_context.h
        DESTINATION include/${PROJECT_NAME}/serialization)

install(FILES
        utils/thread_safe_queue.h
        DESTINATION include/${PROJECT_NAME}/utils)

include(CMakePackageConfigHelpers)

install(TARGETS orbslam DESTINATION lib EXPORT orbslamTarget)

install(EXPORT orbslamTarget
        FILE orbslamTarget.cmake
        DESTINATION lib/cmake/orbslam
        )

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/orbslamConfig.cmake"
        INSTALL_DESTINATION "lib/cmake/orbslam"
        NO_SET_AND_CHECK_MACRO
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
        )

write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/orbslamConfigVersion.cmake"
        VERSION "${Tutorial_VERSION_MAJOR}.${Tutorial_VERSION_MINOR}"
        COMPATIBILITY AnyNewerVersion
)

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/orbslamConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/orbslamConfigVersion.cmake
        DESTINATION lib/cmake/orbslam
        )