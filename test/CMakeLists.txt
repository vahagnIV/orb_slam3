configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test/googletest-download)
if (result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif ()
find_package(GTest)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/test/googletest-download)
if (result)
    message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif ()

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/test/googletest-src
        ${CMAKE_BINARY_DIR}/test/googletest-build
        EXCLUDE_FROM_ALL)

set(ALL_TEST_SOURCE_FILES
        mock/mock_frame_base.h
        mock/mock_feature_extractor.h
        features/feature_tests.cpp
        features/feature_tests.h
        features/projection_search_iterator_tests.h
        features/projection_search_iterator_tests.cpp
        frame/monocular_frame_tests.cpp
        frame/monocular_frame_tests.h
        frame/frame_base_tests.h
        frame/frame_base_tests.cpp
        geometry/homography_estimator_tests.h
        geometry/homography_estimator_tests.cpp
        geometry/essential_estimator_tests.cpp
        geometry/essential_estimator_tests.h
        geometry/general_geometry_tests.cpp
        geometry/general_geometry_tests.h
        geometry/utils_tests.h
        geometry/utils_tests.cpp
        geometry/test_utils.cpp
        geometry/test_utils.h)
add_executable(orb_slam_unit_tests ${ALL_TEST_SOURCE_FILES})
target_link_libraries(orb_slam_unit_tests orbslam gmock gmock_main)

gtest_add_tests(TARGET orb_slam_unit_tests
        SOURCES ${ALL_TEST_SOURCE_FILES}
        TEST_SUFFIX Tests
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
