cmake_minimum_required(VERSION 2.8)
project(DBoW2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3  ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 ")

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DBoW2/DUtils/Random.h
  DBoW2/DUtils/Timestamp.h)
set(SRCS_DUTILS
  DBoW2/DUtils/Random.cpp
  DBoW2/DUtils/Timestamp.cpp)

find_package(OpenCV  QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

include(CMakePackageConfigHelpers)

install(TARGETS DBoW2 DESTINATION lib EXPORT DBoW2Target)
install(FILES DBoW2/BowVector.h 
	DBoW2/FClass.h 
	DBoW2/FeatureVector.h 
	DBoW2/FORB.h 
	DBoW2/ScoringObject.h 
	DBoW2/TemplatedVocabulary.h DESTINATION include/DBoW2)

install(FILES DBoW2/DUtils/Random.h 
	DBoW2/DUtils/Timestamp.h DESTINATION include/DBoW2/DUtils)

install(EXPORT DBoW2Target
	FILE DBoW2Target.cmake
	DESTINATION lib/cmake/DBoW2)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/DBoW2Config.cmake"
        INSTALL_DESTINATION "lib/cmake/DBoW2"
        NO_SET_AND_CHECK_MACRO
        NO_CHECK_REQUIRED_COMPONENTS_MACRO
        )


write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/DBoW2ConfigVersion.cmake"
        VERSION "${VERSION_MAJOR}.${VERSION_MINOR}"
        COMPATIBILITY AnyNewerVersion
)

install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/DBoW2Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/DBoW2ConfigVersion.cmake
        DESTINATION lib/cmake/DBoW2      )
