//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_TYPEDEFS_H_
#define ORB_SLAM3_INCLUDE_TYPEDEFS_H_

#ifndef DUMP
#define  DUMP
#endif

// == stl ===
#include <chrono>
#include <cstddef>

// == Eigen ===
#include <Eigen/Eigen>

namespace orb_slam3 {
#define CAMERA_PARAMS_COUNT 4

typedef double precision_t;

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TImageGray8U;
typedef Eigen::Matrix<precision_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TImageGray;

/*!
 * Although in the sense of C++ object a 2D point of projective space in homogenous coordinates
 * is indistinguishable from a 3D point, we will use another typedef for that
 */
typedef Eigen::Matrix<precision_t, 3, 1> TPoint3D;
typedef Eigen::Matrix<precision_t, 3, 1> HomogenousPoint;

typedef Eigen::Matrix<precision_t, 2, 1> TPoint2D;

typedef Eigen::Matrix<precision_t, 3, 1> TVector3D;
typedef Eigen::Matrix<precision_t, 2, 1> T2DVector;

typedef Eigen::Matrix<precision_t, 3, 3> TMatrix33;

typedef std::chrono::system_clock::time_point TimePoint;
}
#endif //ORB_SLAM3_INCLUDE_TYPEDEFS_H_
