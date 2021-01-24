//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_TYPEDEFS_H_
#define ORB_SLAM3_INCLUDE_TYPEDEFS_H_
#include <chrono>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

namespace orb_slam3{

typedef double precision_t;

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> TImageGray8U;
typedef Eigen::Matrix<precision_t , Eigen::Dynamic, Eigen::Dynamic> TImageGray;

typedef Eigen::Matrix<precision_t, Eigen::Dynamic, 2> TKeyPoints;
typedef cv::Mat DescriptorSet;

typedef Eigen::Matrix<precision_t, 3, 1 > TPoint3D;
typedef Eigen::Matrix<precision_t, 2, 1 > TPoint2D;




typedef cv::Matx31f T3DVector;

typedef std::chrono::system_clock::time_point TimePoint;
}
#endif //ORB_SLAM3_INCLUDE_TYPEDEFS_H_
