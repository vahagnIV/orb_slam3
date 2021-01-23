//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_TYPEDEFS_H_
#define ORB_SLAM3_INCLUDE_TYPEDEFS_H_
#include <chrono>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

namespace orb_slam3{

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> ImageRGB8U;
typedef cv::Mat ImageGray8U;
typedef cv::Mat ImageGray32F;

typedef cv::KeyPoint KeyPoint;
typedef cv::Mat DescriptorSet;

typedef cv::Mat TDistortionCoefficients;
typedef cv::Matx33f TIntrinsicMatrix;
typedef cv::Matx33f T3DTransformationMatrix;

typedef cv::Matx31f T3DVector;

typedef std::chrono::system_clock::time_point TimePoint;
}
#endif //ORB_SLAM3_INCLUDE_TYPEDEFS_H_
