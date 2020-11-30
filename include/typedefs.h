//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_TYPEDEFS_H_
#define ORB_SLAM3_INCLUDE_TYPEDEFS_H_
#include <opencv2/opencv.hpp>

namespace nvision{

typedef cv::Mat ImageRGB8U;
typedef cv::Mat ImageGray8U;
typedef cv::Mat ImageGray32F;

typedef cv::KeyPoint KeyPoint;
typedef cv::Mat DescriptorSet;

typedef cv::Mat DistortionCoefficients;
typedef cv::Mat IntrinsicMatrix;
}
#endif //ORB_SLAM3_INCLUDE_TYPEDEFS_H_
