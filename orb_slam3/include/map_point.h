//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_MAP_POINT_H_
#define ORB_SLAM3_INCLUDE_MAP_POINT_H_

#include <opencv2/opencv.hpp>

namespace orb_slam3 {

class MapPoint {
 public:
  cv::Point2f xi;
  cv::Point3f x;
};

}  // namespace orb_slam3

#endif  // ORB_SLAM3_INCLUDE_MAP_POINT_H_
