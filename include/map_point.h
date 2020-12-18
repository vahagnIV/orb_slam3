//
// Created by vahagn on 12/8/20.
//

#ifndef ORB_SLAM3_INCLUDE_MAP_POINT_H_
#define ORB_SLAM3_INCLUDE_MAP_POINT_H_

#include <opencv2/opencv.hpp>

namespace nvision {

class MapPoint {
 public:
  cv::Point2f pt;
  float depth;
};

}

#endif //ORB_SLAM3_INCLUDE_MAP_POINT_H_
