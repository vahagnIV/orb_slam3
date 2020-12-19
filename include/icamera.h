//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_ICAMERA_H_
#define ORB_SLAM3_INCLUDE_ICAMERA_H_

#include <typedefs.h>

namespace orb_slam3 {

class ICamera {
 public:
  virtual void UndistortKeyPoints(const cv::Mat & points, cv::Mat & out_undistorted_points) = 0;
  virtual ~ICamera() = default;

};

}

#endif //ORB_SLAM3_INCLUDE_ICAMERA_H_
