//
// Created by vahagn on 11/30/20.
//

#ifndef ORB_SLAM3_INCLUDE_ICAMERA_H_
#define ORB_SLAM3_INCLUDE_ICAMERA_H_

#include <typedefs.h>

namespace nvision {

class ICamera {
 public:
  virtual void UndistortKeyPoints(std::vector<KeyPoint> & in_out_keypoints) = 0;

};

}

#endif //ORB_SLAM3_INCLUDE_ICAMERA_H_
