//
// Created by vahagn on 21.11.21.
//

#ifndef ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_INCLUDE_IMAGE_PUBLISHER_H_
#define ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_INCLUDE_IMAGE_PUBLISHER_H_
#include <vector>

namespace orb_slam3 {

class ImagePublisher {
 public:
  virtual void Publish(const std::vector<uint8_t> &data, int w, int h) = 0;
};

}

#endif //ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_INCLUDE_IMAGE_PUBLISHER_H_
