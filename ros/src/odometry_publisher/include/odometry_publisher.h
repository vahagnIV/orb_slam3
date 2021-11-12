//
// Created by vahagn on 11/11/2021.
//

#ifndef ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_SRC_ODOMETRY_PUBLISHER_H_
#define ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_SRC_ODOMETRY_PUBLISHER_H_

namespace orb_slam3 {
namespace ros_publisher {

class OdometryPublisher {
 public:
  virtual void Start() = 0;
  virtual void Stop() = 0;
  virtual ~OdometryPublisher() = default;
};

}
}

#endif //ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_SRC_ODOMETRY_PUBLISHER_H_
