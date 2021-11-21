//
// Created by vahagn on 12/11/2021.
//

#ifndef ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_INCLUDE_ROS_COMMON_H_
#define ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_INCLUDE_ROS_COMMON_H_

#include <string>
#include <odometry_publisher.h>
#include <image_publisher.h>

namespace orb_slam3 {
namespace ros_publisher {

void Initialize(int argc, char *argv[]);
OdometryPublisher *CreateOdometryPublisher(const std::string &topic_name);
ImagePublisher *CreateImagePublisher(const std::string &topic_name);

}
}

#endif //ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_INCLUDE_ROS_COMMON_H_
