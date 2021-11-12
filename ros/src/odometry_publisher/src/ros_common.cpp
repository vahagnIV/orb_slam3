//
// Created by vahagn on 12/11/2021.
//

#include "ros_common.h"
#include <ros/ros.h>
#include "odometry_publisher_impl.h"

namespace orb_slam3 {
namespace ros_publisher {


void Initialize(int argc, char * argv[]) {
  ros::init(argc, argv, "OdometryPublisherNode");
}

OdometryPublisher * CreatePublisher(const std::string & topic_name) {
  ros::NodeHandle n;
  return new OdometryPublisherImpl(n, topic_name);
}

}
}