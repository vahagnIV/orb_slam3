//
// Created by vahagn on 11/11/2021.
//

#include "odometry_publisher.h"


namespace orb_slam3 {

OdometryPublisher::OdometryPublisher(ros::NodeHandle & n) {
  n.advertise<ros::Str>()

}

}