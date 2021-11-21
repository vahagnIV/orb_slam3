//
// Created by vahagn on 21.11.21.
//

#ifndef ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_SRC_IMAGE_PUBLISHER_IMPL_H_
#define ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_SRC_IMAGE_PUBLISHER_IMPL_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <typedefs.h>
#include <image_publisher.h>

namespace orb_slam3 {

class ImagePublisherImpl: public ImagePublisher {
 public:
  ImagePublisherImpl(ros::NodeHandle &n, std::string topic_name);
  void Publish(const std::vector<uint8_t> &data, int w, int h) override;
 private:
  image_transport::Publisher publisher_;
  image_transport::ImageTransport image_transport_;
};

}

#endif //ORB_SLAM3_ROS_SRC_ODOMETRY_PUBLISHER_SRC_IMAGE_PUBLISHER_IMPL_H_
