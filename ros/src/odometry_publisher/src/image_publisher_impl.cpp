//
// Created by vahagn on 21.11.21.
//

#include "image_publisher_impl.h"
#include <sensor_msgs/image_encodings.h>
namespace orb_slam3 {

ImagePublisherImpl::ImagePublisherImpl(ros::NodeHandle &n, std::string topic_name) : image_transport_(n) {
  publisher_ = image_transport_.advertise(topic_name, 1);
}

void ImagePublisherImpl::Publish(const std::vector<uint8_t> &data, int w, int h) {
  sensor_msgs::Image msg;
  msg.data = data;
  msg.height = h;
  msg.width = w;
  msg.step = w;
  msg.encoding = sensor_msgs::image_encodings::MONO8;
  msg.header.frame_id = "camera";
  publisher_.publish(msg);
}

}