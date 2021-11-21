//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_TRACKING_INFO_H_
#define ORB_SLAM3_SRC_MESSAGES_TRACKING_INFO_H_

#include "base_message.h"
#include <geometry/pose.h>

namespace orb_slam3 {
namespace messages {

class TrackingInfo : public BaseMessage {
 public:
  TrackingInfo(const geometry::Pose & pose,
               const TimePoint time_point,
               const geometry::Pose & velocity);
  TrackingInfo(const std::vector<uint8_t> & serialized);
  void Serialize(std::vector<uint8_t> & out_serialized) const override;
  MessageType Type() const override;
  geometry::Pose position;
  TimePoint time_point;
  geometry::Pose velocity;
};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_TRACKING_INFO_H_
