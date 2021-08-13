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
  MessageType Type() const override;
  const geometry::Pose position;
  const TimePoint time_point;
  const geometry::Pose & velocity;
};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_TRACKING_INFO_H_
