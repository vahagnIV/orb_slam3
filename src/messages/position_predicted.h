//
// Created by vahagn on 18/04/2022.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_POSITION_PREDICTED_H_
#define ORB_SLAM3_SRC_MESSAGES_POSITION_PREDICTED_H_
#include "base_message.h"
#include <frame/base_frame.h>

namespace orb_slam3 {
namespace messages {

class PositionPredicted : public BaseMessage {
 public:
  PositionPredicted(frame::BaseFrame * base_frame);
  MessageType Type() const override;
  void Serialize(std::vector<uint8_t> & out_serialized) const override;

  geometry::Pose pose;

};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_POSITION_PREDICTED_H_
