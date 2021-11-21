//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_CREATED_H_
#define ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_CREATED_H_

#include "base_message.h"
#include <frame/key_frame.h>

namespace orb_slam3 {
namespace messages {

class KeyFrameCreated : public BaseMessage {
 public:
  KeyFrameCreated(const frame::KeyFrame * key_frame);
  KeyFrameCreated(const std::vector<uint8_t> & serialized);
  MessageType Type() const override;
  size_t id;
  size_t map_id;
  geometry::Pose position;
  void Serialize(std::vector<uint8_t> & out_serialized) const override;


};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_CREATED_H_
