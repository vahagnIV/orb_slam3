//
// Created by vahagn on 13.08.21.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_POSITION_UPDATED_H_
#define ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_POSITION_UPDATED_H_
#include "base_message.h"
#include <frame/key_frame.h>

namespace orb_slam3 {
namespace messages {
class KeyFramePositionUpdated : public BaseMessage {
 public:
  KeyFramePositionUpdated(const frame::KeyFrame * keyframe);
  MessageType Type() const override;
  const geometry::Pose position;

};
}
}
#endif //ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_POSITION_UPDATED_H_
