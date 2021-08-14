//
// Created by vahagn on 13/08/2021.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_DELETED_H_
#define ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_DELETED_H_
#include "base_message.h"
#include <frame/key_frame.h>

namespace orb_slam3 {
namespace messages {

class KeyFrameDeleted : public BaseMessage {
 public:
  KeyFrameDeleted(const frame::KeyFrame * key_frame);
  MessageType Type() const override;
  const size_t id;
};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_DELETED_H_
