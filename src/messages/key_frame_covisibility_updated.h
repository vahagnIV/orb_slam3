//
// Created by vahagn on 13.08.21.
//

#ifndef ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_COVISIBILITY_UPDATED_H_
#define ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_COVISIBILITY_UPDATED_H_

#include <unordered_set>

#include <messages/base_message.h>
#include <frame/key_frame.h>

namespace orb_slam3 {
namespace messages {

class KeyFrameCovisibilityUpdated : public BaseMessage {
 public:
  KeyFrameCovisibilityUpdated(size_t kf_id, const std::vector<frame::KeyFrame *> & covisibles);
  MessageType Type() const override;
  const size_t kf_id;
  const std::vector<size_t> connected_key_frames;
 private:
  static std::vector<size_t> ToVector(const std::vector<frame::KeyFrame *> & covisibles);

};

}
}

#endif //ORB_SLAM3_SRC_MESSAGES_KEY_FRAME_COVISIBILITY_UPDATED_H_
