//
// Created by vahagn on 13/08/2021.
//

#include "key_frame_deleted.h"

namespace orb_slam3 {
namespace messages {

KeyFrameDeleted::KeyFrameDeleted(const frame::KeyFrame * key_frame) : id(key_frame->Id()) {
}

MessageType KeyFrameDeleted::Type() const {
  return KEYFRAME_DELETED;
}

}
}