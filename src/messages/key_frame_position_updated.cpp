//
// Created by vahagn on 13.08.21.
//

#include "key_frame_position_updated.h"

namespace orb_slam3 {
namespace messages {

KeyFramePositionUpdated::KeyFramePositionUpdated(const frame::KeyFrame *keyframe)
    : id(keyframe->Id()), position(keyframe->GetPosition()) {
}

MessageType KeyFramePositionUpdated::Type() const {
  return KEYFRAME_POSITION_UPDATED;
}

}
}