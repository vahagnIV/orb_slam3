//
// Created by vahagn on 13/08/2021.
//

#include "key_frame_created.h"

namespace orb_slam3 {
namespace messages {

const MessageType KeyFrameCreated::type = KEYFRAME_CREATED;

KeyFrameCreated::KeyFrameCreated(const frame::KeyFrame * key_frame) : id(key_frame->Id()),
                                                                      map_id((size_t) key_frame->GetMap()),
                                                                      position(key_frame->GetPosition()) {

}

MessageType KeyFrameCreated::Type() const {
  return KEYFRAME_CREATED;
}

}
}