//
// Created by vahagn on 13/08/2021.
//

#include "key_frame_deleted.h"
#include "serialization_utils.h"

namespace orb_slam3 {
namespace messages {

KeyFrameDeleted::KeyFrameDeleted(const frame::KeyFrame * key_frame) : id(key_frame->Id()) {
}

KeyFrameDeleted::KeyFrameDeleted(const std::vector<uint8_t> & serialized) : id(0) {
  INIT_DESERIALIZATION(serialized);
  COPY_FROM(source, id);
}

void KeyFrameDeleted::Serialize(std::vector<uint8_t> & out_serialized) const {
  INIT_SERIALIZATION(out_serialized, sizeof(id));
  COPY_TO(dest, id);
}

MessageType KeyFrameDeleted::Type() const {
  return KEYFRAME_DELETED;
}

}
}