//
// Created by vahagn on 13.08.21.
//

#include "key_frame_position_updated.h"
#include "serialization_utils.h"

namespace orb_slam3 {
namespace messages {

KeyFramePositionUpdated::KeyFramePositionUpdated(const frame::KeyFrame * keyframe)
    : id(keyframe->Id()), position(keyframe->GetPosition()) {
}

KeyFramePositionUpdated::KeyFramePositionUpdated(const std::vector<uint8_t> & serialized) {
  INIT_DESERIALIZATION(serialized);
  COPY_FROM(source, id);
  DeSerializePose(source, position);
}

void KeyFramePositionUpdated::Serialize(std::vector<uint8_t> & out_serialized) const{
  INIT_SERIALIZATION(out_serialized, sizeof(id) + POSITION_SIZE)


  COPY_TO(dest, id);
  SerializePose(position, dest);
}

MessageType KeyFramePositionUpdated::Type() const {
  return KEYFRAME_POSITION_UPDATED;
}

}
}