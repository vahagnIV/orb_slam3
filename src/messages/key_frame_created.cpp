//
// Created by vahagn on 13/08/2021.
//

#include "key_frame_created.h"
#include "serialization_utils.h"

namespace orb_slam3 {
namespace messages {

KeyFrameCreated::KeyFrameCreated(const frame::KeyFrame * key_frame) : id(key_frame->Id()),
                                                                      map_id((size_t) key_frame->GetMap()),
                                                                      position(key_frame->GetPosition()) {

}

KeyFrameCreated::KeyFrameCreated(const std::vector<uint8_t> & serialized) {
  INIT_DESERIALIZATION(serialized);
  COPY_FROM(source, id);
  COPY_FROM(source, map_id);
  DeSerializePose(source, position);
}

MessageType KeyFrameCreated::Type() const {
  return KEYFRAME_CREATED;
}

void KeyFrameCreated::Serialize(std::vector<uint8_t> & out_serialized) const {

  INIT_SERIALIZATION(out_serialized, sizeof(id) + sizeof(map_id) + POSITION_SIZE);

  COPY_TO(dest, id);
  COPY_TO(dest, map_id);

  SerializePose(position, dest);

}

}
}