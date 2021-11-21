//
// Created by vahagn on 13.08.21.
//

#include "key_frame_covisibility_updated.h"
#include "serialization_utils.h"

namespace orb_slam3 {
namespace messages {

KeyFrameCovisibilityUpdated::KeyFrameCovisibilityUpdated(size_t kf_id,
                                                         const std::vector<frame::KeyFrame *> & covisibles) :
    kf_id(kf_id),
    connected_key_frames(ToVector(covisibles)) {

}

KeyFrameCovisibilityUpdated::KeyFrameCovisibilityUpdated(const std::vector<uint8_t> & serialized) {
  INIT_DESERIALIZATION(serialized);
  COPY_FROM(source, kf_id);

  size_t connected_kf_size;
  COPY_FROM(source, connected_kf_size);
  connected_key_frames.resize(connected_kf_size);
  while (connected_kf_size--) {
    COPY_FROM(source, connected_key_frames[connected_kf_size - 1]);
  }
}

MessageType KeyFrameCovisibilityUpdated::Type() const {
  return KEYFRAME_COVISIBILITY_UPDATED;
}

std::vector<size_t> KeyFrameCovisibilityUpdated::ToVector(const std::vector<frame::KeyFrame *> & covisibles) {
  std::vector<size_t> result;
  std::transform(covisibles.begin(),
                 covisibles.end(),
                 std::back_inserter(result),
                 [](const frame::KeyFrame * kf) {
                   return kf->Id();
                 });
  return result;
}

void KeyFrameCovisibilityUpdated::Serialize(std::vector<uint8_t> & out_serialized) const {
  size_t connected_kf_size = connected_key_frames.size();
  INIT_SERIALIZATION(out_serialized, sizeof(kf_id) + connected_kf_size);
  COPY_TO(dest, kf_id);
  COPY_TO(dest, connected_kf_size);
  for (auto & q: connected_key_frames) {
    COPY_TO(dest, q);
  }
}

}
}