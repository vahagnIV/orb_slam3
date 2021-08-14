//
// Created by vahagn on 13.08.21.
//

#include "key_frame_covisibility_updated.h"
namespace orb_slam3 {
namespace messages {

KeyFrameCovisibilityUpdated::KeyFrameCovisibilityUpdated(size_t kf_id,
                                                         const std::vector<frame::KeyFrame *> & covisibles) :
    kf_id(kf_id),
    connected_key_frames(ToVector(covisibles)){

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

}
}