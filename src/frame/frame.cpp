//
// Created by vahagn on 11/05/2021.
//
#include "frame.h"
#include "key_frame.h"

size_t orb_slam3::frame::Frame::next_id_ = 0;

namespace orb_slam3 {
namespace frame {

Frame::Frame(std::istream & stream, serialization::SerializationContext & context) : BaseFrame(stream, context),
                                                                                     reference_keyframe_(nullptr) {

}

void Frame::SetReferenceKeyFrame(KeyFrame * reference_keyframe) {
  reference_keyframe_ = reference_keyframe;
  relative_position_ = GetPosition() * reference_keyframe->GetPosition().GetInversePose();
}

}
}
