//
// Created by vahagn on 11/29/20.
//

#include "frame/frame_base.h"

namespace orb_slam3 {
namespace frame {
FrameBase::id_type FrameBase::next_id_ = 0;

void FrameBase::SetPosition(const geometry::Pose & pose) noexcept {
  pose_ = pose;
}

/*
FrameBase::FrameBase() : reference_frame_(nullptr) {

}

void FrameBase::InitializeIdentity() noexcept {
  SetPosition(cv::Matx44f::eye());
}

void FrameBase::SetPosition(const cv::Matx44f &position) noexcept {
  for (int i = 0; i < 3; ++i) {
    T_camera2world_(i) = position(3, i);
    for (int j = 0; j < 3; ++j) {
      R_camera2world_(i, j) = position(i, j);
    }
  }
  R_world2camera_ = R_camera2world_.t();

  T_0world_ = -R_world2camera_ * T_camera2world_;
}

void FrameBase::ComputeBoW() {
  if (bow_vector_.empty()) {
    std::vector<cv::Mat> vCurrentDesc =
Converter::toDescriptorVector(descriptors_);
    orb_vocabulary_->transform(vCurrentDesc, bow_vector_, feature_vector_, 4);
  }
}*/
}
}  // namespace orb_slam3