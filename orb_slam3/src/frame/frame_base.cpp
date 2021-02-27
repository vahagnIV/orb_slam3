//
// Created by vahagn on 11/29/20.
//

// == orb-slam3 ===
#include "frame/frame_base.h"

namespace orb_slam3 {
namespace frame {
FrameBase::id_type FrameBase::next_id_ = 0;

void FrameBase::SetPosition(const geometry::Pose & pose) noexcept {
  pose_ = pose;
}

void FrameBase::InitializeIdentity() noexcept {
  pose_.setToOriginImpl();
}

}
}  // namespace orb_slam3