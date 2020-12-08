//
// Created by vahagn on 11/29/20.
//

#include "frame_base.h"

namespace nvision{

void FrameBase::InitializeIdentity() noexcept {
  current_pose_ = cv::Matx44f::eye();
}

}