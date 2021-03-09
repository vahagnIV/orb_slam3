//
// Created by vahagn on 09.03.21.
//

#include "optimization/utils/add_frames.h"
namespace orb_slam3 {

void AddFrames(g2o::SparseOptimizer &optimizer, const std::vector<frame::FrameBase *> &frames) {
  frame::FrameBase *init_frame = frames[0];
  for (auto frame: frames) {
    if (frame->Id() < init_frame->Id())
      init_frame = frame;
    auto pose = frame->GetPose();
    pose->setFixed(false);
    optimizer.addVertex(pose);
  }
  optimizer.vertex(init_frame->Id())->setFixed(true);
}

}