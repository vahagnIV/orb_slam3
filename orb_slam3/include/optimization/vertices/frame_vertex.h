//
// Created by vahagn on 08.05.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_FRAME_VERTEX_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_FRAME_VERTEX_H_

// === g2o ===
#include <g2o/types/sba/vertex_se3_expmap.h>

// === orb_slam3 ===
#include <frame/frame_base.h>

namespace orb_slam3 {
namespace optimization {
namespace vertices {

class FrameVertex : public g2o::VertexSE3Expmap {
 public:
  FrameVertex() = default;
  explicit FrameVertex(frame::FrameBase * frame) : frame_(frame) {
    setEstimate(g2o::SE3Quat(frame->GetPose()->R, frame->GetPose()->T));
    setId(frame->Id());
  }

  frame::FrameBase * GetFrame() { return frame_; }
 private:
  frame::FrameBase * frame_;
};

}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_FRAME_VERTEX_H_
