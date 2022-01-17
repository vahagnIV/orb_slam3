//
// Created by vahagn on 08.05.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_FRAME_VERTEX_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_FRAME_VERTEX_H_

// === g2o ===
#include <g2o/types/sba/vertex_se3_expmap.h>

// === orb_slam3 ===
#include <frame/base_frame.h>

namespace orb_slam3 {
namespace optimization {
namespace vertices {


class FrameVertex : public g2o::VertexSE3Expmap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FrameVertex() = default;
  explicit FrameVertex(frame::BaseFrame * frame) : frame_(frame) {
    setEstimate(g2o::SE3Quat(frame->GetStagingPosition().R, frame->GetStagingPosition().T));
  }

  frame::BaseFrame * GetFrame() { return frame_; }
 private:
  frame::BaseFrame * frame_;
};

}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_VERTICES_FRAME_VERTEX_H_
