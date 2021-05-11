//
// Created by vahagn on 12.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_EDGES_SE3_PROJECT_XYZ_POSE_ONLY_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_EDGES_SE3_PROJECT_XYZ_POSE_ONLY_H_

// === g2o ===
#include <g2o/core/base_unary_edge.h>
// === orb-slam3 ===

#include "../../camera/icamera.h"
#include <optimization/vertices/frame_vertex.h>

namespace orb_slam3 {
namespace optimization {
namespace edges {

class SE3ProjectXYZPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, vertices::FrameVertex> {
 public:
  SE3ProjectXYZPoseOnly(const camera::ICamera * camera, const TPoint3D & point);
  void computeError() override;
  void linearizeOplus() override;
  bool read(std::istream & is) override;
  bool write(std::ostream & os) const override;
 private:
  const camera::ICamera * camera_;
  TPoint3D point_;

};

}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_EDGES_SE3_PROJECT_XYZ_POSE_ONLY_H_
