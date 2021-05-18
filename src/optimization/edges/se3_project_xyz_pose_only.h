//
// Created by vahagn on 12.03.21.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_EDGES_SE3_PROJECT_XYZ_POSE_ONLY_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_EDGES_SE3_PROJECT_XYZ_POSE_ONLY_H_


// === orb-slam3 ===

#include <camera/monocular_camera.h>
#include <optimization/vertices/frame_vertex.h>
#include "ba_unary_edge.h"

namespace orb_slam3 {
namespace optimization {
namespace edges {

class SE3ProjectXYZPoseOnly : public BAUnaryEdge {
 public:
  SE3ProjectXYZPoseOnly(map::MapPoint * map_point, size_t feature_id, const camera::MonocularCamera * camera, const TPoint3D & point);
  void computeError() override;
  void linearizeOplus() override;
  bool read(std::istream & is) override;
  bool write(std::ostream & os) const override;
 private:
  const camera::MonocularCamera * camera_;
  TPoint3D point_;

};

}
}
}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OPTIMIZATION_EDGES_SE3_PROJECT_XYZ_POSE_ONLY_H_
