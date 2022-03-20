//
// Created by vahagn on 03/03/21.
//

#ifndef ORB_SLAM3_SE3_PROJECT_XYZ_POSE_ONLY_H
#define ORB_SLAM3_SE3_PROJECT_XYZ_POSE_ONLY_H

#include <camera/monocular_camera.h>

#include <optimization/vertices/frame_vertex.h>
#include <optimization/vertices/map_point_vertex.h>

#include <optimization/edges/ba_binary_edge.h>

namespace orb_slam3 {
namespace optimization {
namespace edges {

class SE3ProjectXYZPose : public BABinaryEdge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit SE3ProjectXYZPose(const camera::MonocularCamera  * camera, precision_t threshold);
  bool read(std::istream & is) { return false; }

  bool write(std::ostream & os) const { return false; }

  void computeError() override;

  bool IsDepthPositive() const override;

  void linearizeOplus() override;

  bool IsValid() const override;
 private:
  const camera::MonocularCamera * camera_;
  precision_t threshold_;
  unsigned step_;

};

}
}
}

#endif //ORB_SLAM3_SE_3_PROJECT_XYZ_POSE_ONLY_H
