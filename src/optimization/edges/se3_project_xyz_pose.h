//
// Created by vahagn on 03/03/21.
//

#ifndef ORB_SLAM3_SE3_PROJECT_XYZ_POSE_ONLY_H
#define ORB_SLAM3_SE3_PROJECT_XYZ_POSE_ONLY_H

#include <g2o/core/base_binary_edge.h>
#include "../../camera/monocular_camera.h"

#include <optimization/vertices/frame_vertex.h>
#include <optimization/vertices/map_point_vertex.h>

namespace orb_slam3 {
namespace optimization {
namespace edges {


class SE3ProjectXYZPose : public g2o::BaseBinaryEdge<2, Eigen::Vector2d,
                                                  vertices::FrameVertex,
                                                  vertices::MapPointVertex> {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit SE3ProjectXYZPose(const orb_slam3::camera::ICamera * camera);
  bool read(std::istream & is) { return false; }

  bool write(std::ostream & os) const { return false; }

  void computeError() override;

  bool IsDepthPositive();

  void linearizeOplus() override;
 private:
  const orb_slam3::camera::ICamera * camera_;

};

}
}
}

#endif //ORB_SLAM3_SE_3_PROJECT_XYZ_POSE_ONLY_H
