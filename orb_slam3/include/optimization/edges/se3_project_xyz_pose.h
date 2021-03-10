//
// Created by vahagn on 03/03/21.
//

#ifndef ORB_SLAM3_SE3_PROJECT_XYZ_POSE_ONLY_H
#define ORB_SLAM3_SE3_PROJECT_XYZ_POSE_ONLY_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <camera/monocular_camera.h>

namespace orb_slam3 {
namespace optimization {
namespace edges {


class SE3ProjectXYZPose : public g2o::BaseBinaryEdge<2, Eigen::Vector2d,
                                                  g2o::VertexSE3Expmap,
                                                  g2o::VertexPointXYZ> {
 public:
  SE3ProjectXYZPose(const orb_slam3::camera::ICamera * camera);
  bool read(std::istream & is) { return false; }

  bool write(std::ostream & os) const { return false; }

  void computeError() override;

//  void linearizeOplus() override;
 private:
  const orb_slam3::camera::ICamera * camera_;

};

}
}
}

#endif //ORB_SLAM3_SE_3_PROJECT_XYZ_POSE_ONLY_H
