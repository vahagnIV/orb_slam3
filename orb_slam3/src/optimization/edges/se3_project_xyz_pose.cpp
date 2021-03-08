//
// Created by vahagn on 03/03/21.
//

#include <optimization/edges/se3_project_xyz_pose.h>
namespace orb_slam {
namespace optimization {
namespace edges {

void SE3ProjectXYZPose::computeError() {
  auto point = dynamic_cast<g2o::VertexPointXYZ *>(vertex(1));

  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(vertex(0));
  g2o::Vector3 pt_camera_system = pose->estimate().map(point->estimate());
  g2o::Vector3::Scalar inv_z = 1. / pt_camera_system[2];
  _error[0] = _measurement[0] - pt_camera_system[0] * inv_z;
  _error[1] = _measurement[1] - pt_camera_system[1] * inv_z;
}

// https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
void SE3ProjectXYZPose::linearizeOplus() {
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(vertex(0));
  auto point = dynamic_cast<g2o::VertexPointXYZ *>(vertex(1));
  g2o::Vector3 pt_camera_system = pose->estimate().map(point->estimate());
  double x = pt_camera_system[0];
  double y = pt_camera_system[1];
  double z = pt_camera_system[2];
  Eigen::Matrix<double, 3, 6> se3_jacobian;
  se3_jacobian << 0.f, z, -y, 1.f, 0.f, 0.f,
      -z, 0.f, x, 0.f, 1.f, 0.f,
      y, -x, 0.f, 0.f, 0.f, 1.f;

//  _jacobianOplusXi = -pCamera->projectJac(xyz_trans) * se3_jacobian;
//      BaseFixedSizedEdge::linearizeOplus();
}

}
}
}
