//
// Created by vahagn on 03/03/21.
//

#include <optimization/edges/se3_project_xyz_pose.h>
namespace orb_slam3 {
namespace optimization {
namespace edges {

SE3ProjectXYZPose::SE3ProjectXYZPose(const orb_slam3::camera::ICamera *camera) : camera_(camera) {
  error()[0] = error()[1] = 0;
}

void SE3ProjectXYZPose::computeError() {
  auto point = dynamic_cast<g2o::VertexPointXYZ *>(_vertices[1]);
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  auto x = point->estimate()[0];
  auto y = point->estimate()[1];
  auto z = point->estimate()[2];
  g2o::Vector3 pt_camera_system = pose->estimate().map(point->estimate());
//  camera_->
  g2o::Vector3::Scalar inv_z = 1. / pt_camera_system[2];
  _error[0] = pt_camera_system[0] * inv_z - _measurement[0];
  _error[1] = pt_camera_system[1] * inv_z - _measurement[1];
}

// https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
/*void SE3ProjectXYZPose::linearizeOplus() {
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  auto point = dynamic_cast<g2o::VertexPointXYZ *>(_vertices[1]);
  g2o::Vector3 pt_camera_system = pose->estimate().map(point->estimate());
  double x = pt_camera_system[0];
  double y = pt_camera_system[1];
  double z = pt_camera_system[2];
  Eigen::Matrix<double, 3, 6> se3_jacobian;
  se3_jacobian << 0.f, z, -y, 1.f, 0.f, 0.f,
      -z, 0.f, x, 0.f, 1.f, 0.f,
      y, -x, 0.f, 0.f, 0.f, 1.f;
  camera::ProjectionJacobianType projection_jacobian;
  camera_->ComputeJacobian(point->estimate(), projection_jacobian);
  _jacobianOplusXi = -projection_jacobian * se3_jacobian;
//      BaseFixedSizedEdge::linearizeOplus();
}*/

}
}
}
