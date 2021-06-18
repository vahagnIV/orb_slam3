//
// Created by vahagn on 03/03/21.
//

#include "se3_project_xyz_pose.h"
// === orb_slam3 ===
#include <map/map_point.h>

namespace orb_slam3 {
namespace optimization {
namespace edges {

SE3ProjectXYZPose::SE3ProjectXYZPose(const camera::MonocularCamera * camera, precision_t threshold)
    : camera_(camera), threshold_(threshold) {
  error()[0] = error()[1] = 0;
}

bool SE3ProjectXYZPose::IsDepthPositive() const {
  auto point = dynamic_cast<g2o::VertexPointXYZ *>(_vertices[1]);
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  return pose->estimate().map(point->estimate())[2] > 0;
}

void SE3ProjectXYZPose::computeError() {
  auto point = dynamic_cast<g2o::VertexPointXYZ *>(_vertices[1]);
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  TPoint3D pt_camera_system = pose->estimate().map(point->estimate());

  TPoint2D distorted;
  camera_->ProjectAndDistort(pt_camera_system, distorted);

  _error = distorted - _measurement;
}

void SE3ProjectXYZPose::linearizeOplus() {
  computeError();
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  auto point = dynamic_cast<g2o::VertexPointXYZ *>(_vertices[1]);
  g2o::Vector3 pt_camera_system = pose->estimate().map(point->estimate());
  const double & x = pt_camera_system[0];
  const double & y = pt_camera_system[1];
  const double & z = pt_camera_system[2];

  camera::ProjectionJacobianType projection_jacobian;
  camera_->ComputeJacobian(pt_camera_system, projection_jacobian);

  Eigen::Matrix<double, 3, 6> se3_jacobian;
  // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
  se3_jacobian << 0.f, z, -y, 1.f, 0.f, 0.f,
      -z, 0.f, x, 0.f, 1.f, 0.f,
      y, -x, 0.f, 0.f, 0.f, 1.f;
  _jacobianOplusXi = projection_jacobian * se3_jacobian;
  _jacobianOplusXj = projection_jacobian * pose->estimate().rotation().toRotationMatrix();
}

bool SE3ProjectXYZPose::IsValid() const {
  return IsDepthPositive() && chi2() < threshold_;
}

}
}
}
