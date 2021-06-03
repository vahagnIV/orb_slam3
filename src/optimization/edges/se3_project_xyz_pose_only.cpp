//
// Created by vahagn on 12.03.21.
//

#include "se3_project_xyz_pose_only.h"

#include <utility>
namespace orb_slam3 {
namespace optimization {
namespace edges {

SE3ProjectXYZPoseOnly::SE3ProjectXYZPoseOnly(map::MapPoint * map_point,
                                             size_t feature_id,
                                             const camera::MonocularCamera * camera,
                                             TPoint3D  point)
    : BAUnaryEdge(map_point, feature_id), camera_(camera), point_(std::move(point)) {
}

void SE3ProjectXYZPoseOnly::computeError() {
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  TPoint3D pt_camera_system = pose->estimate().map(point_);
  TPoint2D distorted;
  camera_->ProjectAndDistort(pt_camera_system, distorted);
  _error = distorted - _measurement;
}

bool SE3ProjectXYZPoseOnly::read(std::istream & is) {
  return false;
}

bool SE3ProjectXYZPoseOnly::write(std::ostream & os) const {
  return false;
}

bool SE3ProjectXYZPoseOnly::IsValid(precision_t threshold) {
  return chi2() < threshold && IsDepthPositive();
}

bool SE3ProjectXYZPoseOnly::IsDepthPositive() {
  return dynamic_cast<vertices::FrameVertex *>(vertex(0))->estimate().map(point_).z() > 0;
}

void SE3ProjectXYZPoseOnly::linearizeOplus() {
//  BaseFixedSizedEdge::linearizeOplus();
//  return;
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  g2o::Vector3 pt_camera_system = pose->estimate().map(point_);
  const double & x = pt_camera_system[0];
  const double & y = pt_camera_system[1];
  const double & z = pt_camera_system[2];

  camera::ProjectionJacobianType projection_jacobian;
  camera_->ComputeJacobian(pt_camera_system, projection_jacobian);

  if (pose->fixed())
    _jacobianOplusXi.setZero();
  else {
    Eigen::Matrix<double, 3, 6> se3_jacobian;
    // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    se3_jacobian << 0.f, z, -y, 1.f, 0.f, 0.f,
        -z, 0.f, x, 0.f, 1.f, 0.f,
        y, -x, 0.f, 0.f, 0.f, 1.f;
    _jacobianOplusXi = projection_jacobian * se3_jacobian;
  }
}
/*
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  g2o::Vector3 pt_camera_system = pose->estimate().map(point_);
  const double &x = pt_camera_system[0];
  const double &y = pt_camera_system[1];
  const double &z = pt_camera_system[2];

  camera::ProjectionJacobianType projection_jacobian;
  camera_->ComputeJacobian(pt_camera_system, projection_jacobian);

  if (pose->fixed())
    _jacobianOplusXi.setZero();
  else {
    Eigen::Matrix<double, 3, 6> se3_jacobian;
    // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    se3_jacobian << 0.f, z, -y, 1.f, 0.f, 0.f,
        -z, 0.f, x, 0.f, 1.f, 0.f,
        y, -x, 0.f, 0.f, 0.f, 1.f;
    _jacobianOplusXi = projection_jacobian * se3_jacobian;
  }*/
}
}
}