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
bool SE3ProjectXYZPose::IsDepthPositive() {
  auto point = dynamic_cast<g2o::VertexPointXYZ *>(_vertices[1]);
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  return pose->estimate().map(point->estimate())[2] > 0;
}

void SE3ProjectXYZPose::computeError() {
  auto point = dynamic_cast<g2o::VertexPointXYZ *>(_vertices[1]);
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  g2o::Vector3 pt_camera_system = pose->estimate().map(point->estimate());
  g2o::Vector3::Scalar inv_z = 1. / pt_camera_system[2];
  HomogenousPoint projected = pt_camera_system * inv_z, distorted;

  camera_->GetDistortionModel()->DistortPoint(projected, distorted);
  _error[0] = distorted[0] - _measurement[0];
  _error[1] = distorted[1] - _measurement[1];
}

void SE3ProjectXYZPose::linearizeOplus() {
  BaseFixedSizedEdge::linearizeOplus();
      return;
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  auto point = dynamic_cast<g2o::VertexPointXYZ *>(_vertices[1]);
  g2o::Vector3 pt_camera_system = pose->estimate().map(point->estimate());
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

  if (point->fixed())
    _jacobianOplusXj.setZero();
  else
    _jacobianOplusXj = projection_jacobian * pose->estimate().rotation().toRotationMatrix();
//  Eigen::Matrix<double, 2, 6> jacobianOplusXi = projection_jacobian * se3_jacobian;
//  Eigen::Matrix<double, 2, 3> jacobianOplusXj = projection_jacobian * pose->estimate().rotation().toRotationMatrix();
//  if (pose->fixed())
//    jacobianOplusXi.setZero();
//
//
//  std::cout << jacobianOplusXi << std::endl;
//  std::cout << _jacobianOplusXi << std::endl;
//  std::cout << jacobianOplusXj << std::endl;
//  std::cout << _jacobianOplusXj << std::endl;
}

}
}
}
