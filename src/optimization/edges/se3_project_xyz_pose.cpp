//
// Created by vahagn on 03/03/21.
//

#include "se3_project_xyz_pose.h"
// === orb_slam3 ===
#include <map/map_point.h>

namespace orb_slam3 {
namespace optimization {
namespace edges {

SE3ProjectXYZPose::SE3ProjectXYZPose(const camera::MonocularCamera *camera, precision_t threshold)
    : camera_(camera), threshold_(threshold), step_(0) {
  error()[0] = error()[1] = 0;
}

bool SE3ProjectXYZPose::IsDepthPositive() const {
  auto point = dynamic_cast<g2o::VertexPointXYZ *>(_vertices[1]);
  auto pose = dynamic_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  return pose->estimate().map(point->estimate()).z() > 1e-4;
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

  ++step_;
  auto pose = dynamic_cast<vertices::FrameVertex *>(_vertices[0]);
  auto point = dynamic_cast<vertices::MapPointVertex *>(_vertices[1]);

  g2o::Vector3 pt_camera_system = pose->estimate().map(point->estimate());
  const double &x = pt_camera_system.x();
  const double &y = pt_camera_system.y();
  const double &z = pt_camera_system.z();
  if (z == 0) {
    std::cout << "Frame id " << pose->GetFrame()->Id() << std::endl;;
    TMatrix33 r = pose->estimate().rotation().toRotationMatrix();
    TVector3D t = pose->estimate().translation();
    TPoint3D pt = point->estimate();
    std::cout << "Estimate:\n" << pt << std::endl;
    std::cout << "Original:\n" << point->GetMapPoint()->GetPosition() << std::endl;

    std::cout << "Frame estimate R:\n" << r << std::endl;
    std::cout << "Frame estimate T:\n" << t << std::endl;
    std::cout << "Frame origin R:\n" << pose->GetFrame()->GetPosition().R;
    std::cout << "Frame origin T:\n" << pose->GetFrame()->GetPosition().T;
    std::cout << "Frame origin staging R:\n" << pose->GetFrame()->GetStagingPosition().R;
    std::cout << "Frame origin staging T:\n" << pose->GetFrame()->GetStagingPosition().T;
    throw std::runtime_error("fff");
  }

  camera::ProjectionJacobianType projection_jacobian;
  camera_->ComputeJacobian(pt_camera_system, projection_jacobian);

  Eigen::Matrix<double, 3, 6> se3_jacobian;
  // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
  se3_jacobian << 0.f, z, -y, 1.f, 0.f, 0.f,
      -z, 0.f, x, 0.f, 1.f, 0.f,
      y, -x, 0.f, 0.f, 0.f, 1.f;
  Eigen::Matrix<precision_t, 2, 6> Xi = projection_jacobian * se3_jacobian;;
  Eigen::Matrix<precision_t, 2, 3> Xj = projection_jacobian * pose->estimate().rotation().toRotationMatrix();
  BABinaryEdge::linearizeOplus();
  std::cout << _jacobianOplusXi << std::endl;
  std::cout << Xi << std::endl;
  Eigen::Matrix<precision_t, 2, 6> deltai = _jacobianOplusXi - Xi;
  Eigen::Matrix<precision_t, 2, 3> deltaj = _jacobianOplusXj - Xj;
  std::cout << deltai << std::endl;
  std::cout << deltaj << std::endl;
  precision_t di = std::abs((deltai).sum()) ;
  std::cout << "Di = " << di << std::endl;
  if (di > 1e-2)
    throw std::runtime_error("ppp");
  if (std::abs((deltaj).sum()) > 1e-2)
    throw std::runtime_error("ppp");
//  _jacobianOplusXi = projection_jacobian * se3_jacobian;
//  _jacobianOplusXj = projection_jacobian * pose->estimate().rotation().toRotationMatrix();
}

bool SE3ProjectXYZPose::IsValid() const {
  return IsDepthPositive() && chi2() < threshold_;
}

}
}
}
