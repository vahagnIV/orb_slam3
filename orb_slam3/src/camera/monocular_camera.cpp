//
// Created by vahagn on 1/23/21.
//

// == orb-slam3 ===
#include <constants.h>
#include "camera/monocular_camera.h"

namespace orb_slam3 {
namespace camera {

void MonocularCamera::UnprojectPoint(const TPoint2D & point, HomogenousPoint & unprojected) const {
  unprojected << (point[0] - Cx()) * fx_inv_, (point[1] - Cy()) * fy_inv_, 1;
}

void MonocularCamera::ProjectPoint(const TPoint3D & point, TPoint2D & projected) const {
  double z_inv = 1 / point[2];
  projected << point[0] * z_inv * Fx() + Cx(), point[1] * z_inv * Fy() + Cy();
}

bool MonocularCamera::UnprojectAndUndistort(const TPoint2D & point, HomogenousPoint & unprojected) const {
  UnprojectPoint(point, unprojected);
  return distortion_model_->UnDistortPoint(unprojected, unprojected);
}

TPoint2D MonocularCamera::Map(const TPoint3D & point3d) const {

  // TODO: rethink definition
  double z_inv = 1 / point3d[2];
  TPoint3D distorted;
  distortion_model_->DistortPoint(TPoint3D{point3d[0] * z_inv, point3d[2] * z_inv, 1}, distorted);

  TPoint2D result;
  ProjectPoint(distorted, result);
  return result;
}

bool MonocularCamera::UndistortPoint(const TPoint2D & point, TPoint2D & undistorted_point) const {
  TPoint3D unprojected, undistorted;
  UnprojectPoint(point, unprojected);
  if (!distortion_model_->UnDistortPoint(unprojected, undistorted))
    return false;
  ProjectPoint(undistorted, undistorted_point);
  return true;
}

bool MonocularCamera::DistortPoint(const TPoint2D & undistorted, TPoint2D & distorted) const {
  TPoint3D unprojected, distorted_3d;
  UnprojectPoint(undistorted, unprojected);

  if (!distortion_model_->DistortPoint(unprojected, distorted_3d))
    return false;
  ProjectPoint(distorted_3d, distorted);
  return true;
}

void MonocularCamera::ComputeImageBounds() {
  TPoint2D top_left{0, 0}, top_right{width_ - 1, 0}, bottom_left{height_ - 1, 0}, bottom_right{width_ - 1, height_ - 1};
  UndistortPoint(top_left, top_left);
  UndistortPoint(top_right, top_right);
  UndistortPoint(bottom_left, bottom_left);
  UndistortPoint(bottom_right, bottom_right);

  max_X_ = std::max(top_right[0], bottom_right[0]);
  max_Y_ = std::max(bottom_left[1], bottom_right[1]);
  min_X_ = std::min(top_right[0], top_left[0]);
  min_Y_ = std::max(top_left[1], bottom_left[1]);
}

void MonocularCamera::ComputeJacobian(const TPoint3D & pt, ProjectionJacobianType & out_jacobian) const {
  const double & x = pt[0];
  const double & y = pt[1];
  const double & z = pt[2];
  const double z_inv = 1 / z;
  const double z_inv2 = z_inv * z_inv;
  ProjectionJacobianType projection_jacobian;
  projection_jacobian << z_inv, 0, -x * z_inv2,
      0, z_inv, -y * z_inv2;
  IDistortionModel::JacobianType distortion_jacobian;
  TPoint2D projected;
  projected << x * z_inv, y * z_inv;
  distortion_model_->ComputeJacobian(projected, distortion_jacobian);
  out_jacobian = distortion_jacobian * projection_jacobian;
}

void MonocularCamera::ProjectAndDistort(const TPoint3D & point, TPoint2D & out_projected) const {
  double z_inv = 1 / point[2];
  TPoint3D projected{point.x() * z_inv, point.y() * z_inv, 1};
  distortion_model_->DistortPoint(projected, projected);
  out_projected << projected.x() * Fx() + Cx(), projected.y() * Fy() + Cy();
}

bool MonocularCamera::IsInFrustum(const HomogenousPoint & distorted) const {
  return distorted.x() > min_X_ && distorted.x() < max_X_ && distorted.y() > min_Y_ && distorted.y() < max_Y_;
}

bool MonocularCamera::IsInScaleInvarianceRegion(const TPoint3D & point) const {
  //TODO: Implement
  return true;
}

}
}
