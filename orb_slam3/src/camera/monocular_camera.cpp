//
// Created by vahagn on 1/23/21.
//

#include <constants.h>
#include "camera/monocular_camera.h"

namespace orb_slam3 {
namespace camera {

void MonocularCamera::UnprojectPoint(TPoint2D & point, TPoint3D & unprojected) const {
  unprojected << (point[0] - Cx()) * fx_inv_, (point[1] - Cy()) * fy_inv_, 1;
}

void MonocularCamera::ProjectPoint(TPoint3D & point, TPoint2D & projected) const {
  double z_inv = 1 / point[2];
  projected << point[0] * z_inv * Fx() + Cx(), point[1] * z_inv * Fy() + Cy();
}

bool MonocularCamera::UnprojectAndUndistort(TPoint2D & point, TPoint3D & unprojected) const {
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

bool MonocularCamera::UndistortPoint(TPoint2D & point, TPoint2D & undistorted_point) const {
  TPoint3D unprojected, undistorted;
  UnprojectPoint(point, unprojected);
  if (!distortion_model_->UnDistortPoint(unprojected, undistorted))
    return false;
  ProjectPoint(undistorted, undistorted_point);
  return true;
}

bool MonocularCamera::DistortPoint(TPoint2D & undistorted, TPoint2D & distorted) const {
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

}
}
