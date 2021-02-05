//
// Created by vahagn on 1/23/21.
//

#include <constants.h>
#include "camera/monocular_camera.h"

namespace orb_slam3 {
namespace camera {

TPoint2D MonocularCamera::Map(const TPoint3D & point3d) const {

  const double & fx = this->_estimate[0];
  const double & fy = this->_estimate[1];
  const double & cx = this->_estimate[2];
  const double & cy = this->_estimate[3];

  TPoint2D result;
  double z_inv = 1 / point3d[2];
  double x = point3d[0] * z_inv;
  double y = point3d[1] * z_inv;

  distortion_model_->DistortPoint(TPoint2D{x, y}, result);

  result[0] = result[0] * fx + cx;
  result[1] = result[1] * fy + cy;
  return result;
}

void MonocularCamera::UndistortPoint(TPoint2D & point, TPoint2D & undistorted_point) const {
  TPoint2D central{(point[0] - Cx()) * fx_inv_, (point[1] - Cy()) * fy_inv_};
  distortion_model_->UnDistortPoint(central, undistorted_point);
  undistorted_point[0] = undistorted_point[0] * Fx() + Cx();
  undistorted_point[1] = undistorted_point[1] * Fy() + Cy();
}

void MonocularCamera::DistortPoint(TPoint2D & undistorted, TPoint2D & distorted) const {
  TPoint2D central{(undistorted[0] - Cx()) * fx_inv_, (undistorted[1] - Cy()) * fy_inv_};
  distortion_model_->DistortPoint(central, distorted);
  distorted[0] = distorted[0] * Fx() + Cx();
  distorted[1] = distorted[1] * Fy() + Cy();
}

void MonocularCamera::ComputeImageBounds() {

  std::vector<TPoint2D> bounds(4), undistorted_bounds(4);
  bounds[0][0] = 0;
  bounds[0][1] = 0;
  bounds[1][0] = 0;
  bounds[1][1] = height_;
  bounds[2][0] = width_;
  bounds[2][1] = 0;
  bounds[3][0] = width_;
  bounds[3][1] = height_;

  UndistortPoint(bounds[0], undistorted_bounds[0]);
  UndistortPoint(bounds[1], undistorted_bounds[1]);
  UndistortPoint(bounds[2], undistorted_bounds[2]);
  UndistortPoint(bounds[3], undistorted_bounds[3]);

  min_X_ = std::min(undistorted_bounds[0][0], undistorted_bounds[1][0]);
  max_X_ = std::max(undistorted_bounds[2][0], undistorted_bounds[3][0]);
  min_Y_ = std::min(undistorted_bounds[0][1], undistorted_bounds[2][1]);
  max_Y_ = std::max(undistorted_bounds[1][1], undistorted_bounds[3][1]);

  grid_element_width_inv_ = constants::FRAME_GRID_COLS / (ImageBoundMaxX() - ImageBoundMinX());
  grid_element_height_inv_ = constants::FRAME_GRID_ROWS / (ImageBoundMaxY() - ImageBoundMinY());
}

}
}
