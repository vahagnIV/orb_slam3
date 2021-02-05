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

bool MonocularCamera::UndistortPoint(TPoint2D & point, TPoint2D & undistorted_point) const {
  TPoint2D central{(point[0] - Cx()) * fx_inv_, (point[1] - Cy()) * fy_inv_};
  if(!distortion_model_->UnDistortPoint(central, undistorted_point))
    return false;
  undistorted_point[0] = undistorted_point[0] * Fx() + Cx();
  undistorted_point[1] = undistorted_point[1] * Fy() + Cy();
  return true;
}

bool MonocularCamera::DistortPoint(TPoint2D & undistorted, TPoint2D & distorted) const {
  TPoint2D central{(undistorted[0] - Cx()) * fx_inv_, (undistorted[1] - Cy()) * fy_inv_};
  if(!distortion_model_->DistortPoint(central, distorted))
    return false;
  distorted[0] = distorted[0] * Fx() + Cx();
  distorted[1] = distorted[1] * Fy() + Cy();
  return true;
}

void MonocularCamera::ComputeImageBounds() {

  std::vector<TPoint2D> bounds(4), undistorted_bounds(4);
  max_X_ = std::numeric_limits<decltype(max_X_)>::min();
  min_X_ = std::numeric_limits<decltype(min_X_)>::max();
  max_Y_ = std::numeric_limits<decltype(max_Y_)>::min();
  min_Y_ = std::numeric_limits<decltype(min_Y_)>::max();
  for (size_t i = 0; i < Width(); ++i) {
    for (size_t j = 0; j < Height(); ++j) {
      TPoint2D pt{i,j}, undistorted;
      if(UndistortPoint(pt, undistorted))
      {
        max_X_ = std::max(max_X_, undistorted[0]);
        max_Y_ = std::max(max_Y_, undistorted[1]);
        min_X_ = std::min(min_X_, undistorted[0]);
        min_Y_ = std::min(min_Y_, undistorted[1]);
      }
    }
  }

  grid_element_width_inv_ = constants::FRAME_GRID_COLS / (ImageBoundMaxX() - ImageBoundMinX());
  grid_element_height_inv_ = constants::FRAME_GRID_ROWS / (ImageBoundMaxY() - ImageBoundMinY());
}

}
}
