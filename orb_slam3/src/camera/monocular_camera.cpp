//
// Created by vahagn on 1/23/21.
//

#include <constants.h>
#include "camera/monocular_camera.h"

namespace orb_slam3 {
namespace camera {

TPoint2D MonocularCamera::Map(const TPoint3D & vector) const {

  const double & fx = this->_estimate[0];
  const double & fy = this->_estimate[1];
  const double & cx = this->_estimate[2];
  const double & cy = this->_estimate[3];

  TPoint2D result;
  double z_inv = 1 / vector[2];
  double x = vector[0] * z_inv;
  double y = vector[1] * z_inv;

  double xd, yd;
  ComputeDistortion(x, y, xd, yd);

  result[0] = xd * fx + cx;
  result[1] = yd * fy + cy;
  return result;
}

void MonocularCamera::UndistortKeyPoints(const std::vector<features::KeyPoint> & keypoints,
                                         std::vector<features::KeyPoint> & out_undistorted_keypoints) const {

  const double & fx = this->_estimate[0];
  const double & fy = this->_estimate[1];
  const double & ifx = 1 / fx;
  const double & ify = 1 / fy;
  const double & cx = this->_estimate[2];
  const double & cy = this->_estimate[3];
  const double & k1 = _estimate[4];
  const double & k2 = _estimate[5];
  const double & p1 = _estimate[6];
  const double & p2 = _estimate[7];
  const double & k3 = _estimate[8];

  out_undistorted_keypoints.resize(keypoints.size());
  for (size_t i = 0; i < keypoints.size(); ++i) {
    const features::KeyPoint & original_key_point = keypoints[i];
    precision_t x = original_key_point.X();
    precision_t y = original_key_point.Y();
    precision_t x0 = x = (x - cx) * ifx;
    precision_t y0 = y = (y - cy) * ify;

    // compensate distortion iteratively
    for (int j = 0; j < 5; j++) {
      double r2 = x * x + y * y;
      double icdist = 1. / (1 + ((k3 * r2 + k2) * r2 + k1) * r2);
      double deltaX = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
      double deltaY = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
      x = (x0 - deltaX) * icdist;
      y = (y0 - deltaY) * icdist;
    }
    out_undistorted_keypoints[i] = keypoints[i];
    out_undistorted_keypoints[i].X() = x * fx + cx;
    out_undistorted_keypoints[i].Y() = y * fy + cy;
  }
}

void MonocularCamera::ComputeImageBounds() {

  std::vector<features::KeyPoint> bounds(4), undistorted_bounds;
  bounds[0].X() = 0;
  bounds[0].Y() = 0;
  bounds[1].X() = 0;
  bounds[1].Y() = height_;
  bounds[2].X() = width_;
  bounds[2].Y() = 0;
  bounds[3].X() = width_;
  bounds[3].Y() = height_;
  UndistortKeyPoints(bounds, undistorted_bounds);

  min_X_ = std::min(undistorted_bounds[0].X(), undistorted_bounds[1].X());
  max_X_ = std::max(undistorted_bounds[2].X(), undistorted_bounds[3].X());
  min_Y_ = std::min(undistorted_bounds[0].Y(), undistorted_bounds[2].Y());
  max_Y_ = std::max(undistorted_bounds[1].Y(), undistorted_bounds[3].Y());

  grid_element_width_inv_ = constants::FRAME_GRID_COLS / (ImageBoundMaxX() - ImageBoundMinX());
  grid_element_height_inv_ = constants::FRAME_GRID_ROWS/ (ImageBoundMaxY() - ImageBoundMinY());
}

// namespace frame

#if DistCoeffsLength == 5
void MonocularCamera::ComputeDistortion(const double x, const double y, double & xd, double & yd) const {
  const double & k1 = _estimate[4];
  const double & k2 = _estimate[5];
  const double & p1 = _estimate[6];
  const double & p2 = _estimate[7];
  const double & k3 = _estimate[8];

  double r2 = x * x + y * y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;

  double cdist = 1 + k1 * r2 + k2 * r4 + k3 * r6;
  double a1 = 2 * x * y;

  xd = x * cdist + p1 * a1 + p2 * (r2 + 2 * x * x);
  yd = y * cdist + p2 * a1 + p1 * (r2 + 2 * y * y);
};

#elif DistCoeffsLength == 8

void MonocularCamera::ComputeDistortion(const double x, const double y, double & xd, double & yd) const {
  const double & k1 = _estimate[4];
  const double & k2 = _estimate[5];
  const double & p1 = _estimate[6];
  const double & p2 = _estimate[7];
  const double & k3 = _estimate[8];
  const double & k4 = _estimate[9];
  const double & k5 = _estimate[10];
  const double & k6 = _estimate[11];

  double r2 = x * x + y * y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;

  double cdist = 1 + k1 * r2 + k2 * r4 + k3 * r6;
  double icdist2 = 1. / (1 + k4 * r2 + k5 * r4 + k6 * r6);
  double a1 = 2 * x * y;

  xd = x * cdist * icdist2 + p1 * a1 + p2 * (r2 + 2 * x * x);
  yd = y * cdist * icdist2 + p2 * a1 + p1 * (r2 + 2 * y * y);
}

#else
#error The DistCoeffsLength should be either 5 or 8
#endif
}
}
