//
// Created by vahagn on 04/02/21.
//

#include "camera/kannala_brandt_5.h"
namespace orb_slam3 {
namespace camera {

KannalaBrandt5::KannalaBrandt5(EstimateType * estimate) : IDistortionModel<5>(estimate) {
}

bool KannalaBrandt5::DistortPoint(const TPoint2D & undistorted, TPoint2D & distorted) {
  const double & k1 = (*estimate_)[4];
  const double & k2 = (*estimate_)[5];
  const double & p1 = (*estimate_)[6];
  const double & p2 = (*estimate_)[7];
  const double & k3 = (*estimate_)[8];

  const double & x = undistorted[0];
  const double & y = undistorted[1];

  double r2 = x * x + y * y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;

  double cdist = 1 + k1 * r2 + k2 * r4 + k3 * r6;
  double a1 = 2 * x * y;

  distorted[0] = x * cdist + p1 * a1 + p2 * (r2 + 2 * x * x);
  distorted[1] = y * cdist + p2 * a1 + p1 * (r2 + 2 * y * y);
  return true;
}

bool KannalaBrandt5::UnDistortPoint(const TPoint2D & distorted, TPoint2D & undistorted) {
  const double & k1 = (*estimate_)[4];
  const double & k2 = (*estimate_)[5];
  const double & p1 = (*estimate_)[6];
  const double & p2 = (*estimate_)[7];
  const double & k3 = (*estimate_)[8];
  undistorted = distorted;

  Scalar & x = undistorted[0];
  Scalar & y = undistorted[1];
  precision_t x0 = x, y0 = y;

  // compensate distortion iteratively
  for (int j = 0; j < 5; j++) {
    double r2 = x * x + y * y;
    double icdist = 1. / (1 + ((k3 * r2 + k2) * r2 + k1) * r2);
    double deltaX = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
    double deltaY = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
    x = (x0 - deltaX) * icdist;
    y = (y0 - deltaY) * icdist;
  }
  return true;
}

}
}