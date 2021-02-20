//
// Created by vahagn on 04/02/21.
//


// == orb-slam3 ===
#include "camera/kannala_brandt_8.h"

namespace orb_slam3 {
namespace camera {

KannalaBrandt8::KannalaBrandt8(IDistortionModel<8>::EstimateType * estimate) : IDistortionModel(estimate) {
}

bool KannalaBrandt8::DistortPoint(const HomogenousPoint & undistorted, HomogenousPoint & distorted) {
  const double & k1 = (*estimate_)[4];
  const double & k2 = (*estimate_)[5];
  const double & p1 = (*estimate_)[6];
  const double & p2 = (*estimate_)[7];
  const double & k3 = (*estimate_)[8];
  const double & k4 = (*estimate_)[9];
  const double & k5 = (*estimate_)[10];
  const double & k6 = (*estimate_)[11];

  const double x = undistorted[0];
  const double y = undistorted[1];
  double & xd = distorted[0];
  double & yd = distorted[1];

  double r2 = x * x + y * y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;

  double cdist = 1 + k1 * r2 + k2 * r4 + k3 * r6;
  double icdist2 = 1. / (1 + k4 * r2 + k5 * r4 + k6 * r6);
  double a1 = 2 * x * y;

  xd = x * cdist * icdist2 + p1 * a1 + p2 * (r2 + 2 * x * x);
  yd = y * cdist * icdist2 + p2 * a1 + p1 * (r2 + 2 * y * y);
  distorted[2] = 1;
  return true;
}

bool KannalaBrandt8::UnDistortPoint(const HomogenousPoint & distorted, HomogenousPoint & undistorted) {
  throw std::runtime_error("Kannala brandt8 undistort is not yet implemented");
  return false;

}

}
}