//
// Created by vahagn on 04/02/21.
//


// == orb-slam3 ===
#include "kannala_brandt_8.h"

namespace orb_slam3 {
namespace camera {

bool KannalaBrandt8::DistortPoint(const HomogenousPoint &undistorted, HomogenousPoint &distorted) const {

  const double x = undistorted[0];
  const double y = undistorted[1];
  double &xd = distorted[0];
  double &yd = distorted[1];

  double r2 = x * x + y * y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;

  double cdist = 1 + K1() * r2 + K2() * r4 + K3() * r6;
  double icdist2 = 1. / (1 + K4() * r2 + K5() * r4 + K6() * r6);
  double a1 = 2 * x * y;

  xd = x * cdist * icdist2 + P1() * a1 + P2() * (r2 + 2 * x * x);
  yd = y * cdist * icdist2 + P2() * a1 + P1() * (r2 + 2 * y * y);
  distorted[2] = 1;
  return true;
}

bool KannalaBrandt8::UnDistortPoint(const HomogenousPoint &distorted, HomogenousPoint &undistorted) const {
  throw std::runtime_error("Kannala brandt8 undistort is not yet implemented");
  return false;
}

void KannalaBrandt8::ComputeJacobian(const TPoint2D &point,
                                     IDistortionModel::JacobianType &out_jacobian) const {

}

}
}