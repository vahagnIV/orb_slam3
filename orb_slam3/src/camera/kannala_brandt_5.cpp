//
// Created by vahagn on 04/02/21.
//

// == orb-slam3 ===
#include "camera/kannala_brandt_5.h"

namespace orb_slam3 {
namespace camera {

KannalaBrandt5::KannalaBrandt5(EstimateType * estimate) : IDistortionModel<5>(estimate) {
}

bool KannalaBrandt5::DistortPoint(const HomogenousPoint & undistorted, HomogenousPoint & distorted) {

  const Scalar & x = undistorted[0];
  const Scalar & y = undistorted[1];

  double r2 = x * x + y * y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;

  double cdist = 1 + K1() * r2 + K2() * r4 + K3() * r6;
  double a1 = 2 * x * y;

  distorted[0] = x * cdist + P1() * a1 + P2() * (r2 + 2 * x * x);
  distorted[1] = y * cdist + P2() * a1 + P1() * (r2 + 2 * y * y);
  distorted[2] = 1;
  return true;
}

bool KannalaBrandt5::UnDistortPoint(const HomogenousPoint & distorted, HomogenousPoint & undistorted) {

  undistorted = distorted;

  Scalar & x = undistorted[0];
  Scalar & y = undistorted[1];
  undistorted[2] = 1;
  precision_t x0 = x = distorted[0], y0 = y = distorted[1];

  // compensate distortion iteratively
  for (int j = 0; j < 150; j++) {
    double r2 = x * x + y * y;
    double icdist = 1. / (1 + ((K3() * r2 + K2()) * r2 + K1()) * r2);
    if (icdist < 0)
      return false;
    double deltaX = 2 * P1() * x * y + P2() * (r2 + 2 * x * x);
    double deltaY = P1() * (r2 + 2 * y * y) + 2 * P2() * x * y;
    double xnew = (x0 - deltaX) * icdist;
    double ynew = (y0 - deltaY) * icdist;
    if (std::abs(x - xnew) < 1e-7 && std::abs(y - ynew) < 1e-7)
      return true;
    x = xnew;
    y = ynew;
  }
  undistorted = distorted;
  return false;
}
void KannalaBrandt5::GetTransformationJacobian(const HomogenousPoint &point,
                                               IDistortionModel<5>::JacobianType &out_jacobian) {
  throw std::runtime_error("Kannala brandt8 undistort is not yet implemented");
}

}
}