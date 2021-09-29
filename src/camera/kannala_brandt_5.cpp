//
// Created by vahagn on 04/02/21.
//

// == orb-slam3 ===
#include "kannala_brandt_5.h"
#include <opencv2/opencv.hpp>

namespace orb_slam3 {
namespace camera {

bool KannalaBrandt5::DistortPoint(const HomogenousPoint &undistorted, HomogenousPoint &distorted) const {

  const precision_t &x = undistorted[0];
  const precision_t &y = undistorted[1];

  precision_t r2 = x * x + y * y;
  precision_t r4 = r2 * r2;
  precision_t r6 = r4 * r2;

  precision_t cdist = 1 + K1() * r2 + K2() * r4 + K3() * r6;
  precision_t a1 = 2 * x * y;

  distorted[0] = x * cdist + P1() * a1 + P2() * (r2 + 2 * x * x);
  distorted[1] = y * cdist + P2() * a1 + P1() * (r2 + 2 * y * y);
  distorted[2] = 1;
  return true;
}

bool KannalaBrandt5::UnDistortPoint(const HomogenousPoint &distorted, HomogenousPoint &undistorted) const {

  undistorted = distorted;

  precision_t &x = undistorted[0];
  precision_t &y = undistorted[1];
  undistorted[2] = 1;
  precision_t x0 = x = distorted[0], y0 = y = distorted[1];

  // compensate distortion iteratively
  for (int j = 0; j < 10; j++) {
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

void KannalaBrandt5::ComputeJacobian(const TPoint2D &point,
                                     IDistortionModel::JacobianType &out_jacobian) const {
  const precision_t &x = point[0];
  const precision_t &y = point[1];

  precision_t r2 = x * x + y * y;
  precision_t r4 = r2 * r2;
  precision_t r6 = r4 * r2;

  precision_t cdist = 1 + K1() * r2 + K2() * r4 + K3() * r6;
  precision_t a1 = 2 * x * y;
  precision_t Dcdist = K1() + 2 * K2() * r2 + 3 * K3() * r4;
  out_jacobian << cdist + 2 * x * x * Dcdist + 2 * y * P1() + 6 * x * P2(), a1 * Dcdist + 2 * x * P1() + 2 * y * P2(),
      a1 * Dcdist + 2 * y * P2() + 2 * x * P1(), cdist + 2 * y * y * Dcdist + 2 * x * P2() + 6 * y * P1();
}

void KannalaBrandt5::Serialize(std::ostream & ostream) const {
  WRITE_TO_STREAM(k1_, ostream);
  WRITE_TO_STREAM(k2_, ostream);
  WRITE_TO_STREAM(p1_, ostream);
  WRITE_TO_STREAM(p2_, ostream);
  WRITE_TO_STREAM(k3_, ostream);
}

}
}