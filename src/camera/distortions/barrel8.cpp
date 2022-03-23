//
// Created by vahagn on 04/02/21.
//


// == orb-slam3 ===
#include "barrel8.h"

namespace orb_slam3 {
namespace camera {

Barrel8::Barrel8(): k1_(0), k2_(0), k3_(0), k4_(0), k5_(0), k6_(0), p1_(0), p2_(0) {

}

Barrel8::Barrel8(std::istream &istream, serialization::SerializationContext &context) {
  READ_FROM_STREAM(k1_, istream);
  READ_FROM_STREAM(k2_, istream);
  READ_FROM_STREAM(p1_, istream);
  READ_FROM_STREAM(p2_, istream);
  READ_FROM_STREAM(k3_, istream);
  READ_FROM_STREAM(k4_, istream);
  READ_FROM_STREAM(k5_, istream);
  READ_FROM_STREAM(k6_, istream);
}

DistortionModelType Barrel8::Type() {
  return DistortionModelType::BARREL8;
}

bool Barrel8::DistortPoint(const HomogenousPoint & undistorted, HomogenousPoint & distorted) const {

  const double x = undistorted[0];
  const double y = undistorted[1];
  double & xd = distorted[0];
  double & yd = distorted[1];

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

bool Barrel8::UnDistortPoint(const HomogenousPoint & distorted, HomogenousPoint & undistorted) const {
  throw std::runtime_error("Kannala brandt8 undistort is not yet implemented");
  return false;
}

void Barrel8::ComputeJacobian(const TPoint2D & point,
                              IDistortionModel::JacobianType & out_jacobian) const {

}

void Barrel8::Serialize(std::ostream & ostream) const {
  WRITE_TO_STREAM(k1_, ostream);
  WRITE_TO_STREAM(k2_, ostream);
  WRITE_TO_STREAM(p1_, ostream);
  WRITE_TO_STREAM(p2_, ostream);
  WRITE_TO_STREAM(k3_, ostream);
  WRITE_TO_STREAM(k4_, ostream);
  WRITE_TO_STREAM(k5_, ostream);
  WRITE_TO_STREAM(k6_, ostream);
}
void Barrel8::ComputeGradientAndHessian(const HomogenousPoint &point, TVector3D &out_gradient, TMatrix33 &out_hessian) {

}

}
}