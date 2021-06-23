//
// Created by vahagn on 22/06/2021.
//

#include "pose.h"

namespace orb_slam3 {
namespace geometry {

std::ostream & operator<<(std::ostream & stream, const Pose & p) {
  stream.write((char *) p.R.data(), p.R.size() * sizeof(decltype(p.R)::Scalar));
  stream.write((char *) p.T.data(), p.T.size() * sizeof(decltype(p.T)::Scalar));
  return stream;
}

void Pose::print() const {
  std::cout << R << std::endl << T << std::endl;
}

TVector3D Pose::Transform(const TPoint3D & point) const {
  return s * (R * point) + T;
}

Pose Pose::GetInversePose() const {
  precision_t s_inv = 1 / s;
  return Pose{.R = R.transpose(), .T = -R.transpose() * T * s_inv, .s = s_inv};
}

}
}