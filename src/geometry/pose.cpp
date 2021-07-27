//
// Created by vahagn on 22/06/2021.
//

#include "pose.h"

namespace orb_slam3 {
namespace geometry {

Pose::Pose(Pose && other) : R(std::move(other.R)), T(std::move(other.T)) {
}

Pose::Pose(const Pose & other) : R(other.R), T(other.T) {
}

Pose::Pose() {
}

Pose::Pose(TMatrix33 R, TVector3D T): R(std::move(R)), T(std::move(T)) {

}

std::ostream & operator<<(std::ostream & stream, const Pose & p) {
  stream.write((char *) p.R.data(), p.R.size() * sizeof(decltype(p.R)::Scalar));
  stream.write((char *) p.T.data(), p.T.size() * sizeof(decltype(p.T)::Scalar));
  return stream;
}

Pose & Pose::operator=(Pose && other) {
  R = std::move(other.R);
  T = std::move(other.T);
  return *this;
}

Pose & Pose::operator=(const Pose & other) {
  R = other.R;
  T = other.T;
  return *this;
}

void Pose::print() const {
  std::cout << R << std::endl << T << std::endl;
}

TVector3D Pose::Transform(const TPoint3D & point) const {
  return R * point + T;
}

Pose Pose::GetInversePose() const {
  return Pose(R.transpose(), -R.transpose() * T);
}

}
}