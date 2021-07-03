//
// Created by vahagn on 01/07/2021.
//

#include "sim3_transformation.h"
#include <iostream>

#define WRITE_TO_STREAM(num, stream) stream.write((char *)(&num), sizeof(num));

namespace orb_slam3 {
namespace geometry {

TVector3D Sim3Transformation::Transform(const TPoint3D & point) const {
  return s * (R * point) + T;
}

Sim3Transformation Sim3Transformation::GetInversePose() const {
  precision_t s_inv = 1 / s;
  return Sim3Transformation{.R = R.transpose(), .T = -R.transpose() * T * s_inv, .s = s_inv};
}

void Sim3Transformation::print() const {
  std::cout << "s: " << s << "\nR: \n" << R << "\nT: \n" << T << std::endl;
}

std::ostream & operator<<(std::ostream & stream, const Sim3Transformation & p) {
  stream.write((char *) p.R.data(), p.R.size() * sizeof(decltype(p.R)::Scalar));
  stream.write((char *) p.T.data(), p.T.size() * sizeof(decltype(p.T)::Scalar));
  WRITE_TO_STREAM(p.s, stream);
  return stream;
}

}
}