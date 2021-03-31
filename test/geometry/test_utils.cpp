//
// Created by vahagn on 31/03/2021.
//

#include "test_utils.h"
namespace orb_slam3 {
namespace test {

TMatrix33 GetRotationMatrixRollPitchYaw(double alpha, double beta, double gamma) {
  TMatrix33 X, Y, Z;
  X << 1, 0, 0,
      0, std::cos(alpha), -std::sin(alpha),
      0, std::sin(alpha), std::cos(alpha);
  Y << std::cos(beta), 0, -std::sin(beta),
      0, 1, 0,
      std::sin(beta), 0, std::cos(beta);
  Z << std::cos(gamma), -std::sin(gamma), 0,
      std::sin(gamma), std::cos(gamma), 0,
      0, 0, 1;

  return X * Y * Z;
}

}
}