//
// Created by vahagn on 09/03/2021.
//

#include "optimization/utils/jacobian.h"

namespace orb_slam3 {
namespace optimization {
namespace utils {

void ComputeJacobian(const TPoint3D &pt,
                     const camera::IDistortionModel *distortion_model,
                     ProjectionJacobianType &out_jacobian) {
  const double &x = pt[0];
  const double &y = pt[1];
  const double &z = pt[2];
  const double z_inv = 1 / z;
  const double z_inv2 = z_inv * z_inv;
  ProjectionJacobianType projection_jacobian;
  projection_jacobian << z_inv, 0, -x * z_inv2,
      0, z_inv, -y * z_inv2;
  typename std::remove_pointer<decltype(distortion_model)>::type::JacobianType distortion_jacobian;
  out_jacobian = distortion_jacobian * projection_jacobian;

}

}
}
}